#!/usr/bin/env python3
"""
Combined Actuator Control + PT Plots GUI (test_guis)

Left panel:  Control actuators on the actuator board (ADC_Testing/Actuator_Testing style).
Right panel: Live PT voltage plots from PT board (PT_BOARD_Multi style).

Uses DAQv2-Comms over UDP. Firmware on both boards unchanged except IP/port:
  - Actuator board: listen on 5005, send sensor data to this PC on actuator_receive_port (default 5006).
  - PT board: send SENSOR_DATA to this PC on pt_receive_port (default 5007).

  pip install -r requirements.txt
  python combined_actuator_pt_gui.py
"""

import os
import socket
import struct
import sys
import time
import json
from typing import Optional, Tuple, List, Dict
from collections import deque
from pathlib import Path

# Prefer PyQt5 on macOS (PyQt6 can segfault with pyqtgraph)
if sys.platform == "darwin":
    os.environ.setdefault("QT_MAC_WANTS_LAYER", "1")
    try:
        from PyQt5 import QtCore, QtGui, QtWidgets
        _QT_HORIZ = QtCore.Qt.Horizontal
    except ImportError:
        from PyQt6 import QtCore, QtGui, QtWidgets
        _QT_HORIZ = QtCore.Qt.Orientation.Horizontal
else:
    try:
        from PyQt6 import QtCore, QtGui, QtWidgets
        _QT_HORIZ = QtCore.Qt.Orientation.Horizontal
    except ImportError:
        from PyQt5 import QtCore, QtGui, QtWidgets
        _QT_HORIZ = QtCore.Qt.Horizontal

import pyqtgraph as pg
import numpy as np

pg.setConfigOptions(antialias=False, useOpenGL=False)

# ---------------------------------------------------------------------------
# DAQv2-Comms protocol
# ---------------------------------------------------------------------------
DIABLO_COMMS_VERSION = 0
MAX_PACKET_SIZE = 512


class PacketType:
    BOARD_HEARTBEAT = 1
    SERVER_HEARTBEAT = 2
    SENSOR_DATA = 3
    ACTUATOR_COMMAND = 4
    SENSOR_CONFIG = 5
    ACTUATOR_CONFIG = 6
    ABORT = 7
    ABORT_DONE = 8
    CLEAR_ABORT = 9


PACKET_HEADER_FORMAT = "<BBI"
PACKET_HEADER_SIZE = 6
ACTUATOR_COMMAND_PACKET_FORMAT = "<B"
ACTUATOR_COMMAND_PACKET_SIZE = 1
ACTUATOR_COMMAND_FORMAT = "<BB"
ACTUATOR_COMMAND_SIZE = 2
SENSOR_DATA_PACKET_FORMAT = "<BB"
SENSOR_DATA_PACKET_SIZE = 2
SENSOR_DATA_CHUNK_FORMAT = "<I"
SENSOR_DATA_CHUNK_SIZE = 4
SENSOR_DATAPOINT_FORMAT = "<Bf"
SENSOR_DATAPOINT_SIZE = 5

# Defaults
DEFAULT_ACTUATOR_IP = "192.168.2.100"
DEFAULT_ACTUATOR_PORT = 5005
DEFAULT_ACTUATOR_RECEIVE_PORT = 5006
DEFAULT_PT_RECEIVE_PORT = 5007
NUM_ACTUATORS = 10
NUM_PTS = 10
UPDATE_INTERVAL_MS = 50
ACTUATOR_UPDATE_MS = 100
DEFAULT_PT_WINDOW_SECONDS = 30.0
MAX_POINTS = 8000
ACTUATOR_LABELS_FILE = Path(__file__).parent / "actuator_labels.json"

PT_COLORS = [
    (255, 80, 80), (80, 255, 80), (80, 150, 255), (255, 200, 80),
    (200, 80, 255), (80, 255, 255), (255, 150, 150), (150, 255, 150),
    (150, 200, 255), (255, 255, 80),
]


def create_actuator_command_packet(commands: List[Tuple[int, int]]) -> bytes:
    if len(commands) == 0 or len(commands) > 255:
        return b""
    header_size = PACKET_HEADER_SIZE
    body_size = ACTUATOR_COMMAND_PACKET_SIZE
    commands_size = len(commands) * ACTUATOR_COMMAND_SIZE
    total_size = header_size + body_size + commands_size
    if total_size > MAX_PACKET_SIZE:
        return b""
    packet = bytearray(total_size)
    offset = 0
    struct.pack_into(PACKET_HEADER_FORMAT, packet, offset, PacketType.ACTUATOR_COMMAND, DIABLO_COMMS_VERSION, int(time.time() * 1000) & 0xFFFFFFFF)
    offset += PACKET_HEADER_SIZE
    struct.pack_into(ACTUATOR_COMMAND_PACKET_FORMAT, packet, offset, len(commands))
    offset += ACTUATOR_COMMAND_PACKET_SIZE
    for actuator_id, actuator_state in commands:
        struct.pack_into(ACTUATOR_COMMAND_FORMAT, packet, offset, actuator_id, actuator_state)
        offset += ACTUATOR_COMMAND_SIZE
    return bytes(packet)


def parse_packet_header(data: bytes) -> Optional[Tuple[int, int, int]]:
    if len(data) < PACKET_HEADER_SIZE:
        return None
    try:
        return struct.unpack(PACKET_HEADER_FORMAT, data[:PACKET_HEADER_SIZE])
    except struct.error:
        return None


def parse_sensor_data_packet(data: bytes) -> Optional[Tuple[dict, List[dict]]]:
    if len(data) < PACKET_HEADER_SIZE + SENSOR_DATA_PACKET_SIZE:
        return None
    header = parse_packet_header(data)
    if header is None or header[0] != PacketType.SENSOR_DATA:
        return None
    packet_type, version, timestamp = header
    offset = PACKET_HEADER_SIZE
    try:
        num_chunks, num_sensors = struct.unpack(SENSOR_DATA_PACKET_FORMAT, data[offset : offset + SENSOR_DATA_PACKET_SIZE])
    except struct.error:
        return None
    offset += SENSOR_DATA_PACKET_SIZE
    per_chunk = SENSOR_DATA_CHUNK_SIZE + num_sensors * SENSOR_DATAPOINT_SIZE
    if len(data) < PACKET_HEADER_SIZE + SENSOR_DATA_PACKET_SIZE + num_chunks * per_chunk:
        return None
    chunks = []
    for _ in range(num_chunks):
        chunk_ts, = struct.unpack(SENSOR_DATA_CHUNK_FORMAT, data[offset : offset + SENSOR_DATA_CHUNK_SIZE])
        offset += SENSOR_DATA_CHUNK_SIZE
        datapoints = []
        for _ in range(num_sensors):
            sid, val = struct.unpack(SENSOR_DATAPOINT_FORMAT, data[offset : offset + SENSOR_DATAPOINT_SIZE])
            datapoints.append({"sensor_id": sid, "data": val})
            offset += SENSOR_DATAPOINT_SIZE
        chunks.append({"timestamp": chunk_ts, "datapoints": datapoints})
    return ({"packet_type": packet_type, "version": version, "timestamp": timestamp}, chunks)


# ---------------------------------------------------------------------------
# UDP receiver for actuator sensor data (current/voltage)
# ---------------------------------------------------------------------------
class ActuatorUDPReceiver(QtCore.QThread):
    sensor_data_received = QtCore.pyqtSignal(dict, list)
    status_update = QtCore.pyqtSignal(str)

    def __init__(self, port: int, bind_address: str = "0.0.0.0"):
        super().__init__()
        self.port = port
        self.bind_address = bind_address
        self._stop = False
        self.sock = None

    def stop(self):
        self._stop = True
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass

    def run(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(0.1)
        try:
            self.sock.bind((self.bind_address, self.port))
            self.status_update.emit(f"Actuator data: {self.bind_address}:{self.port}")
        except OSError as e:
            self.status_update.emit(f"Actuator bind error: {e}")
            return
        while not self._stop:
            try:
                data, _ = self.sock.recvfrom(MAX_PACKET_SIZE)
                header = parse_packet_header(data)
                if header is not None and header[0] == PacketType.SENSOR_DATA:
                    result = parse_sensor_data_packet(data)
                    if result:
                        self.sensor_data_received.emit(result[0], result[1])
            except socket.timeout:
                continue
            except Exception as e:
                if not self._stop:
                    self.status_update.emit(f"Actuator recv: {e}")
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        self.status_update.emit("Actuator receiver stopped")


# ---------------------------------------------------------------------------
# UDP receiver for PT sensor data
# ---------------------------------------------------------------------------
class PTUDPReceiver(QtCore.QThread):
    sensor_data_received = QtCore.pyqtSignal(dict, list)
    status_update = QtCore.pyqtSignal(str)

    def __init__(self, port: int, bind_address: str = "0.0.0.0"):
        super().__init__()
        self.port = port
        self.bind_address = bind_address
        self._stop = False
        self.sock = None
        self.total_packets = 0
        self.total_bytes = 0
        self.start_time = None

    def stop(self):
        self._stop = True
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass

    def get_stats(self) -> Dict:
        if self.start_time is None:
            return {"packets": 0, "bytes": 0, "packets_per_sec": 0.0, "bytes_per_sec": 0.0}
        elapsed = time.time() - self.start_time
        return {
            "packets": self.total_packets,
            "bytes": self.total_bytes,
            "packets_per_sec": self.total_packets / elapsed if elapsed > 0 else 0.0,
            "bytes_per_sec": self.total_bytes / elapsed if elapsed > 0 else 0.0,
        }

    def run(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(0.1)
        try:
            self.sock.bind((self.bind_address, self.port))
            self.status_update.emit(f"PT data: {self.bind_address}:{self.port}")
            self.start_time = time.time()
        except OSError as e:
            self.status_update.emit(f"PT bind error: {e}")
            return
        while not self._stop:
            try:
                data, _ = self.sock.recvfrom(MAX_PACKET_SIZE)
                self.total_packets += 1
                self.total_bytes += len(data)
                header = parse_packet_header(data)
                if header is not None and header[0] == PacketType.SENSOR_DATA:
                    result = parse_sensor_data_packet(data)
                    if result:
                        self.sensor_data_received.emit(result[0], result[1])
            except socket.timeout:
                continue
            except Exception as e:
                if not self._stop:
                    self.status_update.emit(f"PT recv: {e}")
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        self.status_update.emit("PT receiver stopped")


# ---------------------------------------------------------------------------
# Left panel: Actuator control
# ---------------------------------------------------------------------------
class ActuatorPanel(QtWidgets.QWidget):
    def __init__(self, device_ip: str, device_port: int, receive_port: int, bind_address: str, parent=None):
        super().__init__(parent)
        self.device_ip = device_ip
        self.device_port = device_port
        self.receive_port = receive_port
        self.bind_address = bind_address
        self.actuator_states = [0] * NUM_ACTUATORS
        self.voltage_readings = [0.0] * NUM_ACTUATORS
        self.command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receiver = None
        self.actuator_widgets = []
        self.actuator_labels = self.load_labels()
        self.init_ui()
        self.start_receiver()

    def load_labels(self) -> Dict[int, str]:
        """Load actuator labels from JSON file"""
        if ACTUATOR_LABELS_FILE.exists():
            try:
                with open(ACTUATOR_LABELS_FILE, 'r') as f:
                    labels = json.load(f)
                    # Convert string keys to int and ensure all actuators have labels
                    result = {}
                    for i in range(NUM_ACTUATORS):
                        actuator_id = i + 1
                        key = str(actuator_id)
                        result[actuator_id] = labels.get(key, "")
                    return result
            except Exception as e:
                print(f"Error loading labels: {e}")
        return {i + 1: "" for i in range(NUM_ACTUATORS)}

    def save_labels(self):
        """Save actuator labels to JSON file"""
        try:
            labels_dict = {str(k): v for k, v in self.actuator_labels.items()}
            with open(ACTUATOR_LABELS_FILE, 'w') as f:
                json.dump(labels_dict, f, indent=2)
        except Exception as e:
            print(f"Error saving labels: {e}")

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        self.status_label = QtWidgets.QLabel("Starting...")
        self.status_label.setStyleSheet("font-weight: bold; padding: 5px;")
        layout.addWidget(self.status_label)
        grid_container = QtWidgets.QWidget()
        grid_layout = QtWidgets.QGridLayout(grid_container)
        grid_layout.setSpacing(8)
        for i in range(NUM_ACTUATORS):
            actuator_id = i + 1
            row, col = i // 5, i % 5
            frame = QtWidgets.QFrame()
            frame.setFrameShape(QtWidgets.QFrame.Shape.Box)
            frame.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
            frame.setStyleSheet("padding: 8px; margin: 3px;")
            fl = QtWidgets.QVBoxLayout(frame)
            id_label = QtWidgets.QLabel(f"Actuator {actuator_id}")
            id_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            id_label.setStyleSheet("font-weight: bold; font-size: 11pt;")
            fl.addWidget(id_label)
            # Add label input field
            label_edit = QtWidgets.QLineEdit()
            label_edit.setPlaceholderText("Label...")
            label_edit.setText(self.actuator_labels.get(actuator_id, ""))
            label_edit.setStyleSheet("font-size: 9pt; padding: 2px;")
            label_edit.textChanged.connect(lambda text, aid=actuator_id: self.on_label_changed(aid, text))
            fl.addWidget(label_edit)
            btn_layout = QtWidgets.QHBoxLayout()
            on_btn = QtWidgets.QPushButton("ON")
            on_btn.setMinimumHeight(36)
            off_btn = QtWidgets.QPushButton("OFF")
            off_btn.setMinimumHeight(36)
            bg = self.palette().color(QtGui.QPalette.ColorRole.Window).name()
            inactive = f"QPushButton {{ font-weight: bold; background-color: {bg}; color: #FFF; border: none; border-radius: 4px; }}"
            on_btn.setStyleSheet(inactive)
            off_btn.setStyleSheet(inactive)
            on_btn.clicked.connect(lambda checked=False, aid=actuator_id: self.set_actuator_state(aid, 1))
            off_btn.clicked.connect(lambda checked=False, aid=actuator_id: self.set_actuator_state(aid, 0))
            btn_layout.addWidget(on_btn)
            btn_layout.addWidget(off_btn)
            fl.addLayout(btn_layout)
            voltage_label = QtWidgets.QLabel("0.000 V")
            voltage_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            voltage_label.setStyleSheet("font-size: 9pt;")
            fl.addWidget(voltage_label)
            grid_layout.addWidget(frame, row, col)
            self.actuator_widgets.append({"frame": frame, "on_btn": on_btn, "off_btn": off_btn, "voltage_label": voltage_label, "label_edit": label_edit})
        layout.addWidget(grid_container, 1)
        for i in range(NUM_ACTUATORS):
            self.update_button_highlight(i, 0)

    def on_label_changed(self, actuator_id: int, text: str):
        """Handle label text change"""
        self.actuator_labels[actuator_id] = text
        self.save_labels()

    def start_receiver(self):
        self.receiver = ActuatorUDPReceiver(port=self.receive_port, bind_address=self.bind_address)
        self.receiver.sensor_data_received.connect(self.on_sensor_data)
        self.receiver.status_update.connect(self.status_label.setText)
        self.receiver.start()

    def on_sensor_data(self, header: dict, chunks: List[dict]):
        if chunks:
            for dp in chunks[-1]["datapoints"]:
                sid = dp["sensor_id"]
                if 0 <= sid < NUM_ACTUATORS:
                    self.voltage_readings[sid] = float(dp["data"])

    def update_button_highlight(self, array_idx: int, actuator_state: int):
        w = self.actuator_widgets[array_idx]
        bg = w["frame"].palette().color(QtGui.QPalette.ColorRole.Window).name()
        inactive = f"QPushButton {{ font-weight: bold; background-color: {bg}; color: #FFF; border: none; border-radius: 4px; }}"
        active = "QPushButton { font-weight: bold; background-color: #FFF; color: #000; border: 2px solid #000; border-radius: 4px; }"
        if actuator_state == 1:
            w["on_btn"].setStyleSheet(active)
            w["off_btn"].setStyleSheet(inactive)
        else:
            w["on_btn"].setStyleSheet(inactive)
            w["off_btn"].setStyleSheet(active)

    def set_actuator_state(self, actuator_id: int, actuator_state: int):
        array_idx = actuator_id - 1
        self.actuator_states[array_idx] = actuator_state
        self.update_button_highlight(array_idx, actuator_state)
        try:
            packet = create_actuator_command_packet([(actuator_id, actuator_state)])
            if packet:
                self.command_sock.sendto(packet, (self.device_ip, self.device_port))
        except Exception as e:
            self.status_label.setText(f"Send error: {e}")

    def update_voltage_display(self):
        for i in range(NUM_ACTUATORS):
            self.actuator_widgets[i]["voltage_label"].setText(f"{self.voltage_readings[i]:.3f} V")

    def apply_settings(self, device_ip: str, device_port: int, receive_port: int, bind_address: str):
        self.device_ip = device_ip
        self.device_port = device_port
        self.receive_port = receive_port
        self.bind_address = bind_address
        if self.receiver:
            self.receiver.stop()
            self.receiver.wait(2000)
        self.start_receiver()

    def stop(self):
        self.save_labels()
        if self.receiver:
            self.receiver.stop()
            self.receiver.wait(2000)
        try:
            self.command_sock.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Right panel: PT plots
# ---------------------------------------------------------------------------
class PTPanel(QtWidgets.QWidget):
    def __init__(self, port: int, bind_address: str, window_seconds: float = DEFAULT_PT_WINDOW_SECONDS, parent=None):
        super().__init__(parent)
        self.port = port
        self.bind_address = bind_address
        self.window_seconds = window_seconds
        self.sensor_data: Dict[int, deque] = {}
        self.sensor_plots: Dict[int, pg.PlotDataItem] = {}
        self.stats_start_time = time.time()
        self.receiver = None
        self.init_ui()
        self.start_receiver()

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        self.status_label = QtWidgets.QLabel("Starting...")
        self.status_label.setStyleSheet("font-weight: bold; padding: 5px;")
        layout.addWidget(self.status_label)
        plot_stats = QtWidgets.QHBoxLayout()
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground("k")
        plot_stats.addWidget(self.plot_widget, 1)
        stats_group = QtWidgets.QGroupBox("Statistics")
        sl = QtWidgets.QVBoxLayout()
        self.packets_label = QtWidgets.QLabel("Packets: 0")
        self.pps_label = QtWidgets.QLabel("Packets/sec: 0.0")
        for w in (self.packets_label, self.pps_label):
            w.setFont(QtGui.QFont("", 10))
        sl.addWidget(self.packets_label)
        sl.addWidget(self.pps_label)
        sl.addStretch()
        stats_group.setLayout(sl)
        stats_group.setFixedWidth(140)
        plot_stats.addWidget(stats_group)
        layout.addLayout(plot_stats, 1)
        self.plot_item = self.plot_widget.addPlot(title="PT Voltages")
        self.plot_item.setTitle("PT Voltages", color="w", size="12pt")
        self.plot_item.setLabel("left", "Voltage (V)", color="w")
        self.plot_item.setLabel("bottom", "Time (s)", color="w")
        self.plot_item.addLegend()
        self.plot_item.showGrid(x=True, y=True, alpha=0.5)
        self.plot_item.getViewBox().setBackgroundColor("k")
        for i in range(NUM_PTS):
            c = PT_COLORS[i % len(PT_COLORS)]
            pen = pg.mkPen(color=c, width=2)
            plot = self.plot_item.plot([], [], pen=pen, name=f"PT {i + 1}")
            self.sensor_plots[i] = plot
            self.sensor_data[i] = deque(maxlen=MAX_POINTS)

    def start_receiver(self):
        self.receiver = PTUDPReceiver(port=self.port, bind_address=self.bind_address)
        self.receiver.sensor_data_received.connect(self.on_sensor_data)
        self.receiver.status_update.connect(self.status_label.setText)
        self.receiver.start()

    def on_sensor_data(self, header: dict, chunks: List[dict]):
        t0 = time.time() - self.stats_start_time
        for ch in chunks:
            for dp in ch["datapoints"]:
                sid = int(dp["sensor_id"])
                if 0 <= sid < NUM_PTS:
                    self.sensor_data[sid].append((t0, float(dp["data"])))

    def update_plots(self):
        t = time.time() - self.stats_start_time
        win = self.window_seconds
        for i in range(NUM_PTS):
            d = self.sensor_data[i]
            if not d:
                continue
            times, vals = [], []
            for ti, v in d:
                if t - ti <= win:
                    times.append(ti)
                    vals.append(v)
            if times:
                self.sensor_plots[i].setData(np.array(times), np.array(vals))
        if t > win:
            self.plot_item.setXRange(t - win, t, padding=0)
        else:
            self.plot_item.setXRange(0, win, padding=0)

    def update_statistics(self):
        if self.receiver is None:
            return
        s = self.receiver.get_stats()
        self.packets_label.setText(f"Packets: {s['packets']}")
        self.pps_label.setText(f"Packets/sec: {s['packets_per_sec']:.2f}")

    def apply_settings(self, port: int, bind_address: str, window_seconds: float):
        self.port = port
        self.bind_address = bind_address
        self.window_seconds = window_seconds
        if self.receiver:
            self.receiver.stop()
            self.receiver.wait(2000)
        self.start_receiver()

    def stop(self):
        if self.receiver:
            self.receiver.stop()
            self.receiver.wait(2000)


# ---------------------------------------------------------------------------
# Main window: two panels side by side
# ---------------------------------------------------------------------------
class CombinedActuatorPTWindow(QtWidgets.QMainWindow):
    def __init__(
        self,
        actuator_ip: str = DEFAULT_ACTUATOR_IP,
        actuator_port: int = DEFAULT_ACTUATOR_PORT,
        actuator_receive_port: int = DEFAULT_ACTUATOR_RECEIVE_PORT,
        pt_receive_port: int = DEFAULT_PT_RECEIVE_PORT,
        bind_address: str = "0.0.0.0",
        pt_window_seconds: float = DEFAULT_PT_WINDOW_SECONDS,
    ):
        super().__init__()
        self.actuator_ip = actuator_ip
        self.actuator_port = actuator_port
        self.actuator_receive_port = actuator_receive_port
        self.pt_receive_port = pt_receive_port
        self.bind_address = bind_address
        self.pt_window_seconds = pt_window_seconds

        self.setWindowTitle("Actuator Control + PT Plots")
        self.setGeometry(80, 80, 1400, 600)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)

        top_bar = QtWidgets.QHBoxLayout()
        settings_btn = QtWidgets.QPushButton("Settings")
        settings_btn.clicked.connect(self.show_settings)
        top_bar.addStretch()
        top_bar.addWidget(settings_btn)
        main_layout.addLayout(top_bar)

        splitter = QtWidgets.QSplitter(_QT_HORIZ)
        self.actuator_panel = ActuatorPanel(
            device_ip=actuator_ip,
            device_port=actuator_port,
            receive_port=actuator_receive_port,
            bind_address=bind_address,
        )
        self.pt_panel = PTPanel(port=pt_receive_port, bind_address=bind_address, window_seconds=pt_window_seconds)
        splitter.addWidget(self.actuator_panel)
        splitter.addWidget(self.pt_panel)
        splitter.setSizes([500, 700])  # left width, right width
        main_layout.addWidget(splitter, 1)

        self.actuator_timer = QtCore.QTimer()
        self.actuator_timer.timeout.connect(self.actuator_panel.update_voltage_display)
        self.actuator_timer.start(ACTUATOR_UPDATE_MS)
        self.pt_timer = QtCore.QTimer()
        self.pt_timer.timeout.connect(self.pt_panel.update_plots)
        self.pt_timer.start(UPDATE_INTERVAL_MS)
        self.stats_timer = QtCore.QTimer()
        self.stats_timer.timeout.connect(self.pt_panel.update_statistics)
        self.stats_timer.start(500)

    def show_settings(self):
        d = QtWidgets.QDialog(self)
        d.setWindowTitle("Settings")
        lo = QtWidgets.QVBoxLayout(d)
        # Actuator
        ag = QtWidgets.QGroupBox("Actuator board")
        al = QtWidgets.QFormLayout()
        self._act_ip_edit = QtWidgets.QLineEdit(self.actuator_ip)
        self._act_port_edit = QtWidgets.QSpinBox()
        self._act_port_edit.setRange(1, 65535)
        self._act_port_edit.setValue(self.actuator_port)
        self._act_recv_edit = QtWidgets.QSpinBox()
        self._act_recv_edit.setRange(1, 65535)
        self._act_recv_edit.setValue(self.actuator_receive_port)
        al.addRow("Device IP:", self._act_ip_edit)
        al.addRow("Command port:", self._act_port_edit)
        al.addRow("Receive port (sensor data):", self._act_recv_edit)
        ag.setLayout(al)
        lo.addWidget(ag)
        # PT
        pt_group = QtWidgets.QGroupBox("PT board")
        pl = QtWidgets.QFormLayout()
        self._pt_port_edit = QtWidgets.QSpinBox()
        self._pt_port_edit.setRange(1, 65535)
        self._pt_port_edit.setValue(self.pt_receive_port)
        self._pt_win_edit = QtWidgets.QDoubleSpinBox()
        self._pt_win_edit.setRange(5, 120)
        self._pt_win_edit.setValue(self.pt_window_seconds)
        self._pt_win_edit.setSuffix(" s")
        pl.addRow("Receive port:", self._pt_port_edit)
        pl.addRow("Time window:", self._pt_win_edit)
        pt_group.setLayout(pl)
        lo.addWidget(pt_group)
        # Bind
        self._bind_edit = QtWidgets.QLineEdit(self.bind_address)
        lo.addWidget(QtWidgets.QLabel("Bind address (0.0.0.0 for all):"))
        lo.addWidget(self._bind_edit)
        # Buttons
        btn_layout = QtWidgets.QHBoxLayout()
        ok_btn = QtWidgets.QPushButton("OK")
        def on_ok():
            self.actuator_ip = self._act_ip_edit.text()
            self.actuator_port = self._act_port_edit.value()
            self.actuator_receive_port = self._act_recv_edit.value()
            self.pt_receive_port = self._pt_port_edit.value()
            self.bind_address = self._bind_edit.text() or "0.0.0.0"
            self.pt_window_seconds = self._pt_win_edit.value()
            self.actuator_panel.apply_settings(self.actuator_ip, self.actuator_port, self.actuator_receive_port, self.bind_address)
            self.pt_panel.apply_settings(self.pt_receive_port, self.bind_address, self.pt_window_seconds)
            d.accept()
        ok_btn.clicked.connect(on_ok)
        cancel_btn = QtWidgets.QPushButton("Cancel")
        cancel_btn.clicked.connect(d.reject)
        btn_layout.addWidget(ok_btn)
        btn_layout.addWidget(cancel_btn)
        lo.addLayout(btn_layout)
        d.exec()

    def closeEvent(self, event):
        self.actuator_panel.stop()
        self.pt_panel.stop()
        event.accept()


def main():
    import argparse
    p = argparse.ArgumentParser(description="Combined Actuator Control + PT Plots GUI")
    p.add_argument("-i", "--actuator-ip", default=DEFAULT_ACTUATOR_IP, help="Actuator board IP")
    p.add_argument("--actuator-port", type=int, default=DEFAULT_ACTUATOR_PORT, help="Actuator command port")
    p.add_argument("--actuator-receive-port", type=int, default=DEFAULT_ACTUATOR_RECEIVE_PORT, help="Port for actuator sensor data")
    p.add_argument("--pt-port", type=int, default=DEFAULT_PT_RECEIVE_PORT, help="Port for PT sensor data")
    p.add_argument("-a", "--address", default="0.0.0.0", help="Bind address")
    args = p.parse_args()
    app = QtWidgets.QApplication(sys.argv)
    w = CombinedActuatorPTWindow(
        actuator_ip=args.actuator_ip,
        actuator_port=args.actuator_port,
        actuator_receive_port=args.actuator_receive_port,
        pt_receive_port=args.pt_port,
        bind_address=args.address,
    )
    w.show()
    sys.exit(app.exec_() if hasattr(app, "exec_") else app.exec())


if __name__ == "__main__":
    main()
