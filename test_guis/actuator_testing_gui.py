import sys
import socket
import struct
import time
from collections import deque
from typing import Dict, List, Tuple, Optional

from PyQt6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg


# ---------------------- Protocol constants (DAQv2 style) ----------------------

DIABLO_COMMS_VERSION = 0
MAX_PACKET_SIZE = 512


class PacketType:
    BOARD_HEARTBEAT = 1
    SERVER_HEARTBEAT = 2
    SENSOR_DATA = 3
    ACTUATOR_COMMAND = 4


PACKET_HEADER_FORMAT = '<BBI'
PACKET_HEADER_SIZE = 6

SENSOR_DATA_PACKET_FORMAT = '<BB'
SENSOR_DATA_PACKET_SIZE = 2
SENSOR_DATA_CHUNK_FORMAT = '<I'
SENSOR_DATA_CHUNK_SIZE = 4
SENSOR_DATAPOINT_FORMAT = '<BI'
SENSOR_DATAPOINT_SIZE = 5

ACTUATOR_COMMAND_PACKET_FORMAT = '<B'
ACTUATOR_COMMAND_PACKET_SIZE = 1
ACTUATOR_COMMAND_FORMAT = '<BB'
ACTUATOR_COMMAND_SIZE = 2

NUM_ACTUATORS = 10

DEFAULT_ACTUATOR_IP = "192.168.2.202"  # Actuator_Testing board
ACTUATOR_COMMAND_PORT = 5005          # Board listens here for commands
SENSOR_RECEIVE_PORT = 5006            # Board sends current-sense packets here

WINDOW_SECONDS = 10.0
UPDATE_INTERVAL_MS = 100
MAX_POINTS = 50000

SENSOR_COLORS = [
    (255, 80, 80),
    (80, 255, 80),
    (80, 150, 255),
    (255, 200, 80),
    (200, 80, 255),
    (80, 255, 255),
    (255, 150, 150),
    (150, 255, 150),
    (150, 200, 255),
    (255, 255, 80),
]


# ---------------------- Packet helpers ----------------------

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
        num_chunks, num_sensors = struct.unpack(
            SENSOR_DATA_PACKET_FORMAT,
            data[offset:offset + SENSOR_DATA_PACKET_SIZE],
        )
    except struct.error:
        return None

    offset += SENSOR_DATA_PACKET_SIZE
    per_chunk_size = SENSOR_DATA_CHUNK_SIZE + (num_sensors * SENSOR_DATAPOINT_SIZE)
    expected_size = PACKET_HEADER_SIZE + SENSOR_DATA_PACKET_SIZE + (num_chunks * per_chunk_size)
    if len(data) < expected_size:
        return None

    chunks: List[dict] = []
    for _ in range(num_chunks):
        try:
            (chunk_ts,) = struct.unpack(
                SENSOR_DATA_CHUNK_FORMAT,
                data[offset:offset + SENSOR_DATA_CHUNK_SIZE],
            )
        except struct.error:
            return None
        offset += SENSOR_DATA_CHUNK_SIZE

        dps: List[dict] = []
        for _ in range(num_sensors):
            try:
                sensor_id, sensor_data = struct.unpack(
                    SENSOR_DATAPOINT_FORMAT,
                    data[offset:offset + SENSOR_DATAPOINT_SIZE],
                )
            except struct.error:
                return None
            offset += SENSOR_DATAPOINT_SIZE
            dps.append({"sensor_id": sensor_id, "data": sensor_data})

        chunks.append({"timestamp": chunk_ts, "datapoints": dps})

    header_dict = {
        "packet_type": packet_type,
        "version": version,
        "timestamp": timestamp,
    }
    return header_dict, chunks


def create_actuator_command_packet(commands: List[Tuple[int, int]]) -> bytes:
    """
    commands: list of (actuator_id, state) where id is 1-10, state 0/1.
    """
    if not commands or len(commands) > 255:
        return b""

    header_size = PACKET_HEADER_SIZE
    body_size = ACTUATOR_COMMAND_PACKET_SIZE
    commands_size = len(commands) * ACTUATOR_COMMAND_SIZE
    total_size = header_size + body_size + commands_size
    if total_size > MAX_PACKET_SIZE:
        return b""

    pkt = bytearray(total_size)
    offset = 0

    # header
    packet_type = PacketType.ACTUATOR_COMMAND
    version = DIABLO_COMMS_VERSION
    timestamp = int(time.time() * 1000) & 0xFFFFFFFF
    struct.pack_into(PACKET_HEADER_FORMAT, pkt, offset, packet_type, version, timestamp)
    offset += PACKET_HEADER_SIZE

    # body header
    struct.pack_into(ACTUATOR_COMMAND_PACKET_FORMAT, pkt, offset, len(commands))
    offset += ACTUATOR_COMMAND_PACKET_SIZE

    # commands
    for actuator_id, state in commands:
        struct.pack_into(ACTUATOR_COMMAND_FORMAT, pkt, offset, actuator_id, state)
        offset += ACTUATOR_COMMAND_SIZE

    return bytes(pkt)


# ---------------------- Receiver thread ----------------------

class SensorReceiver(QtCore.QThread):
    sensor_data_received = QtCore.pyqtSignal(dict, list, str)
    status_update = QtCore.pyqtSignal(str)

    def __init__(self, port: int, bind_address: str = "0.0.0.0"):
        super().__init__()
        self.port = port
        self.bind_address = bind_address
        self._stop = False
        self.sock: Optional[socket.socket] = None

    def stop(self):
        self._stop = True

    def run(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(0.1)
        try:
            self.sock.bind((self.bind_address, self.port))
            self.status_update.emit(f"Listening on {self.bind_address}:{self.port}")
        except OSError as e:
            self.status_update.emit(f"Error binding: {e}")
            return

        while not self._stop:
            try:
                data, addr = self.sock.recvfrom(MAX_PACKET_SIZE)
            except socket.timeout:
                continue
            except Exception as e:
                if not self._stop:
                    self.status_update.emit(f"Error: {e}")
                continue

            src_ip = addr[0]
            header = parse_packet_header(data)
            if not header or header[0] != PacketType.SENSOR_DATA:
                continue

            result = parse_sensor_data_packet(data)
            if not result:
                continue
            header_dict, chunks = result
            self.sensor_data_received.emit(header_dict, chunks, src_ip)

        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        self.status_update.emit("Stopped")


# ---------------------- Main GUI ----------------------

class ActuatorTestingWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Actuator Testing – Current Sense + Control")
        self.resize(1100, 700)

        self.device_ip = DEFAULT_ACTUATOR_IP
        self.device_port = ACTUATOR_COMMAND_PORT

        self.window_seconds = WINDOW_SECONDS
        self.show_current = False

        self.voltage_history: Dict[int, deque] = {
            i: deque(maxlen=MAX_POINTS) for i in range(1, NUM_ACTUATORS + 1)
        }
        self.plot_enabled: Dict[int, bool] = {i: True for i in range(1, NUM_ACTUATORS + 1)}

        self._build_ui()

        self.command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.receiver = SensorReceiver(SENSOR_RECEIVE_PORT)
        self.receiver.sensor_data_received.connect(self.on_sensor_data)
        self.receiver.status_update.connect(self.status_label.setText)
        self.receiver.start()

        self.update_timer = QtCore.QTimer(self)
        self.update_timer.timeout.connect(self.update_plots)
        self.update_timer.start(UPDATE_INTERVAL_MS)

    # ----- UI -----

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        # Left: actuator controls
        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)

        net_row = QtWidgets.QHBoxLayout()
        net_row.addWidget(QtWidgets.QLabel("Actuator IP:"))
        self.ip_edit = QtWidgets.QLineEdit(self.device_ip)
        net_row.addWidget(self.ip_edit)
        net_row.addWidget(QtWidgets.QLabel("Cmd port:"))
        self.port_spin = QtWidgets.QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(self.device_port)
        net_row.addWidget(self.port_spin)
        left_layout.addLayout(net_row)

        # Graph channel toggles (similar to sense_testing_gui)
        chan_row = QtWidgets.QHBoxLayout()
        chan_row.addWidget(QtWidgets.QLabel("Graph actuators:"))
        self.channel_checkboxes: Dict[int, QtWidgets.QCheckBox] = {}
        for aid in range(1, NUM_ACTUATORS + 1):
            cb = QtWidgets.QCheckBox(f"A{aid}")
            cb.setChecked(True)
            r, g, b = SENSOR_COLORS[(aid - 1) % len(SENSOR_COLORS)]
            cb.setStyleSheet(
                f"QCheckBox {{ color: rgb({r},{g},{b}); font-weight: bold; }}"
            )
            cb.stateChanged.connect(
                lambda state, a=aid: self.on_channel_toggled(a, state)
            )
            self.channel_checkboxes[aid] = cb
            chan_row.addWidget(cb)
        chan_row.addStretch()
        left_layout.addLayout(chan_row)

        # Voltage / current toggle
        mode_row = QtWidgets.QHBoxLayout()
        self.show_current_cb = QtWidgets.QCheckBox("Show current (A)")
        self.show_current_cb.setToolTip("When checked, convert sense voltage to current using V/20/0.05.")
        self.show_current_cb.stateChanged.connect(self.on_show_current_toggled)
        mode_row.addWidget(self.show_current_cb)
        mode_row.addStretch()
        left_layout.addLayout(mode_row)

        grid = QtWidgets.QGridLayout()
        self.button_map: Dict[int, QtWidgets.QPushButton] = {}
        self.voltage_labels: Dict[int, QtWidgets.QLabel] = {}

        for i in range(NUM_ACTUATORS):
            actuator_id = i + 1
            row = i // 2
            col = (i % 2) * 2

            btn = QtWidgets.QPushButton(f"Actuator {actuator_id}")
            btn.setCheckable(True)
            btn.clicked.connect(lambda checked, aid=actuator_id: self.on_actuator_toggled(aid, checked))

            vlabel = QtWidgets.QLabel("0.000 V")
            vlabel.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            vlabel.setStyleSheet(
                "font-size: 9pt; padding: 0 2px; color: white; background-color: transparent;"
            )

            self.button_map[actuator_id] = btn
            self.voltage_labels[actuator_id] = vlabel

            grid.addWidget(btn, row, col)
            grid.addWidget(vlabel, row, col + 1)

        left_layout.addLayout(grid)
        left_layout.addStretch()

        self.status_label = QtWidgets.QLabel("Ready")
        self.status_label.setStyleSheet("color: #8ab; font-size: 11px;")
        left_layout.addWidget(self.status_label)

        # Right: plot
        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)

        self.plot_widget = pg.PlotWidget(background="#1e1e1e")
        axis_style = {"color": "#FFFFFF", "font-size": "11pt"}
        self.plot_widget.setLabel("left", "Current sense (V)", **axis_style)
        self.plot_widget.setLabel("bottom", "Time (s)", **axis_style)
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.enableAutoRange(axis="y")
        self.plot_widget.addLegend(offset=(10, 10))

        for axis_name in ("left", "bottom"):
            ax = self.plot_widget.getPlotItem().getAxis(axis_name)
            try:
                if hasattr(ax, "setStyle"):
                    ax.setStyle(tickTextColor="#FFFFFF", tickFont=QtGui.QFont(None, 10))
            except Exception:
                pass

        self.curves: Dict[int, pg.PlotDataItem] = {}
        for actuator_id in range(1, NUM_ACTUATORS + 1):
            r, g, b = SENSOR_COLORS[(actuator_id - 1) % len(SENSOR_COLORS)]
            pen = pg.mkPen(color=(r, g, b), width=2)
            curve = self.plot_widget.plot([], [], pen=pen, name=f"A{actuator_id}")
            self.curves[actuator_id] = curve

        right_layout.addWidget(self.plot_widget)

        layout.addWidget(left, stretch=0)
        layout.addWidget(right, stretch=1)

    # ----- Networking helpers -----

    def on_actuator_toggled(self, actuator_id: int, checked: bool):
        state = 1 if checked else 0
        pkt = create_actuator_command_packet([(actuator_id, state)])
        if not pkt:
            return
        ip = self.ip_edit.text().strip() or self.device_ip
        port = int(self.port_spin.value())
        try:
            self.command_sock.sendto(pkt, (ip, port))
            self.status_label.setText(f"Sent command: actuator {actuator_id} -> {state}")
        except OSError as e:
            self.status_label.setText(f"Send error: {e}")

    def on_sensor_data(self, header: dict, chunks: List[dict], source_ip: str):
        if not chunks:
            return

        latest = chunks[-1]
        now = time.time()
        t0 = getattr(self, "_t0", None)
        if t0 is None:
            t0 = now
            self._t0 = t0
        t_rel = now - t0

        for dp in latest["datapoints"]:
            sid = int(dp.get("sensor_id", -1))
            data_u32 = int(dp.get("data", 0))

            # Actuator_Testing now sends 1-indexed IDs; still accept legacy 0-indexed.
            if 1 <= sid <= NUM_ACTUATORS:
                actuator_id = sid
            elif 0 <= sid < NUM_ACTUATORS:
                actuator_id = sid + 1
            else:
                continue

            # Voltage is stored as float bits in data_u32
            try:
                voltage = struct.unpack("<f", struct.pack("<I", data_u32 & 0xFFFFFFFF))[0]
            except struct.error:
                continue

            hist = self.voltage_history[actuator_id]
            hist.append((t_rel, float(voltage)))

            # Update numeric readout according to mode
            if self.show_current:
                # V_adc -> V_shunt -> I = V_shunt / 0.05
                v_shunt = voltage / 20.0
                current = v_shunt / 0.05
                self.voltage_labels[actuator_id].setText(f"{current:.3f} A")
            else:
                self.voltage_labels[actuator_id].setText(f"{voltage:.3f} V")

    def on_channel_toggled(self, actuator_id: int, state: int):
        from PyQt6.QtCore import Qt
        enabled = state == QtCore.Qt.CheckState.Checked.value
        self.plot_enabled[actuator_id] = enabled
        curve = self.curves.get(actuator_id)
        if curve is not None:
            curve.setVisible(enabled)

    def on_show_current_toggled(self, state: int):
        from PyQt6.QtCore import Qt
        self.show_current = state == QtCore.Qt.CheckState.Checked.value
        # Update axis label
        axis_style = {"color": "#FFFFFF", "font-size": "11pt"}
        if self.show_current:
            self.plot_widget.setLabel("left", "Current (A)", **axis_style)
        else:
            self.plot_widget.setLabel("left", "Current sense (V)", **axis_style)
        # Force labels to refresh on next packet; plot refresh will convert accordingly
        self.update_plots()

    def update_plots(self):
        for actuator_id, curve in self.curves.items():
            if not self.plot_enabled.get(actuator_id, True):
                continue
            hist = self.voltage_history[actuator_id]
            if not hist:
                continue
            xs, ys = zip(*hist)
            # Limit time window
            t_max = xs[-1]
            t_min = max(0.0, t_max - self.window_seconds)
            xs_plot = []
            ys_plot = []
            for x, v in hist:
                if x < t_min:
                    continue
                if self.show_current:
                    v_shunt = v / 20.0
                    value = v_shunt / 0.05
                else:
                    value = v
                xs_plot.append(x)
                ys_plot.append(value)
            curve.setData(xs_plot, ys_plot)


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = ActuatorTestingWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

