#!/usr/bin/env python3
"""
Combined Sensor & Actuator Control GUI
Opens one full-screen window with:
- Left (66%): Sensor/PT data receiver and real-time plotting
- Right (33%): Actuator control and voltage monitoring

Requirements: pip install pyqt6 pyqtgraph numpy
"""

import csv
import json
import os
import re
import socket
import struct
import sys
import time
from pathlib import Path
from typing import Optional, Tuple, List, Dict
from collections import deque

# PT calibration CSV (relative to this file); used to show psi for PT connectors present in CSV
PT_CALIBRATION_CSV = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),  # Go up to project root
    "PT_Board", "Calibration", "PT Calibration Attempt 2026-02-04_test2.csv"
)

# Fix Qt "cocoa" platform plugin on macOS when Homebrew Qt plugins path lacks platforms/
if sys.platform == 'darwin':
    _qt_plugins = os.environ.get('QT_QPA_PLATFORM_PLUGIN_PATH')
    if not _qt_plugins or not os.path.isdir(_qt_plugins):
        _candidates = [
            '/opt/homebrew/share/qt/plugins/platforms',
            '/opt/homebrew/Cellar/qtbase/6.10.1/share/qt/plugins/platforms',
        ]
        if os.path.isdir('/opt/homebrew/Cellar/qtbase'):
            for _name in sorted(os.listdir('/opt/homebrew/Cellar/qtbase'), reverse=True):
                _p = os.path.join('/opt/homebrew/Cellar/qtbase', _name, 'share', 'qt', 'plugins', 'platforms')
                if os.path.isdir(_p):
                    _candidates.insert(1, _p)
                    break
        for _p in _candidates:
            if os.path.isdir(_p) and any(_f.startswith('libqcocoa') for _f in os.listdir(_p)):
                os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = _p
                break

from PyQt6 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
import numpy as np

# Protocol constants from DAQv2-Comms.h
DIABLO_COMMS_VERSION = 0
MAX_PACKET_SIZE = 512

# PacketType enum from DiabloEnums.h
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

# Default configuration
DEFAULT_SENSOR_IP = '192.168.2.101'  # Sensor board IP address
DEFAULT_ACTUATOR_IP = '192.168.2.201'  # Actuator board IP address
DEFAULT_DEVICE_PORT = 5005  # Port device listens on for actuator commands
DEFAULT_RECEIVE_PORT = 5006  # Port device sends sensor data to

# Struct format strings (little-endian, matching C++ packed structs)
PACKET_HEADER_FORMAT = '<BBI'  # 6 bytes total
PACKET_HEADER_SIZE = 6

ACTUATOR_COMMAND_PACKET_FORMAT = '<B'  # 1 byte
ACTUATOR_COMMAND_PACKET_SIZE = 1

ACTUATOR_COMMAND_FORMAT = '<BB'  # 2 bytes
ACTUATOR_COMMAND_SIZE = 2

SENSOR_DATA_PACKET_FORMAT = '<BB'  # 2 bytes
SENSOR_DATA_PACKET_SIZE = 2

SENSOR_DATA_CHUNK_FORMAT = '<I'  # 4 bytes
SENSOR_DATA_CHUNK_SIZE = 4

SENSOR_DATAPOINT_FORMAT = '<BI'  # 5 bytes: uint8_t sensor_id + uint32_t data
SENSOR_DATAPOINT_SIZE = 5

# Plotting constants
DEFAULT_WINDOW_SECONDS = 10.0
MAX_POINTS = 10000
UPDATE_INTERVAL_MS = 50  # Update plots every 50ms
NUM_CONNECTORS = 10  # Number of connectors being cycled (1-10)
NUM_ACTUATORS = 10
ACTUATOR_LABELS_FILE = Path(__file__).parent / "actuator_labels.json"
SENSOR_LABELS_FILE = Path(__file__).parent / "sensor_labels.json"

# Colors for sensors (cycle through if more than this)
SENSOR_COLORS = [
    (255, 80, 80),    # Red
    (80, 255, 80),    # Green
    (80, 150, 255),   # Blue
    (255, 200, 80),   # Orange
    (200, 80, 255),   # Purple
    (80, 255, 255),   # Cyan
    (255, 150, 150),  # Light Red
    (150, 255, 150),  # Light Green
    (150, 200, 255),  # Light Blue
    (255, 255, 80),   # Yellow
]



CONFIG_FILE = Path(__file__).parent / "config.json"

class ConfigManager:
    """Manages loading and saving of application configuration."""
    def __init__(self):
        self.config = {
            "actuators": {str(i): "" for i in range(1, NUM_ACTUATORS + 1)},
            "sensors": {str(i): "" for i in range(1, NUM_CONNECTORS + 1)},
            "network": {
                "actuator_ip": DEFAULT_ACTUATOR_IP,
                "actuator_port": DEFAULT_DEVICE_PORT,
                "sensor_ip_filter": DEFAULT_SENSOR_IP,
                "receive_port": DEFAULT_RECEIVE_PORT
            },
            "display": {
                "adc_bits": 32,
                "ref_voltage": 2.5,
                "window_seconds": DEFAULT_WINDOW_SECONDS,
                "y_axis_min": 0.0,
                "y_axis_max": 200.0,
                "y_axis_autoscale": True
            },
            "mappings": {
                "GN2": 0,
                "ETH": 0,
                "LOX": 0
            }
        }
        self.load()

    def load(self):
        if CONFIG_FILE.exists():
            try:
                with open(CONFIG_FILE, 'r') as f:
                    loaded = json.load(f)
                    # Recursive update to preserve defaults for missing keys
                    self._update_dict(self.config, loaded)
            except Exception as e:
                print(f"Error loading config: {e}")
        else:
            # Config doesn't exist, try to migrate legacy labels
            print("Config file not found. checking for legacy label files...")
            self._migrate_legacy_labels()
            # Save the new config immediately
            self.save()

    def _migrate_legacy_labels(self):
        # Migrate Actuator Labels
        if ACTUATOR_LABELS_FILE.exists():
            try:
                with open(ACTUATOR_LABELS_FILE, 'r') as f:
                    labels = json.load(f)
                    for k, v in labels.items():
                        if k in self.config["actuators"]:
                            self.config["actuators"][k] = v
                print(f"Migrated actuator labels from {ACTUATOR_LABELS_FILE}")
            except Exception as e:
                print(f"Error migrating actuator labels: {e}")

        # Migrate Sensor Labels
        if SENSOR_LABELS_FILE.exists():
            try:
                with open(SENSOR_LABELS_FILE, 'r') as f:
                    labels = json.load(f)
                    for k, v in labels.items():
                        # Support both "1" and "PT1" style keys just in case, though file implies IDs
                        # The legacy code keys were strings of ints "1", "2"
                        if k in self.config["sensors"]:
                            self.config["sensors"][k] = v
                print(f"Migrated sensor labels from {SENSOR_LABELS_FILE}")
            except Exception as e:
                print(f"Error migrating sensor labels: {e}")

    def save(self):
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(self.config, f, indent=4)
            print(f"Saved config to {CONFIG_FILE}")
        except Exception as e:
            print(f"Error saving config: {e}")

    def _update_dict(self, target, source):
        for k, v in source.items():
            if isinstance(v, dict) and k in target and isinstance(target[k], dict):
                self._update_dict(target[k], v)
            else:
                target[k] = v

    # Helpers
    def get_actuator_label(self, idx): return self.config["actuators"].get(str(idx), "")
    def set_actuator_label(self, idx, txt): self.config["actuators"][str(idx)] = txt; self.save()
    
    def get_sensor_label(self, idx): return self.config["sensors"].get(str(idx), "")
    def set_sensor_label(self, idx, txt): self.config["sensors"][str(idx)] = txt; self.save()

CONFIG = ConfigManager()

pg.setConfigOptions(antialias=False)


# ---------------------- PT pressure from calibration ----------------------
def calculate_pressure(adc_code: float, PT_A: float, PT_B: float, PT_C: float, PT_D: float) -> float:
    """Compute pressure (psi) from ADC code using cubic polynomial."""
    return (PT_A * (adc_code ** 3)) + (PT_B * (adc_code ** 2)) + (PT_C * adc_code) + PT_D


def load_pt_calibration(csv_path: str) -> Dict[int, Tuple[float, float, float, float]]:
    """
    Load PT calibration coefficients from CSV.
    Returns dict: connector_id -> (PT_A, PT_B, PT_C, PT_D) for each PT present in the CSV.
    CSV columns per PT: ADC Code, Pressure, Coefficient 0 (A), 1 (B), 2 (C), 3 (D).
    Uses the last data row as the calibration coefficients.
    Works with any number of PTs; PT numbers are discovered from column names "PT{N} Coefficient 0".
    """
    result = {}
    if not os.path.isfile(csv_path):
        return result
    try:
        with open(csv_path, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            rows = list(reader)
            fieldnames = reader.fieldnames or []
        if not rows:
            return result
        # Discover PT numbers from column names (e.g. "PT1 Coefficient 0" -> pt_num 1)
        pt_nums = set()
        for col in fieldnames:
            m = re.match(r"PT(\d+)\s+Coefficient\s+0", col, re.IGNORECASE)
            if m:
                pt_nums.add(int(m.group(1)))
        # Use last row for coefficients (final calibration state)
        last = rows[-1]
        for pt_num in sorted(pt_nums):
            a = float(last.get(f"PT{pt_num} Coefficient 0", 0))
            b = float(last.get(f"PT{pt_num} Coefficient 1", 0))
            c = float(last.get(f"PT{pt_num} Coefficient 2", 0))
            d = float(last.get(f"PT{pt_num} Coefficient 3", 0))
            result[pt_num] = (a, b, c, d)
        return result
    except Exception:
        return result


# Demo mode: fake UDP packets from separate module
from demo_sensor_sender import build_demo_packet


# ---------------------- Protocol Functions ----------------------
def parse_packet_header(data: bytes) -> Optional[Tuple[int, int, int]]:
    """Parse the packet header. Returns: (packet_type, version, timestamp) or None"""
    if len(data) < PACKET_HEADER_SIZE:
        return None
    try:
        packet_type, version, timestamp = struct.unpack(PACKET_HEADER_FORMAT, data[:PACKET_HEADER_SIZE])
        return (packet_type, version, timestamp)
    except struct.error:
        return None


def parse_sensor_data_packet(data: bytes) -> Optional[Tuple[dict, List[dict]]]:
    """Parse a sensor data packet. Returns: (header_dict, chunks_list) or None"""
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
            data[offset:offset + SENSOR_DATA_PACKET_SIZE]
        )
    except struct.error:
        return None
    
    offset += SENSOR_DATA_PACKET_SIZE
    
    per_chunk_size = SENSOR_DATA_CHUNK_SIZE + (num_sensors * SENSOR_DATAPOINT_SIZE)
    expected_size = PACKET_HEADER_SIZE + SENSOR_DATA_PACKET_SIZE + (num_chunks * per_chunk_size)
    
    if len(data) < expected_size:
        return None
    
    chunks = []
    for chunk_idx in range(num_chunks):
        try:
            chunk_timestamp, = struct.unpack(
                SENSOR_DATA_CHUNK_FORMAT,
                data[offset:offset + SENSOR_DATA_CHUNK_SIZE]
            )
        except struct.error:
            return None
        
        offset += SENSOR_DATA_CHUNK_SIZE
        
        datapoints = []
        for sensor_idx in range(num_sensors):
            try:
                sensor_id, sensor_data = struct.unpack(
                    SENSOR_DATAPOINT_FORMAT,
                    data[offset:offset + SENSOR_DATAPOINT_SIZE]
                )
                datapoints.append({
                    'sensor_id': sensor_id,
                    'data': sensor_data
                })
                offset += SENSOR_DATAPOINT_SIZE
            except struct.error:
                return None
        
        chunks.append({
            'timestamp': chunk_timestamp,
            'datapoints': datapoints
        })
    
    header_dict = {
        'packet_type': packet_type,
        'version': version,
        'timestamp': timestamp
    }
    
    return (header_dict, chunks)


def create_actuator_command_packet(commands: List[Tuple[int, int]]) -> bytes:
    """
    Create an actuator command packet.
    commands: List of (actuator_id, actuator_state) tuples
    actuator_id: 1-10 (1-indexed)
    actuator_state: 0 = OFF, non-zero = ON
    """
    if len(commands) == 0 or len(commands) > 255:
        return b''
    
    # Calculate packet size
    header_size = PACKET_HEADER_SIZE
    body_size = ACTUATOR_COMMAND_PACKET_SIZE
    commands_size = len(commands) * ACTUATOR_COMMAND_SIZE
    total_size = header_size + body_size + commands_size
    
    if total_size > MAX_PACKET_SIZE:
        return b''
    
    # Create packet buffer
    packet = bytearray(total_size)
    offset = 0
    
    # Packet header
    packet_type = PacketType.ACTUATOR_COMMAND
    version = DIABLO_COMMS_VERSION
    timestamp = int(time.time() * 1000) & 0xFFFFFFFF  # 32-bit timestamp in milliseconds
    
    struct.pack_into(PACKET_HEADER_FORMAT, packet, offset, packet_type, version, timestamp)
    offset += PACKET_HEADER_SIZE
    
    # Actuator command packet body
    num_commands = len(commands)
    struct.pack_into(ACTUATOR_COMMAND_PACKET_FORMAT, packet, offset, num_commands)
    offset += ACTUATOR_COMMAND_PACKET_SIZE
    
    # Actuator commands
    for actuator_id, actuator_state in commands:
        struct.pack_into(ACTUATOR_COMMAND_FORMAT, packet, offset, actuator_id, actuator_state)
        offset += ACTUATOR_COMMAND_SIZE
    
    return bytes(packet)


# ---------------------- UDP Receiver Thread ----------------------
class UDPReceiver(QtCore.QThread):
    """Thread that receives UDP packets and emits decoded sensor data"""
    sensor_data_received = QtCore.pyqtSignal(dict, list, str)  # header, chunks, source_ip
    status_update = QtCore.pyqtSignal(str)
    packet_received = QtCore.pyqtSignal(int, int)  # packet_size, packet_type
    
    def __init__(self, port: int = DEFAULT_RECEIVE_PORT, bind_address: str = '0.0.0.0'):
        super().__init__()
        self.port = port
        self.bind_address = bind_address
        self._stop = False
        self.sock = None
        self.total_packets = 0
        self.total_bytes = 0
        self.start_time = None
        
    def stop(self):
        """Stop the receiver thread"""
        self._stop = True
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
    
    def get_stats(self) -> Dict:
        """Get current statistics"""
        if self.start_time is None:
            return {'packets': 0, 'bytes': 0, 'packets_per_sec': 0.0, 'bytes_per_sec': 0.0}
        
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            pps = self.total_packets / elapsed
            bps = self.total_bytes / elapsed
        else:
            pps = 0.0
            bps = 0.0
        
        return {
            'packets': self.total_packets,
            'bytes': self.total_bytes,
            'packets_per_sec': pps,
            'bytes_per_sec': bps,
            'elapsed': elapsed
        }
    
    def run(self):
        """Main receiver loop"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(0.1)  # Non-blocking with timeout
        
        try:
            self.sock.bind((self.bind_address, self.port))
            self.status_update.emit(f"Listening on {self.bind_address}:{self.port}")
            self.start_time = time.time()
        except OSError as e:
            self.status_update.emit(f"Error binding: {e}")
            return
        
        while not self._stop:
            try:
                data, addr = self.sock.recvfrom(MAX_PACKET_SIZE)
                self.total_packets += 1
                self.total_bytes += len(data)
                
                header = parse_packet_header(data)
                if header is None:
                    continue
                
                packet_type, version, timestamp = header
                self.packet_received.emit(len(data), packet_type)
                
                if packet_type == PacketType.SENSOR_DATA:
                    result = parse_sensor_data_packet(data)
                    if result:
                        header_dict, chunks = result
                        source_ip = addr[0]  # Extract IP address from (ip, port) tuple
                        self.sensor_data_received.emit(header_dict, chunks, source_ip)
                        
            except socket.timeout:
                continue
            except Exception as e:
                if not self._stop:
                    self.status_update.emit(f"Error: {e}")
                continue
        
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
        self.status_update.emit("Stopped")


# ---------------------- Sensor Plot Widget (reusable panel) ----------------------
class SensorPlotWidget(QtWidgets.QWidget):
    """Reusable sensor/PT plotting panel. Used in SensorPlotWindow and CombinedMainWindow."""
    def __init__(self, receiver, bind_address: str = '0.0.0.0', parent=None):
        super().__init__(parent)
        self.receiver = receiver
        self.bind_address = bind_address
        self.window_seconds = DEFAULT_WINDOW_SECONDS
        self.display_moving_avg_samples = 10  # Moving average for displayed values
        self.graph_moving_avg_samples = 1  # Moving average for graphed lines (1 = no smoothing)
        
        # ADC conversion settings
        self.adc_bits = 32  # ADC bit count (default: 32-bit)
        self.reference_voltage = 2.5  # Reference voltage in Volts (default: 2.5V)
        
        # IP filter for sensor data (default to sensor board IP)
        self.filter_source_ip = DEFAULT_SENSOR_IP  # Only accept data from this IP
        
        # Y-axis settings
        self.y_axis_auto_scale = True  # Auto-scale Y-axis by default
        self.y_axis_min = 0.0  # Minimum Y-axis value (psi)
        self.y_axis_max = 200.0  # Maximum Y-axis value (psi)
        
        # Data storage: sensor_id -> deque of (timestamp_ms, value)
        self.sensor_data: Dict[int, deque] = {}  # Voltage data for statistics display
        self.sensor_adc_codes: Dict[int, deque] = {}  # Store ADC codes for pressure calculation
        self.sensor_psi_data: Dict[int, deque] = {}  # PSI data for plotting (calibrated sensors only)
        self.sensor_plots: Dict[int, pg.PlotDataItem] = {}
        self.plot_enabled: Dict[int, bool] = {i: True for i in range(1, NUM_CONNECTORS + 1)}
        
        # Statistics
        self.stats_start_time = time.time()
        
        self.pt_calibration = load_pt_calibration(PT_CALIBRATION_CSV)
        
        # Sensor labels (connector_id -> label string)
        self.sensor_labels = {i: CONFIG.get_sensor_label(i) for i in range(1, NUM_CONNECTORS + 1)}
        
        # Load display settings from Config
        disp = CONFIG.config["display"]
        self.window_seconds = disp["window_seconds"]
        self.adc_bits = disp["adc_bits"]
        self.reference_voltage = disp["ref_voltage"]
        self.y_axis_min = disp["y_axis_min"]
        self.y_axis_max = disp["y_axis_max"]
        self.y_axis_auto_scale = disp["y_axis_autoscale"]
        self.filter_source_ip = CONFIG.config["network"]["sensor_ip_filter"]
        
        # Demo mode: synthetic PT data via UDP to localhost (same decode path as real hardware)
        self.demo_mode = False
        self.demo_start_time: Optional[float] = None
        self.demo_send_socket: Optional[socket.socket] = None
        self.demo_send_timer: Optional[QtCore.QTimer] = None
        
        self.init_ui()
        
        # Connect to receiver signals
        self.receiver.sensor_data_received.connect(self.on_sensor_data)
        self.receiver.status_update.connect(self.on_status_update)
        self.receiver.packet_received.connect(self.on_packet_received)
        
        # Timer for updating plots and statistics
        self.update_timer = QtCore.QTimer(self)
        self.update_timer.timeout.connect(self.update_plots)
        self.update_timer.start(UPDATE_INTERVAL_MS)
        
        # Timer for updating statistics display
        self.stats_timer = QtCore.QTimer(self)
        self.stats_timer.timeout.connect(self.update_statistics)
        self.stats_timer.start(500)  # Update stats every 500ms
    
    def on_sensor_label_changed(self, connector_id: int, text: str):
        """Handle sensor label text change"""
        CONFIG.set_sensor_label(connector_id, text)
        self.sensor_labels[connector_id] = text  # Update local cache
        
        # Update dashoard label widget if it exists
        if connector_id in self.sensor_label_inputs:
             self.sensor_label_inputs[connector_id].setText(text)
             
        # Update the plot legend if this sensor has a plot
        if connector_id in self.sensor_plots:
            self.update_plot_legend(connector_id)
    
    def update_plot_legend(self, connector_id: int):
        """Update the legend for a specific sensor plot"""
        if connector_id not in self.sensor_plots:
            return
        
        plot = self.sensor_plots[connector_id]
        label = self.sensor_labels.get(connector_id, "")
        if label:
            new_name = f"PT {connector_id}: {label} (psi)"
        else:
            new_name = f"PT {connector_id} (psi)"
        
        # Update legend by finding and modifying the legend item
        if self.legend:
            for item in self.legend.items:
                if len(item) >= 2 and item[0] == plot:
                    label_item = item[1]
                    label_item.setText(new_name)
                    break
    
    def init_ui(self):
        """Initialize the user interface"""
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Top panel with controls
        top_panel = QtWidgets.QHBoxLayout()
        
        # Connection info
        self.status_label = QtWidgets.QLabel("Starting...")
        self.status_label.setStyleSheet("font-weight: bold; padding: 5px;")
        self.status_label.setFixedWidth(400)  # Fixed width to prevent layout resize on text change
        top_panel.addWidget(self.status_label)
        
        top_panel.addStretch()
        
        # Y-axis auto-scale toggle
        self.auto_scale_checkbox = QtWidgets.QCheckBox("Auto-scale Y-axis")
        self.auto_scale_checkbox.setChecked(self.y_axis_auto_scale)
        self.auto_scale_checkbox.stateChanged.connect(self.on_auto_scale_toggled)
        self.auto_scale_checkbox.setStyleSheet("padding: 5px;")
        top_panel.addWidget(self.auto_scale_checkbox)
        
        layout.addLayout(top_panel)
        
        # Horizontal layout for plot and statistics
        plot_stats_layout = QtWidgets.QHBoxLayout()
        
        # Plot widget
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground('k')  # Black background
        plot_stats_layout.addWidget(self.plot_widget, 1)  # Takes most of the space
        
        # Statistics panel on the right
        stats_widget = QtWidgets.QWidget()
        stats_main_layout = QtWidgets.QVBoxLayout(stats_widget)
        stats_main_layout.setContentsMargins(0, 0, 0, 0)
        
        # Network statistics group
        network_stats_group = QtWidgets.QGroupBox("Network")
        network_stats_layout = QtWidgets.QVBoxLayout()
        
        self.packets_label = QtWidgets.QLabel("Packets: 0")
        self.pps_label = QtWidgets.QLabel("Packets/sec: 0.0")
        self.bytes_label = QtWidgets.QLabel("Bytes: 0")
        self.bps_label = QtWidgets.QLabel("Bytes/sec: 0.0")
        
        # Increase font size for statistics
        font = QtGui.QFont()
        font.setPointSize(10)
        self.packets_label.setFont(font)
        self.pps_label.setFont(font)
        self.bytes_label.setFont(font)
        self.bps_label.setFont(font)
        
        network_stats_layout.addWidget(self.packets_label)
        network_stats_layout.addWidget(self.pps_label)
        network_stats_layout.addWidget(self.bytes_label)
        network_stats_layout.addWidget(self.bps_label)
        network_stats_group.setLayout(network_stats_layout)
        stats_main_layout.addWidget(network_stats_group)
        
        # Connector statistics group with scrollable area
        connector_stats_group = QtWidgets.QGroupBox("Sensors")
        connector_stats_group_layout = QtWidgets.QVBoxLayout()
        
        # Moving average controls
        ma_group = QtWidgets.QGroupBox("Moving Average")
        ma_group_layout = QtWidgets.QVBoxLayout()
        
        # Graph moving average
        graph_ma_layout = QtWidgets.QHBoxLayout()
        graph_ma_layout.addWidget(QtWidgets.QLabel("Graph:"))
        self.graph_ma_spinbox = QtWidgets.QSpinBox()
        self.graph_ma_spinbox.setMinimum(1)
        self.graph_ma_spinbox.setMaximum(100)
        self.graph_ma_spinbox.setValue(self.graph_moving_avg_samples)
        self.graph_ma_spinbox.setSuffix(" samples")
        self.graph_ma_spinbox.valueChanged.connect(self.on_graph_moving_avg_changed)
        graph_ma_layout.addWidget(self.graph_ma_spinbox)
        ma_group_layout.addLayout(graph_ma_layout)
        
        # Display moving average
        display_ma_layout = QtWidgets.QHBoxLayout()
        display_ma_layout.addWidget(QtWidgets.QLabel("Display:"))
        self.display_ma_spinbox = QtWidgets.QSpinBox()
        self.display_ma_spinbox.setMinimum(1)
        self.display_ma_spinbox.setMaximum(100)
        self.display_ma_spinbox.setValue(self.display_moving_avg_samples)
        self.display_ma_spinbox.setSuffix(" samples")
        self.display_ma_spinbox.valueChanged.connect(self.on_display_moving_avg_changed)
        display_ma_layout.addWidget(self.display_ma_spinbox)
        ma_group_layout.addLayout(display_ma_layout)
        
        ma_group.setLayout(ma_group_layout)
        connector_stats_group_layout.addWidget(ma_group)
        
        # Scrollable area for connector values
        connector_scroll = QtWidgets.QScrollArea()
        connector_scroll.setWidgetResizable(True)
        connector_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        
        connector_stats_content = QtWidgets.QWidget()
        connector_stats_layout = QtWidgets.QVBoxLayout(connector_stats_content)
        connector_stats_layout.setSpacing(5)
        
        # Create labels and plot toggles for each connector
        self.connector_labels = {}
        self.connector_toggles: Dict[int, QtWidgets.QCheckBox] = {}
        self.sensor_label_inputs: Dict[int, QtWidgets.QLabel] = {}
        small_font = QtGui.QFont()
        small_font.setPointSize(9)
        tiny_font = QtGui.QFont()
        tiny_font.setPointSize(8)
        
        for i in range(1, NUM_CONNECTORS + 1):
            # Add label input field (only visible for calibrated sensors)
            if i in self.pt_calibration:
                # Replaced editable QLineEdit with read-only QLabel as requested
                label_input = QtWidgets.QLabel()
                label_input.setText(self.sensor_labels.get(i, ""))
                label_input.setFont(tiny_font)
                label_input.setStyleSheet("padding: 2px; margin-bottom: 2px; color: #AAAAAA;")
                
                connector_stats_layout.addWidget(label_input)
                self.sensor_label_inputs[i] = label_input
            
            # Data display row
            row = QtWidgets.QHBoxLayout()
            cb = QtWidgets.QCheckBox()
            cb.setChecked(True)
            cb.stateChanged.connect(lambda s, cid=i: self._on_plot_toggle(cid, s))
            self.connector_toggles[i] = cb
            row.addWidget(cb)
            label = QtWidgets.QLabel(f"C{i}: --- V")
            label.setFont(small_font)
            label.setTextFormat(QtCore.Qt.TextFormat.RichText)
            color_idx = i % len(SENSOR_COLORS)
            color = SENSOR_COLORS[color_idx]
            label.setStyleSheet(f"color: rgb{color}; padding: 2px;")
            row.addWidget(label)
            connector_stats_layout.addLayout(row)
            self.connector_labels[i] = label
            
            # Add spacing between sensors
            if i < NUM_CONNECTORS:
                connector_stats_layout.addSpacing(5)
        
        connector_stats_layout.addStretch()  # Push connectors to top
        connector_scroll.setWidget(connector_stats_content)
        connector_stats_group_layout.addWidget(connector_scroll)
        connector_stats_group.setLayout(connector_stats_group_layout)
        stats_main_layout.addWidget(connector_stats_group, 1)
        
        stats_widget.setFixedWidth(250)  # Fixed width for stats panel
        plot_stats_layout.addWidget(stats_widget)
        
        layout.addLayout(plot_stats_layout, 1)
        
        # Create initial plot
        self.plot_item = self.plot_widget.addPlot(title="Pressure Data Over Time (Calibrated Sensors)")
        
        # Set title color and size to white for visibility on black background
        self.plot_item.setTitle("Pressure Data Over Time (Calibrated Sensors)", color='w', size='14pt')
        
        # Set axis labels to white
        self.plot_item.setLabel('left', 'Pressure (psi)', color='w')
        self.plot_item.setLabel('bottom', 'Time (seconds)', color='w')
        
        self.plot_item.addLegend()
        # Show grid with white/gray lines for visibility on black background
        self.plot_item.showGrid(x=True, y=True, alpha=0.5)
        # Set grid color to light gray/white
        self.plot_item.getViewBox().setBackgroundColor('k')  # Ensure black background
        
        # Increase font size for axis labels and ticks
        font = QtGui.QFont()
        font.setPointSize(12)
        
        # Set all axis text to white
        left_axis = self.plot_item.getAxis('left')
        bottom_axis = self.plot_item.getAxis('bottom')
        
        # Set font size for ticks
        left_axis.setStyle(tickFont=font)
        bottom_axis.setStyle(tickFont=font)
        
        # Set label font size by accessing the label item directly
        try:
            left_axis.label.setFont(font)
            bottom_axis.label.setFont(font)
        except AttributeError:
            pass
        
        # Set axis line and text colors to white
        left_axis.setPen('w')
        bottom_axis.setPen('w')
        left_axis.setTextPen('w')
        bottom_axis.setTextPen('w')
        
        # Legend - make it visible on black background with white text
        self.legend = self.plot_item.legend
        if self.legend:
            self.legend.setBrush(pg.mkBrush('k'))  # Black background for legend
            self.legend.setPen(pg.mkPen('w'))  # White border
        
        # Pre-initialize plots for all 10 connectors
        self.init_connector_plots()
    
    def init_connector_plots(self):
        """Pre-initialize plots for all 10 connectors"""
        for connector_id in range(1, NUM_CONNECTORS + 1):
            self.sensor_data[connector_id] = deque(maxlen=MAX_POINTS)
            self.sensor_adc_codes[connector_id] = deque(maxlen=MAX_POINTS)
            # Only initialize PSI data and plots for calibrated sensors
            if connector_id in self.pt_calibration:
                self.sensor_psi_data[connector_id] = deque(maxlen=MAX_POINTS)
                self.add_sensor_plot(connector_id)
    
    def _on_plot_toggle(self, connector_id: int, state):
        self.plot_enabled[connector_id] = bool(state)
    
    
    def on_auto_scale_toggled(self, state):
        """Handle auto-scale Y-axis toggle"""
        self.y_axis_auto_scale = bool(state)
        CONFIG.config["display"]["y_axis_autoscale"] = self.y_axis_auto_scale
        CONFIG.save()
        # Update settings checkbox if it exists
        if hasattr(self, 'settings_widget_ref') and self.settings_widget_ref:
             # This is a bit circular, but we can emit a signal or just let the settings
             # tab refresh next time it is shown.
             pass
    
    def _clear_all_sensor_deques(self):
        """Clear all sensor data deques so plot/stats show only new data."""
        for k in list(self.sensor_data.keys()):
            self.sensor_data[k] = deque(maxlen=MAX_POINTS)
            self.sensor_adc_codes[k] = deque(maxlen=MAX_POINTS)
            if k in self.sensor_psi_data:
                self.sensor_psi_data[k] = deque(maxlen=MAX_POINTS)
    
    def on_demo_mode_toggled(self, state):
        """Handle Demo mode checkbox: toggle demo_mode, start/stop UDP sender, clear deques.
        
        Uses QTimer.singleShot to defer state changes to avoid Wayland buffer size mismatch
        crashes when the window is maximized (xdg_surface buffer error).
        """
        # Defer the actual state change to avoid Wayland crash when maximized
        QtCore.QTimer.singleShot(0, lambda: self._apply_demo_mode(bool(state)))
    
    def _apply_demo_mode(self, enabled: bool):
        """Apply demo mode state change (called via QTimer to avoid Wayland crashes)."""
        self.demo_mode = enabled
        if self.demo_mode:
            self.demo_start_time = time.time()
            self._clear_all_sensor_deques()
            self.status_label.setText("Demo mode – synthetic data (UDP)")
            try:
                self.demo_send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.demo_send_timer = QtCore.QTimer(self)
                self.demo_send_timer.timeout.connect(self._send_demo_packet)
                self.demo_send_timer.start(UPDATE_INTERVAL_MS)
            except OSError:
                self.demo_send_socket = None
                self.demo_send_timer = None
        else:
            self.demo_start_time = None
            if self.demo_send_timer is not None:
                self.demo_send_timer.stop()
                self.demo_send_timer = None
            if self.demo_send_socket is not None:
                try:
                    self.demo_send_socket.close()
                except OSError:
                    pass
                self.demo_send_socket = None
            self._clear_all_sensor_deques()
            self.status_label.setText("Listening")
    
    def on_graph_moving_avg_changed(self, value):
        """Handle graph moving average window size change"""
        self.graph_moving_avg_samples = value
    
    def on_display_moving_avg_changed(self, value):
        """Handle display moving average window size change"""
        self.display_moving_avg_samples = value
    
    def code_to_voltage(self, code_uint32: int) -> float:
        """
        Convert raw ADC code to voltage.
        code_uint32: uint32_t representation of the ADC code
        Returns: voltage in Volts
        """
        # Reinterpret uint32_t as int32_t (signed)
        if code_uint32 >= 0x80000000:
            code_int32 = code_uint32 - 0x100000000
        else:
            code_int32 = code_uint32
        
        # Convert to voltage using user-specified settings
        # For signed ADC: voltage = (code * ref_voltage) / (2^(bits-1))
        max_code = 2 ** (self.adc_bits - 1)
        voltage = (code_int32 * self.reference_voltage) / max_code
        return voltage
    
    def on_status_update(self, message: str):
        """Handle status updates from receiver thread"""
        if not self.demo_mode:
            self.status_label.setText(message)
    
    def on_packet_received(self, packet_size: int, packet_type: int):
        """Handle packet received notification"""
        pass  # Statistics are updated separately
    
    def on_sensor_data(self, header: dict, chunks: List[dict], source_ip: str):
        """Handle received sensor data"""
        if self.demo_mode:
            if source_ip != "127.0.0.1":
                return  # In demo mode only accept packets from local demo sender
        else:
            if source_ip != self.filter_source_ip:
                return  # Ignore data from other sources
        
        current_time = time.time()
        
        for chunk in chunks:
            chunk_timestamp_ms = chunk['timestamp']
            # Convert packet timestamp to relative time in seconds
            relative_time = (current_time - self.stats_start_time)
            
            for dp in chunk['datapoints']:
                sensor_id = dp['sensor_id']
                code_uint32 = dp['data']  # Received as uint32_t from protocol
                
                # Convert code to voltage
                voltage = self.code_to_voltage(code_uint32)
                
                # Initialize sensor data storage if needed (for sensors outside 1-10 range)
                if sensor_id not in self.sensor_data:
                    self.sensor_data[sensor_id] = deque(maxlen=MAX_POINTS)
                    self.sensor_adc_codes[sensor_id] = deque(maxlen=MAX_POINTS)
                    # Only create PSI storage and plot for calibrated sensors
                    if sensor_id in self.pt_calibration:
                        self.sensor_psi_data[sensor_id] = deque(maxlen=MAX_POINTS)
                        self.add_sensor_plot(sensor_id)
                
                # Add data point (use relative time from start)
                self.sensor_data[sensor_id].append((relative_time, voltage))
                self.sensor_adc_codes[sensor_id].append((relative_time, code_uint32))
                
                # Calculate and store PSI if calibration exists
                if sensor_id in self.pt_calibration:
                    a, b, c, d = self.pt_calibration[sensor_id]
                    psi = calculate_pressure(code_uint32, a, b, c, d)
                    self.sensor_psi_data[sensor_id].append((relative_time, psi))
    
    def _send_demo_packet(self):
        """Build and send one demo UDP packet to localhost (called by demo timer)."""
        if not self.demo_mode or self.demo_start_time is None or not self.pt_calibration:
            return
        packet = build_demo_packet(
            self.pt_calibration,
            self.demo_start_time,
            self.stats_start_time,
            packet_type=PacketType.SENSOR_DATA,
            version=DIABLO_COMMS_VERSION,
            max_packet_size=MAX_PACKET_SIZE,
        )
        if packet and self.demo_send_socket is not None:
            try:
                self.demo_send_socket.sendto(
                    packet, ("127.0.0.1", self.receiver.port)
                )
            except OSError:
                pass
    
    def add_sensor_plot(self, sensor_id: int):
        """Add a new sensor plot (for calibrated sensors only)"""
        if sensor_id not in self.plot_enabled:
            self.plot_enabled[sensor_id] = True
        color_idx = sensor_id % len(SENSOR_COLORS)
        color = SENSOR_COLORS[color_idx]
        
        # Get label for this sensor
        label = self.sensor_labels.get(sensor_id, "")
        if label:
            plot_name = f"PT {sensor_id}: {label} (psi)"
        else:
            plot_name = f"PT {sensor_id} (psi)"
        
        pen = pg.mkPen(color=color, width=2)
        plot = self.plot_item.plot([], [], pen=pen, name=plot_name)
        
        # Update legend text color to white for this new item
        if self.legend:
            # Find the legend item for this plot and set text color to white
            for item in self.legend.items:
                if len(item) >= 2 and item[0] == plot:
                    label_item = item[1]
                    # Set text color to white
                    label_item.setColor('w')
        
        self.sensor_plots[sensor_id] = plot
    
    def update_plots(self):
        """Update all sensor plots (PSI data for calibrated sensors only)"""
        if not self.sensor_psi_data:
            return
        
        current_time = time.time() - self.stats_start_time
        time_window = self.window_seconds
        
        # Only plot calibrated sensors that have PSI data
        for sensor_id, psi_deque in self.sensor_psi_data.items():
            if sensor_id not in self.sensor_plots:
                continue
            enabled = self.plot_enabled.get(sensor_id, True)
            self.sensor_plots[sensor_id].setVisible(enabled)
            if not enabled:
                continue
            if len(psi_deque) == 0:
                continue
            
            # Extract time and PSI value arrays
            times = []
            psi_values = []
            
            for t, psi in psi_deque:
                # Only show data within the time window
                if current_time - t <= time_window:
                    times.append(t)
                    psi_values.append(psi)
            
            if len(times) > 0:
                # Convert to numpy arrays
                times_array = np.array(times)
                psi_array = np.array(psi_values)
                
                # Apply moving average smoothing to graph if window > 1
                if self.graph_moving_avg_samples > 1 and len(psi_array) >= self.graph_moving_avg_samples:
                    # Use convolution for efficient moving average
                    kernel = np.ones(self.graph_moving_avg_samples) / self.graph_moving_avg_samples
                    smoothed_psi = np.convolve(psi_array, kernel, mode='valid')
                    # Adjust times array to match smoothed data length
                    smoothed_times = times_array[self.graph_moving_avg_samples - 1:]
                    
                    # Update plot with smoothed data
                    self.sensor_plots[sensor_id].setData(smoothed_times, smoothed_psi)
                else:
                    # Update plot with raw data
                    self.sensor_plots[sensor_id].setData(times_array, psi_array)
        
        # Update x-axis range
        if current_time > time_window:
            self.plot_item.setXRange(current_time - time_window, current_time, padding=0)
        else:
            self.plot_item.setXRange(0, time_window, padding=0)
        
        # Update y-axis range based on auto-scale setting
        if self.y_axis_auto_scale:
            self.plot_item.enableAutoRange(axis='y')
        else:
            self.plot_item.setYRange(self.y_axis_min, self.y_axis_max, padding=0)
            self.plot_item.disableAutoRange(axis='y')
    
    def update_statistics(self):
        """Update statistics display"""
        if self.receiver is None:
            return
        
        stats = self.receiver.get_stats()
        
        self.packets_label.setText(f"Packets: {stats['packets']}")
        self.pps_label.setText(f"Packets/sec: {stats['packets_per_sec']:.2f}")
        
        # Format bytes
        bytes_val = stats['bytes']
        if bytes_val < 1024:
            bytes_str = f"{bytes_val} B"
        elif bytes_val < 1024 * 1024:
            bytes_str = f"{bytes_val / 1024:.2f} KB"
        else:
            bytes_str = f"{bytes_val / (1024 * 1024):.2f} MB"
        self.bytes_label.setText(f"Bytes: {bytes_str}")
        
        # Format bytes per second
        bps = stats['bytes_per_sec']
        if bps < 1024:
            bps_str = f"{bps:.2f} B/s"
        elif bps < 1024 * 1024:
            bps_str = f"{bps / 1024:.2f} KB/s"
        else:
            bps_str = f"{bps / (1024 * 1024):.2f} MB/s"
        self.bps_label.setText(f"Bytes/sec: {bps_str}")
        
        # Update per-connector statistics
        for connector_id in range(1, NUM_CONNECTORS + 1):
            if connector_id in self.sensor_data and len(self.sensor_data[connector_id]) > 0:
                # Get latest values
                values = [v for t, v in self.sensor_data[connector_id]]
                if values:
                    current = values[-1]
                    
                    # Calculate moving average over last N samples for display
                    n_samples = min(self.display_moving_avg_samples, len(values))
                    moving_avg = sum(values[-n_samples:]) / n_samples
                    
                    text = f"C{connector_id}: {current:.4f} V<br/>  MA: {moving_avg:.4f} V"
                    # Show psi for PTs when calibration is loaded (larger font)
                    if connector_id in self.pt_calibration:
                        # Get ADC code values for pressure calculation
                        if connector_id in self.sensor_adc_codes and len(self.sensor_adc_codes[connector_id]) > 0:
                            adc_values = [code for t, code in self.sensor_adc_codes[connector_id]]
                            n_samples_adc = min(self.display_moving_avg_samples, len(adc_values))
                            moving_avg_adc = sum(adc_values[-n_samples_adc:]) / n_samples_adc
                            
                            a, b, c, d = self.pt_calibration[connector_id]
                            psi = calculate_pressure(moving_avg_adc, a, b, c, d)
                            text += f"<br/><span style='font-size: 16pt; font-weight: bold'>{psi:.2f} psi</span>"
                    self.connector_labels[connector_id].setText(text)
                else:
                    self.connector_labels[connector_id].setText(f"C{connector_id}: --- V")
            else:
                self.connector_labels[connector_id].setText(f"C{connector_id}: --- V")
    



# ---------------------- Sensor Plot Window (standalone) ----------------------
class SensorPlotWindow(QtWidgets.QMainWindow):
    """Standalone window that embeds SensorPlotWidget."""
    def __init__(self, receiver, bind_address: str = '0.0.0.0'):
        super().__init__()
        self.setWindowTitle(f"Sensor Data Receiver - Port {receiver.port}")
        self.setGeometry(100, 100, 1200, 800)
        self.setCentralWidget(SensorPlotWidget(receiver, bind_address, self))
    
    def closeEvent(self, event):
        """Handle window close event"""
        w = self.centralWidget()
        if w and hasattr(w, 'save_sensor_labels'):
            w.save_sensor_labels()
        event.accept()


# ---------------------- Actuator Control Widget (reusable panel) ----------------------
class ActuatorControlWidget(QtWidgets.QWidget):
    """Reusable actuator control panel. Used in ActuatorControlWindow and CombinedMainWindow."""
    def __init__(self, receiver, device_ip: str = None, device_port: int = None, parent=None):
        super().__init__(parent)
        self.receiver = receiver
        # Load from CONFIG (ignoring args if None, effectively preferring config)
        self.device_ip = CONFIG.config["network"]["actuator_ip"]
        self.device_port = CONFIG.config["network"]["actuator_port"]
        
        # Actuator state tracking (1-indexed: 1-10)
        # 0 = OFF, 1 = ON
        self.actuator_states = [0] * NUM_ACTUATORS
        
        # Voltage readings (0-indexed: 0-9, maps to actuator 1-10)
        # Store as voltage in Volts
        self.voltage_readings = [0.0] * NUM_ACTUATORS
        
        # Actuator labels (1-indexed: 1-10)
        self.actuator_labels = {i: CONFIG.get_actuator_label(i) for i in range(1, NUM_ACTUATORS + 1)}
        
        # UDP socket for sending commands
        self.command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.init_ui()
        
        # Connect to receiver signals
        self.receiver.sensor_data_received.connect(self.on_sensor_data)
        self.receiver.status_update.connect(self.on_status_update)
        
        # Timer for updating current display
        self.update_timer = QtCore.QTimer(self)
        self.update_timer.timeout.connect(self.update_current_display)
        self.update_timer.start(100)  # Update every 100ms
    

    
        self.command_sock = None
    
    def on_label_changed(self, actuator_id: int, text: str):
        """Handle label text change (internal or external)"""
        CONFIG.set_actuator_label(actuator_id, text)
        self.actuator_labels[actuator_id] = text
        self.update_label_display(actuator_id, text)
    
    def init_ui(self):
        """Initialize the user interface"""
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Top panel with status
        top_panel = QtWidgets.QHBoxLayout()
        
        self.status_label = QtWidgets.QLabel("Ready")
        self.status_label.setStyleSheet("font-weight: bold; padding: 5px;")
        top_panel.addWidget(self.status_label)
        
        top_panel.addStretch()
        
        layout.addLayout(top_panel)
        
        # Main content area with actuators in a grid: 2 columns x 5 rows
        grid_container = QtWidgets.QWidget()
        grid_layout = QtWidgets.QGridLayout(grid_container)
        grid_layout.setSpacing(10)
        
        # Create actuator controls in a 2x5 grid
        self.actuator_widgets = []
        for i in range(NUM_ACTUATORS):
            actuator_id = i + 1  # 1-indexed
            
            # Calculate grid position: 2 columns, 5 rows
            row = i // 2  # 0-4
            col = i % 2   # 0 or 1
            
            # Create widget for each actuator
            actuator_frame = QtWidgets.QFrame()
            actuator_frame.setFrameShape(QtWidgets.QFrame.Shape.Box)
            actuator_frame.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
            actuator_frame.setStyleSheet("padding: 10px; margin: 5px;")
            
            actuator_layout = QtWidgets.QVBoxLayout(actuator_frame)
            
            # Actuator ID label
            # If label exists, show label. Else show "Actuator {id}"
            label_text = self.actuator_labels.get(actuator_id, "") or f"Actuator {actuator_id}"
            id_label = QtWidgets.QLabel(label_text)
            id_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            id_label.setStyleSheet("font-weight: bold; font-size: 12pt; padding: 5px;")
            actuator_layout.addWidget(id_label)
            
            # Button container
            button_container = QtWidgets.QHBoxLayout()
            button_container.setSpacing(5)
            
            # ON button
            on_btn = QtWidgets.QPushButton("ON")
            on_btn.setMinimumHeight(40)
            bg_color = self.palette().color(QtGui.QPalette.ColorRole.Window).name()
            on_btn.setStyleSheet(f"""
                QPushButton {{
                    font-size: 11pt;
                    font-weight: bold;
                    background-color: {bg_color};
                    color: #FFFFFF;
                    border: none;
                    border-radius: 5px;
                    padding: 5px;
                }}
            """)
            on_btn.clicked.connect(lambda checked=False, aid=actuator_id: self.set_actuator_state(aid, 1))
            
            # OFF button
            off_btn = QtWidgets.QPushButton("OFF")
            off_btn.setMinimumHeight(40)
            off_btn.setStyleSheet(f"""
                QPushButton {{
                    font-size: 11pt;
                    font-weight: bold;
                    background-color: {bg_color};
                    color: #FFFFFF;
                    border: none;
                    border-radius: 5px;
                    padding: 5px;
                }}
            """)
            off_btn.clicked.connect(lambda checked=False, aid=actuator_id: self.set_actuator_state(aid, 0))
            
            button_container.addWidget(on_btn)
            button_container.addWidget(off_btn)
            actuator_layout.addLayout(button_container)
            
            # Voltage reading label
            voltage_label = QtWidgets.QLabel("Voltage: 0.000 V")
            voltage_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            voltage_label.setStyleSheet("font-size: 10pt; padding: 5px;")
            actuator_layout.addWidget(voltage_label)
            
            # Add to grid
            grid_layout.addWidget(actuator_frame, row, col)
            
            # Store widget references
            self.actuator_widgets.append({
                'frame': actuator_frame,
                'on_btn': on_btn,
                'off_btn': off_btn,
                'voltage_label': voltage_label,
                'id_label': id_label
            })
        
        layout.addWidget(grid_container, 1)
        
        # Initialize all actuators to OFF state (highlight OFF buttons)
        for i in range(NUM_ACTUATORS):
            self.update_button_highlight(i, 0)

    def update_label_display(self, actuator_id, text):
        """External method to update label display when changed in settings"""
        idx = actuator_id - 1
        if 0 <= idx < len(self.actuator_widgets):
             new_text = text or f"Actuator {actuator_id}"
             self.actuator_widgets[idx]['id_label'].setText(new_text)

    def on_label_changed(self, actuator_id: int, text: str):
        """Handle label text change (internal or external)"""
        CONFIG.set_actuator_label(actuator_id, text)
        self.actuator_labels[actuator_id] = text
        self.update_label_display(actuator_id, text)
    
    def on_status_update(self, message: str):
        """Handle status updates from receiver thread"""
        pass
    
    def on_sensor_data(self, header: dict, chunks: List[dict], source_ip: str):
        """Handle received sensor data (voltage readings)"""
        if chunks:
            latest_chunk = chunks[-1]
            for dp in latest_chunk['datapoints']:
                sensor_id = dp['sensor_id']  # 1-indexed (1-10)
                code_uint32 = dp['data']  # Received as uint32_t from protocol
                
                if code_uint32 >= 0x80000000:
                    code_int32 = code_uint32 - 0x100000000
                else:
                    code_int32 = code_uint32
                
                voltage = (code_int32 * 2.5) / 2147483648.0
                array_idx = sensor_id - 1
                if 0 <= array_idx < NUM_ACTUATORS:
                    self.voltage_readings[array_idx] = voltage
    
    def update_button_highlight(self, array_idx: int, actuator_state: int):
        """Update button highlighting based on actuator state."""
        widget = self.actuator_widgets[array_idx]
        bg_color = widget['frame'].palette().color(QtGui.QPalette.ColorRole.Window).name()
        inactive_style = f"""
            QPushButton {{
                font-size: 11pt;
                font-weight: bold;
                background-color: {bg_color};
                color: #FFFFFF;
                border: none;
                border-radius: 5px;
                padding: 5px;
            }}
        """
        active_style = """
            QPushButton {
                font-size: 11pt;
                font-weight: bold;
                background-color: #FFFFFF;
                color: #000000;
                border: 2px solid #000000;
                border-radius: 5px;
                padding: 5px;
            }
        """
        if actuator_state == 1:
            widget['on_btn'].setStyleSheet(active_style)
            widget['off_btn'].setStyleSheet(inactive_style)
        else:
            widget['on_btn'].setStyleSheet(inactive_style)
            widget['off_btn'].setStyleSheet(active_style)
    
    def set_actuator_state(self, actuator_id: int, actuator_state: int):
        """Set actuator state and send command packet."""
        array_idx = actuator_id - 1
        self.actuator_states[array_idx] = actuator_state
        self.update_button_highlight(array_idx, actuator_state)
        self.send_actuator_command(actuator_id, actuator_state)
    
    def send_actuator_command(self, actuator_id: int, actuator_state: int):
        """Send an actuator command packet to the device."""
        try:
            commands = [(actuator_id, actuator_state)]
            packet = create_actuator_command_packet(commands)
            if len(packet) > 0:
                self.command_sock.sendto(packet, (self.device_ip, self.device_port))
                print(f"Sent command: Actuator {actuator_id} -> {'ON' if actuator_state else 'OFF'}")
            else:
                print(f"Error: Failed to create packet for actuator {actuator_id}")
        except OSError as e:
            err = e.errno
            if err == 65:
                msg = f"No route to host — check device IP ({self.device_ip})"
            elif err == 64:
                msg = f"Network unreachable — check WiFi/Ethernet"
            else:
                msg = f"Network error: [{err}] {e}"
            print(f"Error sending command: {e}")
            self.status_label.setText(msg)
        except Exception as e:
            print(f"Error sending command: {e}")
            self.status_label.setText(f"Error: {e}")
    
    def update_current_display(self):
        """Update the voltage reading display for all actuators"""
        for i in range(NUM_ACTUATORS):
            voltage = self.voltage_readings[i]
            widget = self.actuator_widgets[i]
            widget['voltage_label'].setText(f"Voltage: {voltage:.3f} V")
    



# ---------------------- Top Bar Widgets ----------------------
class PressureBarWidget(QtWidgets.QWidget):
    """
    Vertical bar gauge showing pressure relative to NOP and MEOP.
    Range: 0 to 1.2 * MEOP
    """
    def __init__(self, title: str, nop: float = 500.0, meop: float = 700.0, fixed_color: QtGui.QColor = None, parent=None):
        super().__init__(parent)
        self.title = title
        self.nop = nop
        self.meop = meop
        self.fixed_color = fixed_color
        self.current_value = 0.0
        self.setMinimumWidth(60)
        self.setMinimumHeight(100)
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Fixed, QtWidgets.QSizePolicy.Policy.Expanding)
        
    def set_value(self, value: float):
        self.current_value = value
        self.update()  # Trigger repaint

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        
        # Geometry
        w = self.width()
        h = self.height()
        
        # Margins (room for text)
        top_margin = 20
        bottom_margin = 20
        bar_w = w - 20
        bar_x = 10
        bar_h = h - top_margin - bottom_margin
        bar_y = top_margin
        
        # Draw Title
        painter.setPen(QtCore.Qt.GlobalColor.white)
        painter.drawText(QtCore.QRect(0, 0, w, top_margin), QtCore.Qt.AlignmentFlag.AlignCenter, self.title)
        
        # Draw Background Bar
        painter.setPen(QtCore.Qt.GlobalColor.gray)
        painter.setBrush(QtGui.QColor(50, 50, 50))
        painter.drawRect(bar_x, bar_y, bar_w, bar_h)
        
        # Scale calculation
        max_val = 1.2 * self.meop
        if max_val <= 0: max_val = 1.0
        
        # Draw Current Level
        fill_ratio = min(max(self.current_value / max_val, 0.0), 1.0)
        fill_h = int(fill_ratio * bar_h)
        fill_y = bar_y + bar_h - fill_h
        
        # Color based on value
        if self.fixed_color:
            painter.setBrush(self.fixed_color)
        else:
            if self.current_value > self.meop:
                painter.setBrush(QtGui.QColor(255, 0, 0)) # Red
            elif self.current_value > self.nop:
                painter.setBrush(QtGui.QColor(255, 165, 0)) # Orange
            else:
                painter.setBrush(QtGui.QColor(0, 255, 0)) # Green
            
        painter.setPen(QtCore.Qt.PenStyle.NoPen)
        painter.drawRect(bar_x, fill_y, bar_w, fill_h)
        
        # Draw Lines for NOP and MEOP
        painter.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.white, 1, QtCore.Qt.PenStyle.DotLine))
        
        # NOP Line
        nop_ratio = self.nop / max_val
        nop_y = bar_y + bar_h - int(nop_ratio * bar_h)
        if 0 <= nop_ratio <= 1:
            painter.drawLine(bar_x, nop_y, bar_x + bar_w, nop_y)
            
        # MEOP Line
        meop_ratio = self.meop / max_val
        meop_y = bar_y + bar_h - int(meop_ratio * bar_h)
        if 0 <= meop_ratio <= 1:
            painter.drawLine(bar_x, meop_y, bar_x + bar_w, meop_y)

        # Draw Value Text
        painter.setPen(QtCore.Qt.GlobalColor.white)
        val_str = f"{self.current_value:.0f}"
        painter.drawText(QtCore.QRect(0, h - bottom_margin, w, bottom_margin), QtCore.Qt.AlignmentFlag.AlignCenter, val_str)


class TopBarWidget(QtWidgets.QWidget):
    navigation_requested = QtCore.pyqtSignal(str)  # "dashboard" or "settings"

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(120)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(10, 5, 10, 5)
        
        # Pressure Bars
        # Colors: GN2=Green, ETH=Red, LOX=Blue
        self.bar_gn2 = PressureBarWidget("GN2", fixed_color=QtGui.QColor(0, 255, 0))
        self.bar_eth = PressureBarWidget("ETH", fixed_color=QtGui.QColor(255, 0, 0))
        self.bar_lox = PressureBarWidget("LOX", fixed_color=QtGui.QColor(0, 0, 255))
        
        layout.addWidget(self.bar_gn2)
        layout.addWidget(self.bar_eth)
        layout.addWidget(self.bar_lox)
        
        layout.addStretch()
        
        # Buttons Layout
        btn_layout = QtWidgets.QVBoxLayout()
        
        # Abort Buttons Row
        abort_row = QtWidgets.QHBoxLayout()
        self.btn_abort = QtWidgets.QPushButton("ABORT")
        self.btn_abort.setMinimumSize(120, 40)
        self.btn_abort.setStyleSheet("background-color: orange; font-weight: bold; color: black;")
        self.btn_abort.clicked.connect(self.abort)
        
        self.btn_emergency = QtWidgets.QPushButton("EMERGENCY ABORT")
        self.btn_emergency.setMinimumSize(120, 40)
        self.btn_emergency.setStyleSheet("background-color: red; font-weight: bold; color: white;")
        self.btn_emergency.clicked.connect(self.abort)
        
        abort_row.addWidget(self.btn_abort)
        abort_row.addWidget(self.btn_emergency)
        btn_layout.addLayout(abort_row)
        
        # Navigation Button
        self.nav_btn = QtWidgets.QPushButton("SETTINGS")
        self.nav_btn.setMinimumHeight(30)
        self.nav_btn.clicked.connect(self.toggle_view)
        btn_layout.addWidget(self.nav_btn)
        
        layout.addLayout(btn_layout)
        
    def abort(self):
        """Placeholder for abort functionality."""
        print("ABORT TRIGGERED")

    def toggle_view(self):
        text = self.nav_btn.text()
        if text == "SETTINGS":
            self.navigation_requested.emit("settings")
            self.nav_btn.setText("DASHBOARD")
        else:
            self.navigation_requested.emit("dashboard")
            self.nav_btn.setText("SETTINGS")


# ---------------------- Settings Widget ----------------------
class SettingsWidget(QtWidgets.QWidget):
    """
    Centralized settings configuration widget.
    """
    mapping_changed = QtCore.pyqtSignal(str, int)  # gauge_name (GN2/ETH/LOX), pt_id
    
    def __init__(self, sensor_widget, actuator_widget, parent=None):
        super().__init__(parent)
        self.sensor_widget = sensor_widget
        self.actuator_widget = actuator_widget
        
        # Default mappings
        self.gauge_mappings = {
            "GN2": 0,
            "ETH": 0,
            "LOX": 0
        }
        
        self.init_ui()
        self.load_values()

    def init_ui(self):
        main_layout = QtWidgets.QHBoxLayout(self)
        
        # Left Column: Sensor & General Settings
        left_col = QtWidgets.QVBoxLayout()
        
        # ... Sensor Settings ...
        sensor_group = QtWidgets.QGroupBox("Sensor & General View Settings")
        sensor_layout = QtWidgets.QFormLayout()
        
        self.demo_chk = QtWidgets.QCheckBox("Demo Mode")
        self.demo_chk.toggled.connect(self.sensor_widget.on_demo_mode_toggled)
        sensor_layout.addRow(self.demo_chk)
        
        self.time_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.time_slider.setRange(1, 60)
        self.time_lbl = QtWidgets.QLabel("10s")
        self.time_slider.valueChanged.connect(self.on_time_changed)
        sensor_layout.addRow("Time Window:", self.time_slider)
        sensor_layout.addRow("", self.time_lbl)
        
        self.y_min_spin = QtWidgets.QDoubleSpinBox()
        self.y_min_spin.setRange(-1000, 10000)
        self.y_min_spin.valueChanged.connect(self.on_y_min_changed)
        sensor_layout.addRow("Y Min:", self.y_min_spin)
        
        self.y_max_spin = QtWidgets.QDoubleSpinBox()
        self.y_max_spin.setRange(-1000, 10000)
        self.y_max_spin.valueChanged.connect(self.on_y_max_changed)
        sensor_layout.addRow("Y Max:", self.y_max_spin)

        self.ip_filter = QtWidgets.QLineEdit()
        self.ip_filter.textChanged.connect(self.on_ip_filter_changed)
        sensor_layout.addRow("Sensor IP Filter:", self.ip_filter)
        
        sensor_group.setLayout(sensor_layout)
        left_col.addWidget(sensor_group)
        
        # ADC Settings
        adc_group = QtWidgets.QGroupBox("ADC Configuration")
        adc_layout = QtWidgets.QFormLayout()
        self.adc_bits_spin = QtWidgets.QSpinBox()
        self.adc_bits_spin.setRange(8, 32)
        self.adc_bits_spin.valueChanged.connect(self.on_adc_bits_changed)
        adc_layout.addRow("ADC Bits:", self.adc_bits_spin)
        
        self.ref_volt_spin = QtWidgets.QDoubleSpinBox()
        self.ref_volt_spin.setRange(0.1, 10.0)
        self.ref_volt_spin.setValue(2.5)
        self.ref_volt_spin.valueChanged.connect(self.on_ref_volt_changed)
        adc_layout.addRow("Ref Voltage (V):", self.ref_volt_spin)
        adc_group.setLayout(adc_layout)
        left_col.addWidget(adc_group)
        
        left_col.addStretch()
        main_layout.addLayout(left_col, 1)
        
        # Middle Column: Actuator Configuration
        mid_col = QtWidgets.QVBoxLayout()
        act_group = QtWidgets.QGroupBox("Actuator Configuration")
        act_layout = QtWidgets.QVBoxLayout()
        
        form = QtWidgets.QFormLayout()
        self.act_ip = QtWidgets.QLineEdit()
        self.act_ip.textChanged.connect(self.on_act_ip_changed)
        form.addRow("Device IP:", self.act_ip)
        
        self.act_port = QtWidgets.QSpinBox()
        self.act_port.setRange(1, 65535)
        self.act_port.valueChanged.connect(self.on_act_port_changed)
        form.addRow("Device Port:", self.act_port)
        act_layout.addLayout(form)
        
        act_layout.addWidget(QtWidgets.QLabel("Actuator Names:"))
        self.act_scroll = QtWidgets.QScrollArea()
        self.act_scroll_widget = QtWidgets.QWidget()
        self.act_form = QtWidgets.QFormLayout(self.act_scroll_widget)
        self.act_inputs = {}
        for i in range(1, NUM_ACTUATORS + 1):
            le = QtWidgets.QLineEdit()
            le.textChanged.connect(lambda txt, idx=i: self.on_actuator_name_changed(idx, txt))
            self.act_form.addRow(f"Actuator {i}:", le)
            self.act_inputs[i] = le
        self.act_scroll.setWidget(self.act_scroll_widget)
        self.act_scroll.setWidgetResizable(True)
        act_layout.addWidget(self.act_scroll)
        
        act_group.setLayout(act_layout)
        mid_col.addWidget(act_group)
        main_layout.addLayout(mid_col, 1)
        
        # Right Column: PT Configuration & Gauge Mapping
        right_col = QtWidgets.QVBoxLayout()
        pt_group = QtWidgets.QGroupBox("Pressure Transducers (PT) & Mapping")
        pt_layout = QtWidgets.QVBoxLayout()
        
        # Gauge Mapping (Moved to top as requested)
        mapping_group = QtWidgets.QGroupBox("Top Bar Gauge Mapping")
        mapping_form = QtWidgets.QFormLayout()
        
        self.combo_gn2 = QtWidgets.QComboBox()
        self.combo_eth = QtWidgets.QComboBox()
        self.combo_lox = QtWidgets.QComboBox()
        
        self.combos = {
            "GN2": self.combo_gn2,
            "ETH": self.combo_eth,
            "LOX": self.combo_lox
        }
        
        for name, combo in self.combos.items():
            combo.addItem("None", 0)
            for pt_id in sorted(self.sensor_widget.pt_calibration.keys()):
                combo.addItem(f"PT {pt_id}", pt_id)
            # Reverted to currentIndexChanged now that we block signals in load_values
            combo.currentIndexChanged.connect(lambda idx, n=name, c=combo: self.on_mapping_changed(n, c))
            mapping_form.addRow(f"{name} Source:", combo)
            
        mapping_group.setLayout(mapping_form)
        pt_layout.addWidget(mapping_group)
        
        # PT Names List
        pt_layout.addWidget(QtWidgets.QLabel("PT Names (Calibrated):"))
        self.pt_scroll = QtWidgets.QScrollArea()
        self.pt_scroll_widget = QtWidgets.QWidget()
        self.pt_form = QtWidgets.QFormLayout(self.pt_scroll_widget)
        self.pt_inputs = {}
        
        # Populate PT inputs based on calibration
        for pt_id in sorted(self.sensor_widget.pt_calibration.keys()):
            le = QtWidgets.QLineEdit()
            le.textChanged.connect(lambda txt, idx=pt_id: self.on_pt_name_changed(idx, txt))
            self.pt_form.addRow(f"PT {pt_id}:", le)
            self.pt_inputs[pt_id] = le
            
        self.pt_scroll.setWidget(self.pt_scroll_widget)
        self.pt_scroll.setWidgetResizable(True)
        pt_layout.addWidget(self.pt_scroll)
        
        pt_group.setLayout(pt_layout)
        right_col.addWidget(pt_group)
        main_layout.addLayout(right_col, 1)

    def load_values(self):
        # Sensor
        with QtCore.QSignalBlocker(self.demo_chk):
            self.demo_chk.setChecked(self.sensor_widget.demo_mode)
        
        with QtCore.QSignalBlocker(self.time_slider):
            self.time_slider.setValue(int(self.sensor_widget.window_seconds))
            
        with QtCore.QSignalBlocker(self.y_min_spin):
            self.y_min_spin.setValue(self.sensor_widget.y_axis_min)
            
        with QtCore.QSignalBlocker(self.y_max_spin):
            self.y_max_spin.setValue(self.sensor_widget.y_axis_max)
            
        with QtCore.QSignalBlocker(self.ip_filter):
            self.ip_filter.setText(self.sensor_widget.filter_source_ip)
            
        with QtCore.QSignalBlocker(self.adc_bits_spin):
            self.adc_bits_spin.setValue(self.sensor_widget.adc_bits)
            
        with QtCore.QSignalBlocker(self.ref_volt_spin):
            self.ref_volt_spin.setValue(self.sensor_widget.reference_voltage)
        
        # Actuator
        with QtCore.QSignalBlocker(self.act_ip):
            self.act_ip.setText(self.actuator_widget.device_ip)
            
        with QtCore.QSignalBlocker(self.act_port):
            self.act_port.setValue(self.actuator_widget.device_port)
            
        for i, le in self.act_inputs.items():
            with QtCore.QSignalBlocker(le):
                le.setText(self.actuator_widget.actuator_labels.get(i, ""))
            
        # PT
        for i, le in self.pt_inputs.items():
            with QtCore.QSignalBlocker(le):
                le.setText(self.sensor_widget.sensor_labels.get(i, ""))
        
        # Mappings
        for name, combo in self.combos.items():
            pt_id = CONFIG.config["mappings"].get(name, 0)
            idx = combo.findData(pt_id)
            if idx >= 0:
                with QtCore.QSignalBlocker(combo):
                    combo.setCurrentIndex(idx)
            
    def on_time_changed(self, val):
        self.time_lbl.setText(f"{val}s")
        self.sensor_widget.window_seconds = float(val)
        CONFIG.config["display"]["window_seconds"] = float(val)
        CONFIG.save()

    def on_y_min_changed(self, val):
        self.sensor_widget.y_axis_min = val
        CONFIG.config["display"]["y_axis_min"] = val
        CONFIG.save()

    def on_y_max_changed(self, val):
        self.sensor_widget.y_axis_max = val
        CONFIG.config["display"]["y_axis_max"] = val
        CONFIG.save()

    def on_ip_filter_changed(self, text):
        self.sensor_widget.filter_source_ip = text
        CONFIG.config["network"]["sensor_ip_filter"] = text
        CONFIG.save()

    def on_adc_bits_changed(self, val):
        self.sensor_widget.adc_bits = val
        CONFIG.config["display"]["adc_bits"] = val
        CONFIG.save()

    def on_ref_volt_changed(self, val):
        self.sensor_widget.reference_voltage = val
        CONFIG.config["display"]["ref_voltage"] = val
        CONFIG.save()

    def on_act_ip_changed(self, text):
        self.actuator_widget.device_ip = text
        CONFIG.config["network"]["actuator_ip"] = text
        CONFIG.save()

    def on_act_port_changed(self, val):
        self.actuator_widget.device_port = val
        CONFIG.config["network"]["actuator_port"] = val
        CONFIG.save()

    def on_actuator_name_changed(self, actuator_id, text):
        self.actuator_widget.on_label_changed(actuator_id, text)
        
    def on_pt_name_changed(self, pt_id, text):
        self.sensor_widget.on_sensor_label_changed(pt_id, text)
        self.update_combo_text(pt_id, text)
        
    def update_combo_text(self, pt_id, label):
        # Update ComboBox user-visible text if needed (optional polish)
        pass

    def on_mapping_changed(self, gauge_name, combo):
        pt_id = combo.currentData()
        self.mapping_changed.emit(gauge_name, pt_id)
        CONFIG.config["mappings"][gauge_name] = pt_id
        CONFIG.save()


# ---------------------- Actuator Control Window (standalone) ----------------------
class ActuatorControlWindow(QtWidgets.QMainWindow):
    """Standalone window that embeds ActuatorControlWidget."""
    def __init__(self, receiver, device_ip: str = DEFAULT_ACTUATOR_IP, device_port: int = DEFAULT_DEVICE_PORT):
        super().__init__()
        self.setWindowTitle(f"Actuator Control - {device_ip}:{device_port}")
        self.setGeometry(1350, 100, 1000, 500)
        self.setCentralWidget(ActuatorControlWidget(receiver, device_ip, device_port, self))
    
    def closeEvent(self, event):
        """Handle window close event"""
        w = self.centralWidget()
        if w and hasattr(w, 'save_labels'):
            w.save_labels()
        if w and hasattr(w, 'close_socket'):
            w.close_socket()
        event.accept()


# ---------------------- Combined Main Window ----------------------
class CombinedMainWindow(QtWidgets.QMainWindow):
    """Single window with sensor plot (left 66%) and actuator control (right 33%)."""
    def __init__(self, receiver, device_ip: str = DEFAULT_ACTUATOR_IP, device_port: int = DEFAULT_DEVICE_PORT, bind_address: str = '0.0.0.0'):
        super().__init__()
        self.setWindowTitle("Diablo Avionics – Sensor & Actuator")
        
        # Central widget container
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        
        # Main vertical layout
        main_layout = QtWidgets.QVBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # Top Bar
        self.top_bar_widget = TopBarWidget(self)
        self.top_bar_widget.navigation_requested.connect(self.on_navigation_requested)
        main_layout.addWidget(self.top_bar_widget)
        
        # Stacked Widget for Dashboard / Settings
        self.stack = QtWidgets.QStackedWidget()
        
        # --- Page 1: Dashboard ---
        dashboard_widget = QtWidgets.QWidget()
        dashboard_layout = QtWidgets.QHBoxLayout(dashboard_widget)
        dashboard_layout.setContentsMargins(0, 0, 0, 0)
        
        self.sensor_widget = SensorPlotWidget(receiver, bind_address, self)
        self.actuator_widget = ActuatorControlWidget(receiver, device_ip, device_port, self)
        
        splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        splitter.addWidget(self.sensor_widget)
        splitter.addWidget(self.actuator_widget)
        splitter.setStretchFactor(0, 2)  # Left 66%
        splitter.setStretchFactor(1, 1)  # Right 33%
        
        dashboard_layout.addWidget(splitter)
        self.stack.addWidget(dashboard_widget)
        
        # --- Page 2: Settings ---
        self.settings_widget = SettingsWidget(self.sensor_widget, self.actuator_widget)
        self.settings_widget.mapping_changed.connect(self.on_mapping_changed)
        self.stack.addWidget(self.settings_widget)
        
        main_layout.addWidget(self.stack, 1)
        
        # Connect sensor data for Top Bar updates
        # leveraging the existing timer update in SensorPlotWidget is tricky without a signal
        # simpler to just shadow the data
        self.gauge_map = CONFIG.config["mappings"]
        
        self.update_timer = QtCore.QTimer(self)
        self.update_timer.timeout.connect(self.update_top_bar)
        self.update_timer.start(100)
    
    def on_navigation_requested(self, view_name):
        if view_name == "settings":
            self.settings_widget.load_values() # Refresh values on enter
            self.stack.setCurrentWidget(self.settings_widget)
        else:
            self.stack.setCurrentIndex(0) # Dashboard
            
    def on_mapping_changed(self, gauge_name, pt_id):
        self.gauge_map[gauge_name] = pt_id # Update local map for internal timer use
        # CONFIG save is handled in settings_widget
        
    def update_top_bar(self):
        # Poll sensor widget for latest PSI values
        # Accessing private data sensor_psi_data directly for simplicity given the code structure
        for gauge, pt_id in self.gauge_map.items():
            val = 0.0
            if pt_id > 0 and pt_id in self.sensor_widget.sensor_psi_data:
                deque_data = self.sensor_widget.sensor_psi_data[pt_id]
                if len(deque_data) > 0:
                    val = deque_data[-1][1] # (time, psi)
            
            if gauge == "GN2":
                self.top_bar_widget.bar_gn2.set_value(val)
            elif gauge == "ETH":
                self.top_bar_widget.bar_eth.set_value(val)
            elif gauge == "LOX":
                self.top_bar_widget.bar_lox.set_value(val)
    
    def closeEvent(self, event):
        """Handle window close: save labels and close actuator socket."""
        # ConfigManager handles saving on change, so explicit save here might be redundant 
        # but safe to keep close actions for sockets
        if hasattr(self, 'actuator_widget') and self.actuator_widget:
            if hasattr(self.actuator_widget, 'close_socket'):
                self.actuator_widget.close_socket()
        event.accept()


# ---------------------- Settings Dialogs ----------------------






# ---------------------- Main Application ----------------------
def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Combined Sensor & Actuator Control GUI (single full-screen window)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Opens one full-screen window: sensor plot on the left, actuator control on the right.

Examples:
  %(prog)s                              # Use default settings
  %(prog)s -i 192.168.2.100             # Specify actuator device IP
  %(prog)s -p 5006                      # Specify receive port
        """
    )
    parser.add_argument(
        '-i', '--ip',
        type=str,
        default=CONFIG.config["network"]["actuator_ip"],
        help=f'Actuator board IP address (default from config: {CONFIG.config["network"]["actuator_ip"]})'
    )
    parser.add_argument(
        '-p', '--port',
        type=int,
        default=CONFIG.config["network"]["receive_port"],
        help=f'UDP port to receive sensor data on (default from config: {CONFIG.config["network"]["receive_port"]})'
    )
    parser.add_argument(
        '-d', '--device-port',
        type=int,
        default=CONFIG.config["network"]["actuator_port"],
        help=f'Device UDP port for actuator commands (default from config: {CONFIG.config["network"]["actuator_port"]})'
    )
    parser.add_argument(
        '-a', '--address',
        type=str,
        default='0.0.0.0',
        help='IP address to bind receiver to (default: 0.0.0.0 for all interfaces)'
    )
    
    args = parser.parse_args()
    
    app = QtWidgets.QApplication(sys.argv)
    
    # Force Fusion style and dark palette (WSL doesn't always pick up system theme)
    app.setStyle("Fusion")
    palette = QtGui.QPalette()
    palette.setColor(QtGui.QPalette.ColorGroup.All, QtGui.QPalette.ColorRole.Window, QtGui.QColor(53, 53, 53))
    palette.setColor(QtGui.QPalette.ColorGroup.All, QtGui.QPalette.ColorRole.WindowText, QtCore.Qt.GlobalColor.white)
    palette.setColor(QtGui.QPalette.ColorGroup.All, QtGui.QPalette.ColorRole.Base, QtGui.QColor(25, 25, 25))
    palette.setColor(QtGui.QPalette.ColorGroup.All, QtGui.QPalette.ColorRole.AlternateBase, QtGui.QColor(53, 53, 53))
    palette.setColor(QtGui.QPalette.ColorGroup.All, QtGui.QPalette.ColorRole.ToolTipBase, QtCore.Qt.GlobalColor.white)
    palette.setColor(QtGui.QPalette.ColorGroup.All, QtGui.QPalette.ColorRole.ToolTipText, QtCore.Qt.GlobalColor.white)
    palette.setColor(QtGui.QPalette.ColorGroup.All, QtGui.QPalette.ColorRole.Text, QtCore.Qt.GlobalColor.white)
    palette.setColor(QtGui.QPalette.ColorGroup.All, QtGui.QPalette.ColorRole.Button, QtGui.QColor(53, 53, 53))
    palette.setColor(QtGui.QPalette.ColorGroup.All, QtGui.QPalette.ColorRole.ButtonText, QtCore.Qt.GlobalColor.white)
    palette.setColor(QtGui.QPalette.ColorGroup.All, QtGui.QPalette.ColorRole.BrightText, QtCore.Qt.GlobalColor.red)
    palette.setColor(QtGui.QPalette.ColorGroup.All, QtGui.QPalette.ColorRole.Link, QtGui.QColor(42, 130, 218))
    palette.setColor(QtGui.QPalette.ColorGroup.All, QtGui.QPalette.ColorRole.Highlight, QtGui.QColor(42, 130, 218))
    palette.setColor(QtGui.QPalette.ColorGroup.All, QtGui.QPalette.ColorRole.HighlightedText, QtCore.Qt.GlobalColor.black)
    app.setPalette(palette)
    
    # Create shared UDP receiver
    receiver = UDPReceiver(port=args.port, bind_address=args.address)
    receiver.start()
    
    # Create single combined window (sensor plot left, actuator control right)
    window = CombinedMainWindow(
        receiver,
        device_ip=args.ip,
        device_port=args.device_port,
        bind_address=args.address,
    )
    # Start maximized as requested
    window.showMaximized()
    
    # Handle cleanup when app exits
    def cleanup():
        receiver.stop()
        receiver.wait(2000)
    
    app.aboutToQuit.connect(cleanup)
    
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
