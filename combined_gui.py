#!/usr/bin/env python3
"""
Combined Sensor & Actuator Control GUI
Opens two separate windows:
1. Sensor Data Receiver - Real-time plotting of sensor data
2. Actuator Control - Control actuators and monitor voltage

Requirements: pip install pyqt6 pyqtgraph numpy
"""

import csv
import os
import socket
import struct
import sys
import time
from typing import Optional, Tuple, List, Dict
from collections import deque

# PT calibration CSV (relative to this file); used to show psi for connectors 1–3
PT_CALIBRATION_CSV = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "PT_Board", "Calibration", "PT Calibration Attempt 2026-02-03_test5.csv"
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
DEFAULT_DEVICE_IP = '192.168.2.100'
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

SENSOR_DATAPOINT_FORMAT = '<Bf'  # 5 bytes
SENSOR_DATAPOINT_SIZE = 5

# Plotting constants
DEFAULT_WINDOW_SECONDS = 10.0
MAX_POINTS = 10000
UPDATE_INTERVAL_MS = 50  # Update plots every 50ms
NUM_CONNECTORS = 10  # Number of connectors being cycled (1-10)
NUM_ACTUATORS = 10

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

pg.setConfigOptions(antialias=False)


# ---------------------- PT pressure from calibration ----------------------
def calculate_pressure(raw_value: float, PT_A: float, PT_B: float, PT_C: float, PT_D: float) -> float:
    """Compute pressure (psi) from raw voltage using cubic polynomial (matches C++ calculatePressure)."""
    return (PT_A * (raw_value ** 3)) + (PT_B * (raw_value ** 2)) + (PT_C * raw_value) + PT_D


def load_pt_calibration(csv_path: str) -> Dict[int, Tuple[float, float, float, float]]:
    """
    Load PT calibration coefficients from CSV.
    Returns dict: connector_id (1–3) -> (PT_A, PT_B, PT_C, PT_D).
    CSV columns per PT: Voltage, Pressure, Coefficient 0 (A), 1 (B), 2 (C), 3 (D).
    Uses the last data row as the calibration coefficients.
    """
    result = {}
    if not os.path.isfile(csv_path):
        return result
    try:
        with open(csv_path, newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        if not rows:
            return result
        # Use last row for coefficients (final calibration state)
        last = rows[-1]
        for pt_num in (1, 2, 3):
            a = float(last.get(f"PT{pt_num} Coefficient 0", 0))
            b = float(last.get(f"PT{pt_num} Coefficient 1", 0))
            c = float(last.get(f"PT{pt_num} Coefficient 2", 0))
            d = float(last.get(f"PT{pt_num} Coefficient 3", 0))
            result[pt_num] = (a, b, c, d)
        return result
    except Exception:
        return result


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
    sensor_data_received = QtCore.pyqtSignal(dict, list)  # header, chunks
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
                        self.sensor_data_received.emit(header_dict, chunks)
                        
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


# ---------------------- Sensor Plot Window ----------------------
class SensorPlotWindow(QtWidgets.QMainWindow):
    def __init__(self, receiver, bind_address: str = '0.0.0.0'):
        super().__init__()
        self.receiver = receiver
        self.bind_address = bind_address
        self.window_seconds = DEFAULT_WINDOW_SECONDS
        self.display_moving_avg_samples = 10  # Moving average for displayed values
        self.graph_moving_avg_samples = 1  # Moving average for graphed lines (1 = no smoothing)
        
        # Data storage: sensor_id -> deque of (timestamp_ms, value)
        self.sensor_data: Dict[int, deque] = {}
        self.sensor_plots: Dict[int, pg.PlotDataItem] = {}
        self.plot_enabled: Dict[int, bool] = {i: True for i in range(1, NUM_CONNECTORS + 1)}
        
        # Statistics
        self.stats_start_time = time.time()
        
        # PT calibration for connectors 1–3 (connector_id -> (PT_A, PT_B, PT_C, PT_D))
        self.pt_calibration = load_pt_calibration(PT_CALIBRATION_CSV)
        
        self.init_ui()
        
        # Connect to receiver signals
        self.receiver.sensor_data_received.connect(self.on_sensor_data)
        self.receiver.status_update.connect(self.on_status_update)
        self.receiver.packet_received.connect(self.on_packet_received)
        
        # Timer for updating plots and statistics
        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self.update_plots)
        self.update_timer.start(UPDATE_INTERVAL_MS)
        
        # Timer for updating statistics display
        self.stats_timer = QtCore.QTimer()
        self.stats_timer.timeout.connect(self.update_statistics)
        self.stats_timer.start(500)  # Update stats every 500ms
    
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle(f"Sensor Data Receiver - Port {self.receiver.port}")
        self.setGeometry(100, 100, 1200, 800)
        
        # Central widget
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        
        # Top panel with controls
        top_panel = QtWidgets.QHBoxLayout()
        
        # Connection info
        self.status_label = QtWidgets.QLabel("Starting...")
        self.status_label.setStyleSheet("font-weight: bold; padding: 5px;")
        top_panel.addWidget(self.status_label)
        
        top_panel.addStretch()
        
        # Settings button
        settings_btn = QtWidgets.QPushButton("Settings")
        settings_btn.clicked.connect(self.show_settings)
        top_panel.addWidget(settings_btn)
        
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
        small_font = QtGui.QFont()
        small_font.setPointSize(9)
        
        for i in range(1, NUM_CONNECTORS + 1):
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
        
        connector_stats_layout.addStretch()  # Push connectors to top
        connector_scroll.setWidget(connector_stats_content)
        connector_stats_group_layout.addWidget(connector_scroll)
        connector_stats_group.setLayout(connector_stats_group_layout)
        stats_main_layout.addWidget(connector_stats_group, 1)
        
        stats_widget.setFixedWidth(250)  # Fixed width for stats panel
        plot_stats_layout.addWidget(stats_widget)
        
        layout.addLayout(plot_stats_layout, 1)
        
        # Create initial plot
        self.plot_item = self.plot_widget.addPlot(title="Sensor Data Over Time (Sensors 1-10)")
        
        # Set title color and size to white for visibility on black background
        self.plot_item.setTitle("Sensor Data Over Time (Sensors 1-10)", color='w', size='14pt')
        
        # Set axis labels to white
        self.plot_item.setLabel('left', 'Voltage (V)', color='w')
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
            self.add_sensor_plot(connector_id)
    
    def _on_plot_toggle(self, connector_id: int, state):
        self.plot_enabled[connector_id] = bool(state)
    
    def on_graph_moving_avg_changed(self, value):
        """Handle graph moving average window size change"""
        self.graph_moving_avg_samples = value
    
    def on_display_moving_avg_changed(self, value):
        """Handle display moving average window size change"""
        self.display_moving_avg_samples = value
    
    def on_status_update(self, message: str):
        """Handle status updates from receiver thread"""
        self.status_label.setText(message)
    
    def on_packet_received(self, packet_size: int, packet_type: int):
        """Handle packet received notification"""
        pass  # Statistics are updated separately
    
    def on_sensor_data(self, header: dict, chunks: List[dict]):
        """Handle received sensor data"""
        current_time = time.time()
        
        for chunk in chunks:
            chunk_timestamp_ms = chunk['timestamp']
            # Convert packet timestamp to relative time in seconds
            relative_time = (current_time - self.stats_start_time)
            
            for dp in chunk['datapoints']:
                sensor_id = dp['sensor_id']
                value = dp['data']
                
                # Initialize sensor data storage if needed (for sensors outside 1-10 range)
                if sensor_id not in self.sensor_data:
                    self.sensor_data[sensor_id] = deque(maxlen=MAX_POINTS)
                    self.add_sensor_plot(sensor_id)
                
                # Add data point (use relative time from start)
                self.sensor_data[sensor_id].append((relative_time, value))
    
    def add_sensor_plot(self, sensor_id: int):
        """Add a new sensor plot"""
        if sensor_id not in self.plot_enabled:
            self.plot_enabled[sensor_id] = True
        color_idx = sensor_id % len(SENSOR_COLORS)
        color = SENSOR_COLORS[color_idx]
        
        pen = pg.mkPen(color=color, width=2)
        plot = self.plot_item.plot([], [], pen=pen, name=f"Sensor {sensor_id}")
        
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
        """Update all sensor plots"""
        if not self.sensor_data:
            return
        
        current_time = time.time() - self.stats_start_time
        time_window = self.window_seconds
        
        for sensor_id, data_deque in self.sensor_data.items():
            if sensor_id not in self.sensor_plots:
                continue
            enabled = self.plot_enabled.get(sensor_id, True)
            self.sensor_plots[sensor_id].setVisible(enabled)
            if not enabled:
                continue
            if len(data_deque) == 0:
                continue
            
            # Extract time and value arrays
            times = []
            values = []
            
            for t, v in data_deque:
                # Only show data within the time window
                if current_time - t <= time_window:
                    times.append(t)
                    values.append(v)
            
            if len(times) > 0:
                # Convert to numpy arrays
                times_array = np.array(times)
                values_array = np.array(values)
                
                # Apply moving average smoothing to graph if window > 1
                if self.graph_moving_avg_samples > 1 and len(values_array) >= self.graph_moving_avg_samples:
                    # Use convolution for efficient moving average
                    kernel = np.ones(self.graph_moving_avg_samples) / self.graph_moving_avg_samples
                    smoothed_values = np.convolve(values_array, kernel, mode='valid')
                    # Adjust times array to match smoothed data length
                    smoothed_times = times_array[self.graph_moving_avg_samples - 1:]
                    
                    # Update plot with smoothed data
                    self.sensor_plots[sensor_id].setData(smoothed_times, smoothed_values)
                else:
                    # Update plot with raw data
                    self.sensor_plots[sensor_id].setData(times_array, values_array)
        
        # Update x-axis range
        if current_time > time_window:
            self.plot_item.setXRange(current_time - time_window, current_time, padding=0)
        else:
            self.plot_item.setXRange(0, time_window, padding=0)
    
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
                    # Show psi for PTs 1–3 when calibration is loaded (larger font)
                    if connector_id in self.pt_calibration:
                        a, b, c, d = self.pt_calibration[connector_id]
                        psi = calculate_pressure(moving_avg, a, b, c, d)
                        text += f"<br/><span style='font-size: 16pt; font-weight: bold'>{psi:.2f} psi</span>"
                    self.connector_labels[connector_id].setText(text)
                else:
                    self.connector_labels[connector_id].setText(f"C{connector_id}: --- V")
            else:
                self.connector_labels[connector_id].setText(f"C{connector_id}: --- V")
    
    def show_settings(self):
        """Show settings dialog"""
        dialog = SensorSettingsDialog(self)
        if dialog.exec():
            self.window_seconds = dialog.window_seconds


# ---------------------- Actuator Control Window ----------------------
class ActuatorControlWindow(QtWidgets.QMainWindow):
    def __init__(self, receiver, device_ip: str = DEFAULT_DEVICE_IP, device_port: int = DEFAULT_DEVICE_PORT):
        super().__init__()
        self.receiver = receiver
        self.device_ip = device_ip
        self.device_port = device_port
        
        # Actuator state tracking (1-indexed: 1-10)
        # 0 = OFF, 1 = ON
        self.actuator_states = [0] * NUM_ACTUATORS
        
        # Voltage readings (0-indexed: 0-9, maps to actuator 1-10)
        # Store as voltage in Volts
        self.voltage_readings = [0.0] * NUM_ACTUATORS
        
        # UDP socket for sending commands
        self.command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.init_ui()
        
        # Connect to receiver signals
        self.receiver.sensor_data_received.connect(self.on_sensor_data)
        self.receiver.status_update.connect(self.on_status_update)
        
        # Timer for updating current display
        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self.update_current_display)
        self.update_timer.start(100)  # Update every 100ms
    
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle(f"Actuator Control - {self.device_ip}:{self.device_port}")
        self.setGeometry(1350, 100, 1000, 500)
        
        # Central widget
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        
        # Top panel with status and settings
        top_panel = QtWidgets.QHBoxLayout()
        
        self.status_label = QtWidgets.QLabel("Ready")
        self.status_label.setStyleSheet("font-weight: bold; padding: 5px;")
        top_panel.addWidget(self.status_label)
        
        top_panel.addStretch()
        
        # Settings button
        settings_btn = QtWidgets.QPushButton("Settings")
        settings_btn.clicked.connect(self.show_settings)
        top_panel.addWidget(settings_btn)
        
        layout.addLayout(top_panel)
        
        # Main content area with actuators in a grid
        grid_container = QtWidgets.QWidget()
        grid_layout = QtWidgets.QGridLayout(grid_container)
        grid_layout.setSpacing(10)
        
        # Create actuator controls in a 5x2 grid
        self.actuator_widgets = []
        for i in range(NUM_ACTUATORS):
            actuator_id = i + 1  # 1-indexed
            
            # Calculate grid position: 5 columns, 2 rows
            row = i // 5  # 0 or 1
            col = i % 5   # 0-4
            
            # Create widget for each actuator
            actuator_frame = QtWidgets.QFrame()
            actuator_frame.setFrameShape(QtWidgets.QFrame.Shape.Box)
            actuator_frame.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
            actuator_frame.setStyleSheet("padding: 10px; margin: 5px;")
            
            actuator_layout = QtWidgets.QVBoxLayout(actuator_frame)
            
            # Actuator ID label
            id_label = QtWidgets.QLabel(f"Actuator {actuator_id}")
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
                'voltage_label': voltage_label
            })
        
        layout.addWidget(grid_container, 1)
        
        # Initialize all actuators to OFF state (highlight OFF buttons)
        for i in range(NUM_ACTUATORS):
            self.update_button_highlight(i, 0)
    
    def on_status_update(self, message: str):
        """Handle status updates from receiver thread"""
        # Only update if it's relevant to actuator control
        pass
    
    def on_sensor_data(self, header: dict, chunks: List[dict]):
        """Handle received sensor data (voltage readings)"""
        # Process the latest chunk
        if chunks:
            latest_chunk = chunks[-1]
            for dp in latest_chunk['datapoints']:
                sensor_id = dp['sensor_id']  # 1-indexed (1-10)
                voltage = dp['data']  # Voltage in Volts (float)
                
                # Convert to 0-indexed for array
                array_idx = sensor_id - 1
                if 0 <= array_idx < NUM_ACTUATORS:
                    self.voltage_readings[array_idx] = voltage
    
    def update_button_highlight(self, array_idx: int, actuator_state: int):
        """
        Update button highlighting based on actuator state.
        array_idx: 0-9 (0-indexed)
        actuator_state: 0 = OFF, 1 = ON
        """
        widget = self.actuator_widgets[array_idx]
        
        # Inactive button style (matches background)
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
        
        # Active button style (white background, black text)
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
        
        # Highlight ON button if state is ON, otherwise highlight OFF button
        if actuator_state == 1:
            widget['on_btn'].setStyleSheet(active_style)
            widget['off_btn'].setStyleSheet(inactive_style)
        else:
            widget['on_btn'].setStyleSheet(inactive_style)
            widget['off_btn'].setStyleSheet(active_style)
    
    def set_actuator_state(self, actuator_id: int, actuator_state: int):
        """
        Set actuator state and send command packet.
        actuator_id: 1-10 (1-indexed)
        actuator_state: 0 = OFF, 1 = ON
        """
        # Convert to 0-indexed for array
        array_idx = actuator_id - 1
        
        # Update local state
        self.actuator_states[array_idx] = actuator_state
        
        # Update UI - highlight the active button
        self.update_button_highlight(array_idx, actuator_state)
        
        # Send command packet
        self.send_actuator_command(actuator_id, actuator_state)
    
    def send_actuator_command(self, actuator_id: int, actuator_state: int):
        """
        Send an actuator command packet to the device.
        actuator_id: 1-10 (1-indexed)
        actuator_state: 0 = OFF, non-zero = ON
        """
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
            if err == 65:  # EHOSTUNREACH on macOS
                msg = f"No route to host — check device IP ({self.device_ip})"
            elif err == 64:  # ENETUNREACH
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
    
    def show_settings(self):
        """Show settings dialog"""
        dialog = ActuatorSettingsDialog(self)
        if dialog.exec():
            self.device_ip = dialog.device_ip
            self.device_port = dialog.device_port
            self.setWindowTitle(f"Actuator Control - {self.device_ip}:{self.device_port}")
    
    def closeEvent(self, event):
        """Handle window close event"""
        if self.command_sock:
            try:
                self.command_sock.close()
            except:
                pass
        event.accept()


# ---------------------- Settings Dialogs ----------------------
class SensorSettingsDialog(QtWidgets.QDialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.setWindowTitle("Sensor Display Settings")
        self.window_seconds = parent.window_seconds
        
        layout = QtWidgets.QVBoxLayout(self)
        
        # Time window
        layout.addWidget(QtWidgets.QLabel("Time Window (seconds)"))
        row = QtWidgets.QHBoxLayout()
        self.slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.slider.setMinimum(1)
        self.slider.setMaximum(60)
        self.slider.setValue(int(self.window_seconds))
        self.slider.valueChanged.connect(self._on_change)
        self.lbl = QtWidgets.QLabel(f"{self.window_seconds:.1f}s")
        row.addWidget(self.slider, 1)
        row.addWidget(self.lbl)
        layout.addLayout(row)
        
        # Close button
        btn = QtWidgets.QPushButton("Close")
        btn.clicked.connect(self.accept)
        layout.addWidget(btn)
    
    def _on_change(self, val):
        self.window_seconds = float(val)
        self.lbl.setText(f"{float(val):.1f}s")


class ActuatorSettingsDialog(QtWidgets.QDialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.setWindowTitle("Actuator Control Settings")
        self.device_ip = parent.device_ip
        self.device_port = parent.device_port
        
        layout = QtWidgets.QVBoxLayout(self)
        
        # Device IP
        layout.addWidget(QtWidgets.QLabel("Device IP Address:"))
        self.ip_edit = QtWidgets.QLineEdit(self.device_ip)
        layout.addWidget(self.ip_edit)
        
        # Device port
        layout.addWidget(QtWidgets.QLabel("Device Port (for commands):"))
        self.device_port_edit = QtWidgets.QSpinBox()
        self.device_port_edit.setRange(1, 65535)
        self.device_port_edit.setValue(self.device_port)
        layout.addWidget(self.device_port_edit)
        
        # Buttons
        btn_layout = QtWidgets.QHBoxLayout()
        ok_btn = QtWidgets.QPushButton("OK")
        ok_btn.clicked.connect(self.accept)
        cancel_btn = QtWidgets.QPushButton("Cancel")
        cancel_btn.clicked.connect(self.reject)
        btn_layout.addWidget(ok_btn)
        btn_layout.addWidget(cancel_btn)
        layout.addLayout(btn_layout)
    
    def accept(self):
        """Validate and accept settings"""
        self.device_ip = self.ip_edit.text()
        self.device_port = self.device_port_edit.value()
        super().accept()


# ---------------------- Main Application ----------------------
def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Combined Sensor & Actuator Control GUI',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Opens two separate windows:
1. Sensor Data Receiver - Real-time plotting of sensor data
2. Actuator Control - Control actuators and monitor voltage

Examples:
  %(prog)s                              # Use default settings
  %(prog)s -i 192.168.2.100             # Specify device IP
  %(prog)s -p 5006                      # Specify receive port
        """
    )
    parser.add_argument(
        '-i', '--ip',
        type=str,
        default=DEFAULT_DEVICE_IP,
        help=f'Device IP address (default: {DEFAULT_DEVICE_IP})'
    )
    parser.add_argument(
        '-p', '--port',
        type=int,
        default=DEFAULT_RECEIVE_PORT,
        help=f'UDP port to receive sensor data on (default: {DEFAULT_RECEIVE_PORT})'
    )
    parser.add_argument(
        '-d', '--device-port',
        type=int,
        default=DEFAULT_DEVICE_PORT,
        help=f'Device UDP port for actuator commands (default: {DEFAULT_DEVICE_PORT})'
    )
    parser.add_argument(
        '-a', '--address',
        type=str,
        default='0.0.0.0',
        help='IP address to bind receiver to (default: 0.0.0.0 for all interfaces)'
    )
    
    args = parser.parse_args()
    
    app = QtWidgets.QApplication(sys.argv)
    
    # Create shared UDP receiver
    receiver = UDPReceiver(port=args.port, bind_address=args.address)
    receiver.start()
    
    # Create both windows
    sensor_window = SensorPlotWindow(receiver, bind_address=args.address)
    actuator_window = ActuatorControlWindow(receiver, device_ip=args.ip, device_port=args.device_port)
    
    # Show both windows
    sensor_window.show()
    actuator_window.show()
    
    # Handle cleanup when app exits
    def cleanup():
        receiver.stop()
        receiver.wait(2000)
    
    app.aboutToQuit.connect(cleanup)
    
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
