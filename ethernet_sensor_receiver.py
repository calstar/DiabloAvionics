#!/usr/bin/env python3
"""
Ethernet Sensor Data Receiver
Receives UDP packets from ethernet link and decodes sensor data packets using DAQv2-Comms protocol.
Only processes SENSOR_DATA packet type (PacketType::SENSOR_DATA = 3).
"""

import socket
import struct
import sys
from typing import Optional, Tuple, List

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

# Default UDP port (from Ethernet test code)
DEFAULT_PORT = 5006

# Struct format strings (little-endian, matching C++ packed structs)
# PacketHeader: packet_type (uint8_t), version (uint8_t), timestamp (uint32_t)
PACKET_HEADER_FORMAT = '<BBI'  # 6 bytes total
PACKET_HEADER_SIZE = 6

# SensorDataPacket: num_chunks (uint8_t), num_sensors (uint8_t)
SENSOR_DATA_PACKET_FORMAT = '<BB'  # 2 bytes
SENSOR_DATA_PACKET_SIZE = 2

# SensorDataChunk: timestamp (uint32_t)
SENSOR_DATA_CHUNK_FORMAT = '<I'  # 4 bytes
SENSOR_DATA_CHUNK_SIZE = 4

# SensorDatapoint: sensor_id (uint8_t), data (uint32_t)
SENSOR_DATAPOINT_FORMAT = '<BI'  # 5 bytes
SENSOR_DATAPOINT_SIZE = 5


def parse_packet_header(data: bytes) -> Optional[Tuple[int, int, int]]:
    """
    Parse the packet header.
    Returns: (packet_type, version, timestamp) or None if parsing fails
    """
    if len(data) < PACKET_HEADER_SIZE:
        return None
    
    try:
        packet_type, version, timestamp = struct.unpack(PACKET_HEADER_FORMAT, data[:PACKET_HEADER_SIZE])
        return (packet_type, version, timestamp)
    except struct.error:
        return None


def parse_sensor_data_packet(data: bytes) -> Optional[Tuple[dict, List[dict]]]:
    """
    Parse a sensor data packet.
    Returns: (header_dict, chunks_list) or None if parsing fails
    """
    if len(data) < PACKET_HEADER_SIZE + SENSOR_DATA_PACKET_SIZE:
        return None
    
    # Parse header
    header = parse_packet_header(data)
    if header is None or header[0] != PacketType.SENSOR_DATA:
        return None
    
    packet_type, version, timestamp = header
    
    # Parse sensor data packet body
    offset = PACKET_HEADER_SIZE
    try:
        num_chunks, num_sensors = struct.unpack(
            SENSOR_DATA_PACKET_FORMAT,
            data[offset:offset + SENSOR_DATA_PACKET_SIZE]
        )
    except struct.error:
        return None
    
    offset += SENSOR_DATA_PACKET_SIZE
    
    # Calculate expected packet size
    per_chunk_size = SENSOR_DATA_CHUNK_SIZE + (num_sensors * SENSOR_DATAPOINT_SIZE)
    expected_size = PACKET_HEADER_SIZE + SENSOR_DATA_PACKET_SIZE + (num_chunks * per_chunk_size)
    
    if len(data) < expected_size:
        print(f"Warning: Packet size mismatch. Expected {expected_size} bytes, got {len(data)}")
        return None
    
    # Parse chunks
    chunks = []
    for chunk_idx in range(num_chunks):
        # Parse chunk timestamp
        try:
            chunk_timestamp, = struct.unpack(
                SENSOR_DATA_CHUNK_FORMAT,
                data[offset:offset + SENSOR_DATA_CHUNK_SIZE]
            )
        except struct.error:
            print(f"Error parsing chunk {chunk_idx} timestamp")
            return None
        
        offset += SENSOR_DATA_CHUNK_SIZE
        
        # Parse datapoints for this chunk
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
                print(f"Error parsing datapoint {sensor_idx} in chunk {chunk_idx}")
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


def print_sensor_data(header: dict, chunks: List[dict]):
    """Print decoded sensor data in a readable format."""
    print("\n" + "=" * 60)
    print("SENSOR DATA PACKET")
    print("=" * 60)
    print(f"Packet Type: SENSOR_DATA ({header['packet_type']})")
    print(f"Version: {header['version']}")
    print(f"Header Timestamp: {header['timestamp']} ms")
    print(f"Number of Chunks: {len(chunks)}")
    
    for chunk_idx, chunk in enumerate(chunks):
        print(f"\n--- Chunk {chunk_idx + 1} ---")
        print(f"Chunk Timestamp: {chunk['timestamp']} ms")
        print(f"Number of Sensors: {len(chunk['datapoints'])}")
        
        for dp in chunk['datapoints']:
            print(f"  Sensor ID {dp['sensor_id']}: {dp['data']}")
    
    print("=" * 60 + "\n")


def receive_and_decode_packets(port: int = DEFAULT_PORT, bind_address: str = '0.0.0.0'):
    """
    Main function to receive UDP packets and decode sensor data packets.
    
    Args:
        port: UDP port to listen on (default: 5006)
        bind_address: IP address to bind to (default: '0.0.0.0' for all interfaces)
    """
    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        sock.bind((bind_address, port))
        print(f"Listening for UDP packets on {bind_address}:{port}")
        print("Waiting for SENSOR_DATA packets...")
        print("Press Ctrl+C to stop\n")
        
        packet_count = 0
        sensor_packet_count = 0
        
        while True:
            try:
                # Receive packet
                data, addr = sock.recvfrom(MAX_PACKET_SIZE)
                packet_count += 1
                
                # Parse header to check packet type
                header = parse_packet_header(data)
                if header is None:
                    print(f"[Packet {packet_count}] Invalid packet header from {addr[0]}:{addr[1]}")
                    continue
                
                packet_type, version, timestamp = header
                
                # Only process SENSOR_DATA packets
                if packet_type == PacketType.SENSOR_DATA:
                    sensor_packet_count += 1
                    result = parse_sensor_data_packet(data)
                    if result:
                        header_dict, chunks = result
                        print(f"[Sensor Packet {sensor_packet_count} from {addr[0]}:{addr[1]}]")
                        print_sensor_data(header_dict, chunks)
                    else:
                        print(f"[Packet {packet_count}] Failed to parse sensor data packet from {addr[0]}:{addr[1]}")
                else:
                    # Silently ignore other packet types
                    packet_type_names = {
                        PacketType.BOARD_HEARTBEAT: "BOARD_HEARTBEAT",
                        PacketType.SERVER_HEARTBEAT: "SERVER_HEARTBEAT",
                        PacketType.ACTUATOR_COMMAND: "ACTUATOR_COMMAND",
                        PacketType.SENSOR_CONFIG: "SENSOR_CONFIG",
                        PacketType.ACTUATOR_CONFIG: "ACTUATOR_CONFIG",
                        PacketType.ABORT: "ABORT",
                        PacketType.ABORT_DONE: "ABORT_DONE",
                        PacketType.CLEAR_ABORT: "CLEAR_ABORT"
                    }
                    packet_name = packet_type_names.get(packet_type, f"UNKNOWN({packet_type})")
                    print(f"[Packet {packet_count}] Ignoring {packet_name} packet from {addr[0]}:{addr[1]}")
                    
            except KeyboardInterrupt:
                print("\n\nStopping receiver...")
                break
            except Exception as e:
                print(f"Error receiving packet: {e}")
                continue
        
        print(f"\nTotal packets received: {packet_count}")
        print(f"Total sensor data packets processed: {sensor_packet_count}")
        
    except OSError as e:
        print(f"Error binding to {bind_address}:{port}: {e}")
        sys.exit(1)
    finally:
        sock.close()


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Receive and decode DAQv2-Comms sensor data packets from UDP',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                    # Listen on default port 5006
  %(prog)s -p 5007            # Listen on port 5007
  %(prog)s -a 192.168.2.100   # Bind to specific IP address
        """
    )
    parser.add_argument(
        '-p', '--port',
        type=int,
        default=DEFAULT_PORT,
        help=f'UDP port to listen on (default: {DEFAULT_PORT})'
    )
    parser.add_argument(
        '-a', '--address',
        type=str,
        default='0.0.0.0',
        help='IP address to bind to (default: 0.0.0.0 for all interfaces)'
    )
    
    args = parser.parse_args()
    
    receive_and_decode_packets(port=args.port, bind_address=args.address)


if __name__ == '__main__':
    main()


