import socket
import time
import threading
import matplotlib.pyplot as plt
import numpy as np

# Configuration
ESP_IP = "192.168.2.255"  # <-- CHANGE this to your ESP's IP
ESP_PORT = 5000           # <-- CHANGE if ESP listens elsewhere
BUFFER_SIZE = 800         # 100 bytes
TEST_DURATION = 10        # seconds
PACKETS_PER_SECOND = 500 # packets per second to send

# Global stats
sent_packets = 0
received_packets = 0
latencies = []

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # Enable broadcast
sock.bind(("0.0.0.0", ESP_PORT))  # Bind to all interfaces on the specified port
sock.settimeout(1)

# Store sent packet timestamps
pending_packets = {}

# Listener thread to receive echoed packets
def listen_for_echoes():
    global received_packets, latencies
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            recv_time = time.time()
            packet_id = data[0:8]  # first 8 bytes = ID
            # print(addr)
            if packet_id in pending_packets and addr[0] == '192.168.2.100':
                send_time = pending_packets.pop(packet_id)
                rtt = recv_time - send_time
                latencies.append(rtt)
                received_packets += 1
        except socket.timeout:
            continue

def main():
    global sent_packets
    # Start listener thread
    listener_thread = threading.Thread(target=listen_for_echoes, daemon=True)
    listener_thread.start()

    print("Starting UDP test...")

    start_time = time.time()
    while time.time() - start_time < TEST_DURATION:
        # Unique packet ID (use a simple counter)
        packet_id = f"{sent_packets:08d}".encode()  # 8-byte ID (zero-padded)
        payload = packet_id + b"_" * (BUFFER_SIZE - len(packet_id))  # pad to 100 bytes
        
        # Send it
        send_time = time.time()
        sock.sendto(payload, (ESP_IP, ESP_PORT))
        pending_packets[packet_id] = send_time
        sent_packets += 1

        # Control sending rate
        time.sleep(1.0 / PACKETS_PER_SECOND)

    # Wait for late echoes
    time.sleep(2)

    # Results
    duration = time.time() - start_time
    print("\n--- Test Results ---")
    print(f"Duration: {duration:.2f} seconds")
    print(f"Packets sent: {sent_packets}")
    print(f"Echoes received: {received_packets}")
    if sent_packets > 0:
        loss = (sent_packets - received_packets) / sent_packets * 100
        print(f"Packet loss: {loss:.2f}%")
    if latencies:
        print(f"Min round-trip latency: {min(latencies)*1000:.2f} ms")
        print(f"Max round-trip latency: {max(latencies)*1000:.2f} ms")
        print(f"Average round-trip latency: {sum(latencies)/len(latencies)*1000:.2f} ms")
        
        
    #print data rate in mega bits per second
    if sent_packets > 0:
        data_rate = (received_packets * BUFFER_SIZE * 8) / (duration * 1e6)  # in Mbps
        print(f"Data rate: {data_rate:.2f} Mbps")
    else:
        print("No packets sent, cannot calculate data rate.")
        
    #plot the data
    

if __name__ == "__main__":
    main()
