import socket
import time

UDP_IP = "0.0.0.0"  # Listen on all interfaces
UDP_PORT = 5006
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP data on {UDP_IP}:{UDP_PORT}")

start_time = time.time()
total_bytes_received = 0

try:
    while True:
        data, addr = sock.recvfrom(65535)  # Max UDP packet size
        total_bytes_received += len(data)

        elapsed_time = time.time() - start_time
        if elapsed_time >= 5:  # Calculate and print every 5 seconds
            data_rate_bps = total_bytes_received / elapsed_time
            data_rate_mbps = (data_rate_bps * 8) / 1000000.0
            print(f"Time: {elapsed_time:.2f} s, Received: {total_bytes_received} bytes, Data Rate: {data_rate_mbps:.2f} Mbps")
            start_time = time.time()
            total_bytes_received = 0

except KeyboardInterrupt:
    print("Stopped by user.")
    sock.close()