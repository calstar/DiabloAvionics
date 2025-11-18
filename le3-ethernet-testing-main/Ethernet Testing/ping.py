import socket
import time

ESP32_IP = "192.168.2.100"  # Replace with your ESP32's IP address
ESP32_PORT = 5000
MESSAGE = "Ping"
NUM_PINGS = 1000
TIMEOUT = 1.0  # Seconds to wait for a response

client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.settimeout(TIMEOUT)

print(f"Pinging {ESP32_IP}:{ESP32_PORT} {NUM_PINGS} times...")

latencies = []  # List to store latency values

for i in range(NUM_PINGS):
    send_time = time.time()
    client_socket.sendto(MESSAGE.encode('utf-8'), (ESP32_IP, ESP32_PORT))
    print(f"Sent: {MESSAGE} (Ping {i+1}) at {send_time:.6f}")

    try:
        data, addr = client_socket.recvfrom(1024)
        recv_time = time.time()
        response = data.decode('utf-8').strip()
        latency_ms = (recv_time - send_time) * 1000
        print(f"Received packet from {addr} at {recv_time:.6f}, Latency: {latency_ms:.3f} ms")
        latencies.append(latency_ms)  # Add latency to the list
    except socket.timeout:
        print("No response received.")
        # break

client_socket.close()

# Calculate and print stats
if latencies:
    min_latency = min(latencies)
    max_latency = max(latencies)
    avg_latency = sum(latencies) / len(latencies)
    print("\n--- Latency Statistics ---")
    print(f"Minimum Latency: {min_latency:.3f} ms")
    print(f"Maximum Latency: {max_latency:.3f} ms")
    print(f"Average Latency: {avg_latency:.3f} ms")
else:
    print("\n--- Latency Statistics ---")
    print("No responses received to calculate latency.")
