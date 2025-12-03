import socket

UDP_IP = "0.0.0.0"  # Listen on all available interfaces
UDP_PORT = 5006    # Must match receiverPort in ESP32 code

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"UDP listener started on {UDP_IP}:{UDP_PORT}")

while True:
    data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
    decoded_data = data.decode('utf-8').strip()
    print(f"Received from {addr}: {decoded_data}")