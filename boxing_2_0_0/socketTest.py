# import socket
# import struct
# import time
# import random

# # IP and port configuration
# DEST_IP = "10.42.0.1"  # IP address of the receiver
# DEST_PORT = 9999       # Port number on the receiver
# LOCAL_IP = "10.42.0.13" # Local IP address

# # Create a UDP socket
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# # Bind the socket to the local IP address
# sock.bind((LOCAL_IP, 0))  # bind to all available ports by specifying 0
# print(f"Sending data to {DEST_IP}:{DEST_PORT} from {LOCAL_IP}")

# try:
#     while True:
#         # Generate random x, y coordinates
#         x = random.uniform(-100, 100)
#         y = random.uniform(-100, 100)

#         # Pack the coordinates into a binary format
#         packed_data = struct.pack('ff', x, y)

#         # Send the packed data to the destination IP and port
#         sock.sendto(packed_data, (DEST_IP, DEST_PORT))

#         # Print the coordinates sent and wait for 1 second
#         print(f"Sent coordinates: ({x:.2f}, {y:.2f})")
#         time.sleep(0.01)  # Send data every second

# except KeyboardInterrupt:
#     print("Stopping data transmission.")

# finally:
#     sock.close()

import socket
import struct
import time

class SocketProtocol:
    def __init__(self, dest_ip, dest_port, local_ip):
        self.dest_ip = dest_ip
        self.dest_port = dest_port
        self.local_ip = local_ip
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((local_ip, 0))  # Bind to all available ports by specifying 0

    def send_coordinates(self, x, y):
        try:
            # Pack the coordinates into a binary format
            packed_data = struct.pack('ff', x, y)

            # Send the packed data to the destination IP and port
            self.sock.sendto(packed_data, (self.dest_ip, self.dest_port))

            # Print the coordinates sent
            print(f"Sent coordinates: ({x:.2f}, {y:.2f})")

        except Exception as e:
            print(f"Error sending data: {e}")

    def __del__(self):
        self.sock.close()
