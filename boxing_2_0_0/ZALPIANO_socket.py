import socket
import struct
import time
import odrive
from odrive.enums import *

class ODriveController:
    def __init__(self):
        # Find ODrive devices by their serial numbers
        self.odrv0 = odrive.find_any(serial_number="394D35333231")
        self.odrv1 = odrive.find_any(serial_number="395235613231")

        # Clear any errors
        self.odrv0.clear_errors()
        self.odrv1.clear_errors()

        # Setup motors
        self.motor0 = self.odrv0.axis0
        self.motor1 = self.odrv1.axis0

        # Set motor states
        self.motor0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.motor1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        # Set control modes
        self.motor0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.motor1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.watchdog_timeout = 0.5
        self.last_trigger_time = time.time()
        

    def move_motors(self, x, y):
        # Here you would convert x, y coordinates to motor positions if needed
        # For this example, we assume direct setting of positions
        self.motor0.controller.input_pos = x
        self.motor1.controller.input_pos = y
        print(f"Motors moved to positions: Motor0 = {x}, Motor1 = {y}")

    def watchdogFeed(self):
        self.motor0.watchdog_feed()
        self.motor1.watchdog_feed()
        self.last_trigger_time = time.time()

    def shutdown(self):
        self.odrv0.dump_errors(odrive.enums.ERROR_PRINTING_FLAGS_INCLUDE_CODES)
        self.odrv1.dump_errors(odrive.enums.ERROR_PRINTING_FLAGS_INCLUDE_CODES)

class UDPServer:
    def __init__(self, ip, port, controller):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        print(f"Listening for incoming messages on {ip}:{port}")
        self.controller = controller

    def listen_for_coordinates(self):
        # try:
        #     while True:
        #         data, addr = self.sock.recvfrom(1024)  # buffer size is 1024 bytes
        #         if len(data) == 8:  # Expecting two floats (4 bytes each)
        #             x, y = struct.unpack('ff', data)
        #             print(f"Received coordinates: ({x}, {y}) from {addr}")
        #             self.controller.move_motors(x, y)
        #         else:
        #             print("Received data of unexpected size.")
        # except KeyboardInterrupt:
        #     print("Shutting down server.")
        # finally:
        #     self.sock.close()
        #     self.controller.shutdown()

        try:
            while True:
                # Receive data from client
                data, addr = self.sock.recvfrom(1024)  # buffer size is 1024 bytes
                self.controller.watchdogFeed()
                
                
                # Check if the data is from the expected IP address
                if addr[0] == "10.42.0.13":
                    # Unpack the received bytes into coordinates (x, y)
                    if len(data) == 8:  # Expecting two floats (4 bytes each)
                        x, y = struct.unpack('ff', data)
                        self.controller.move_motors(x, y)
                        # print(f"Received coordinates: ({x}, {y}) from {addr}")
                    else:
                        print(f"Received data of unexpected size from {addr}")
                else:
                    print(f"Received message from unauthorized address: {addr}")
        except KeyboardInterrupt:
            print("Shutting down server.")
        finally:
            self.sock.close()

def main():
    odrive_controller = ODriveController()
    udp_server = UDPServer("10.42.0.1", 9999, odrive_controller)
    udp_server.listen_for_coordinates()

if __name__ == '__main__':
    main()
