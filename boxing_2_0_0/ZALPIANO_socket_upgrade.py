import socket
import struct
import time
import odrive
from odrive.enums import *
import threading

class ODriveUDPController:
    def __init__(self, ip, port):
        # Initialize UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        print(f"Listening for incoming messages on {ip}:{port}")
        
        # Find and setup ODrive devices
        self.odrv0 = odrive.find_any(serial_number="394D35333231")
        self.odrv1 = odrive.find_any(serial_number="395235613231")
        self.odrv0.clear_errors()
        self.odrv1.clear_errors()
        self.motor0 = self.odrv0.axis0
        self.motor1 = self.odrv1.axis0
        self.motor0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.motor1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.motor0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.motor1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        
        # Setup watchdog parameters
        self.watchdog_timeout = 0.5
        self.last_trigger_time = time.time()

        # Setup a persistent watchdog timer
        self.watchdog_timer = threading.Timer(0.1, self.watchdog_feed)
        self.watchdog_timer.daemon = True
        self.watchdog_timer.start()

    def listen_and_move_motors(self):
        try:
            while True:
                data, addr = self.sock.recvfrom(1024)  # Buffer size is 1024 bytes
                
                if addr[0] == "10.42.0.13" and len(data) == 8:
                    x, y = struct.unpack('ff', data)
                    self.move_motors(x, y)
                elif len(data) != 8:
                    print(f"Received data of unexpected size from {addr}")
                else:
                    print(f"Received message from unauthorized address: {addr}")
        except KeyboardInterrupt:
            print("Shutting down server.")
        finally:
            self.sock.close()
            self.shutdown()

    def move_motors(self, x, y):
        self.motor0.controller.input_pos = x
        self.motor1.controller.input_pos = y
        print(f"Motors moved to positions: Motor0 = {x}, Motor1 = {y}")

    def watchdog_feed(self):
        if time.time() - self.last_trigger_time > self.watchdog_timeout:
            self.motor0.watchdog_feed()
            self.motor1.watchdog_feed()
            self.last_trigger_time = time.time()
            # Restart timer
            self.watchdog_timer = threading.Timer(0.1, self.watchdog_feed)
            self.watchdog_timer.start()

    def shutdown(self):
        print("Controller shutdown. Errors dumped if any.")
        self.watchdog_timer.cancel()
        self.odrv0.dump_errors(odrive.enums.ERROR_PRINTING_FLAGS_INCLUDE_CODES)
        self.odrv1.dump_errors(odrive.enums.ERROR_PRINTING_FLAGS_INCLUDE_CODES)

def main():
    controller = ODriveUDPController("10.42.0.1", 9999)
    controller.listen_and_move_motors()

if __name__ == '__main__':
    main()
