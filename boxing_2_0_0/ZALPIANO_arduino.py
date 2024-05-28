import serial
import time
import odrive
from odrive.enums import *
import threading


class ODriveSerialController:
    def __init__(self, serial_port, baud_rate):
        # Initialize serial port
        self.serial = serial.Serial(serial_port, baud_rate, timeout=1)
        time.sleep(2)  # Wait for Arduino reset

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
        self.watchdog_timeout = 0.2
        self.last_trigger_time = time.time()

        # Setup a persistent watchdog timer
        self.watchdog_timer = threading.Timer(self.watchdog_timeout, self.watchdog_feed)
        self.watchdog_timer.daemon = True
        self.watchdog_timer.start()

    def run(self):
        try:
            while True:
                if self.serial.in_waiting > 0:
                    line = self.serial.readline().decode('utf-8').strip()
                    scaled_values = line.split(',')
                    if len(scaled_values) == 2:
                        pwm1, pwm2 = map(float, scaled_values)
                        pos1 = self.pwm_to_position(pwm1)
                        pos2 = self.pwm_to_position(pwm2)
                        self.move_motors(pos1, pos2)
                        print(f"Motor0 position: {pos1:.2f}, Motor1 position: {pos2:.2f}")
        except KeyboardInterrupt:
            print("Exiting program.")
        finally:
            self.serial.close()

    def pwm_to_position(self, pwm):
        return (pwm + 1000) * 0.0007 - 0.7


    def move_motors(self, pos1, pos2):
        # Check if either position exceeds the maximum allowed value
        if abs(pos1) > 0.9 or abs(pos2) > 0.9:
            error_message = "Error: Motor position out of allowed range (>0.9)."
            print(error_message)
            # Raise an exception with the error message
            raise ValueError(error_message)
        else:
            # If both positions are within the acceptable range, move the motors
            self.motor0.controller.input_pos = pos1
            self.motor1.controller.input_pos = pos2
            print(f"Motors moved to positions: Motor0 = {pos1}, Motor1 = {pos2}")

    def watchdog_feed(self):
        # if time.time() - self.last_trigger_time > self.watchdog_timeout:
        #     self.motor0.watchdog_feed()
        #     self.motor1.watchdog_feed()
        #     self.last_trigger_time = time.time()
        #     # Restart timer
        #     self.watchdog_timer = threading.Timer(0.1, self.watchdog_feed)
        #     self.watchdog_timer.start()
        self.motor0.watchdog_feed()
        self.motor1.watchdog_feed()
        self.last_trigger_time = time.time()
        # Restart timer
        self.watchdog_timer = threading.Timer(self.watchdog_timeout, self.watchdog_feed)
        self.watchdog_timer.start()

def main():
    # Specify your serial port and baud rate
    controller = ODriveSerialController('/dev/ttyACM0', 115200)
    controller.run()

if __name__ == '__main__':
    main()
