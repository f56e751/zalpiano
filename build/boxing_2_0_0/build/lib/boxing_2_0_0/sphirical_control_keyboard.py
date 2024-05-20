import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
import odrive
from odrive.enums import *
import threading
import sys
import select
import numpy as np

##CHANGES##
#theta -> relative_heading
#xyAngle -> theta
#tiltAngle -> phi
############

class ControlRotation():
    def __init__(self, maximumPhi, maximumLength):
        # maximumPhi: rad
        # maximumLength: use this value to transform Length to xyPlaneRadius = 1
        #                abs(x), abs(y) <= maximumLength
        self.axis0_sign = 1
        self.axis1_sign = -1
        self.gearRatio = 10
        self.xyPlaneRadius = 1
        self.maximumLength = maximumLength
        self.fixed_radius = self.xyPlaneRadius / np.sin(maximumPhi) #length of rod

    def calculate_spherical_coordinates(self, x, y):
        # transform x, y to unit Length
        distance_from_origin = math.sqrt(x**2 + y**2)
        if distance_from_origin > self.maximumLength:
            raise ValueError("The distance from the origin to (x, y) must be smaller than or equal to self.maximumLength")
        
        tf_x = x / self.maximumLength
        tf_y = y / self.maximumLength
        r_xy = math.sqrt(tf_x**2 + tf_y**2)
        theta = math.atan2(y, x)
        phi = math.asin(r_xy / self.fixed_radius)  # r_xy should be less than or equal to fixed_radius
        return theta, phi

    def rotateAnyPoint(self, x, y, relative_heading):
        theta, phi = self.calculate_spherical_coordinates(x, y)
        tan = math.tan(phi)
        sin = math.sin(theta + relative_heading - np.pi / 2)
        cos = math.cos(theta + relative_heading - np.pi / 2)
        axis0_degree = self.axis0_sign * math.degrees(math.atan(tan * cos))
        axis1_degree = self.axis1_sign * math.degrees(math.atan(tan * sin))
        return axis0_degree, axis1_degree

    def degreeToPose(self, degree):
        return degree / 360 * self.gearRatio

class ODriveNode(Node):
    def __init__(self):
        super().__init__('odrive_node')
        
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

        self.watchdog_timeout = 0.5
        self.last_trigger_time = time.time()

        self.control_rotation = ControlRotation(maximumPhi = np.deg2rad(30), maximumLength = 10)  # Example fixed radius value
        self.current_person_heading = 0  # Initialize person heading

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.watchdog_feed_callback)

        # Start a separate thread for keyboard input
        self.input_thread = threading.Thread(target=self.keyboard_input)
        self.input_thread.daemon = True
        self.input_thread.start()

    def optimal_action_callback(self, msg):
        # Process using current_person_heading
        if msg.data != [float(1000.0), float(1000.0)]:
            x, y = msg.data  # Assuming msg.data contains the target (x, y) coordinates
            axis0_degree, axis1_degree = self.control_rotation.rotateAnyPoint(x, y, self.current_person_heading)
            axis0_position = self.control_rotation.degreeToPose(axis0_degree)
            axis1_position = self.control_rotation.degreeToPose(axis1_degree)
            
            self.motor0.controller.input_pos = axis0_position
            self.motor1.controller.input_pos = axis1_position
        else:
            self.motor0.controller.input_pos = 0
            self.motor1.controller.input_pos = 0

    def person_heading_callback(self, msg):
        # Update current person heading
        self.current_person_heading = msg.data

    def watchdog_feed_callback(self):
        # Feed the watchdog to ensure motor safety
        self.motor0.watchdog_feed()
        self.motor1.watchdog_feed()
        self.last_trigger_time = time.time()

    def keyboard_input(self):
        print("Enter coordinates (x y) or 'q' to quit:")
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                input_data = sys.stdin.readline().strip()
                if input_data.lower() == 'q':
                    print("Exiting...")
                    rclpy.shutdown()
                    break
                try:
                    x, y = map(float, input_data.split())
                    self.process_keyboard_input(x, y)
                except ValueError:
                    print("Invalid input. Please enter coordinates in the format: x y")

    def process_keyboard_input(self, x, y):
        axis0_degree, axis1_degree = self.control_rotation.rotateAnyPoint(x, y, self.current_person_heading)
        axis0_position = self.control_rotation.degreeToPose(axis0_degree)
        axis1_position = self.control_rotation.degreeToPose(axis1_degree)
        
        self.motor0.controller.input_pos = axis0_position
        self.motor1.controller.input_pos = axis1_position

def main(args=None):
    rclpy.init(args=args)
    odrive_node = ODriveNode()
    rclpy.spin(odrive_node)
    odrive_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
