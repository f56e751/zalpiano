import math
import numpy as np
import cv2
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import odrive
from odrive.enums import *

class ControlRotation():
    def __init__(self):
        # self.aSign = 1
        # self.bSign = -1

        self.axis0_sign = 1
        self.axis1_sign = -1

        self.gearRatio = 10
        self.tiltAngle = np.deg2rad(30)
        self.xyAngles = [0, -np.pi / 4, -np.pi / 2, -3 * np.pi / 4, -np.pi]

    def rotateAnyPoint(self, xyAngle, theta):
        # -pi < xyAngle < 0 
        # xyAngle = 0     -> move Left  (baseline: robot to human)
        # xyAngle = -pi/2 -> move Back
        # xyAngle = -pi   -> move Right
        tan = math.tan(self.tiltAngle)
        sin = math.sin(xyAngle + theta)
        cos = math.cos(xyAngle + theta)
        
        # a = self.aSign * math.degrees(math.atan(tan * sin))
        # b = self.bSign * math.degrees(math.atan(tan * cos))

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

        # Create separate subscriptions for each topic
        self.optimal_action_subscriber = self.create_subscription(Float64, 'optimal_action', self.optimal_action_callback, 10)
        self.relative_heading_subscriber = self.create_subscription(Float64, 'relative_heading', self.relative_heading_callback, 10)

        self.control_rotation = ControlRotation()
        self.current_theta = 0  # Initialize theta

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.watchdog_feed_callback)

    def optimal_action_callback(self, msg):
        # Process using current_theta
        if msg.data != float(1000.0):
            axis0_degree, axis1_degree = self.control_rotation.rotateAnyPoint(msg.data, self.current_theta)
            axis0_position = self.control_rotation.degreeToPose(axis0_degree)
            axis1_position = self.control_rotation.degreeToPose(axis1_degree)
            
            self.motor0.controller.input_pos = axis0_position
            self.motor1.controller.input_pos = axis1_position
        else:
            self.motor0.controller.input_pos = 0
            self.motor1.controller.input_pos = 0

    def relative_heading_callback(self, msg):
        # Update current theta
        self.current_theta = msg.data

    def watchdog_feed_callback(self):
        # Feed the watchdog to ensure motor safety
        self.motor0.watchdog_feed()
        self.motor1.watchdog_feed()
        self.last_trigger_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    odrive_node = ODriveNode()
    rclpy.spin(odrive_node)
    odrive_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
