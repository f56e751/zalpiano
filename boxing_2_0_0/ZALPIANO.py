import math
import numpy as np
import cv2
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32  # Assuming the message type is changed to Float32MultiArray to accommodate (x, y)
import odrive
from odrive.enums import *

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class ODriveNode(Node):
    def __init__(self):
        super().__init__('odrive_node')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
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
        self.optimal_action_subscriber = self.create_subscription(Float32MultiArray, 'optimal_action', self.optimal_action_callback, qos_profile)


        self.timer_period = 0.005
        self.timer = self.create_timer(self.timer_period, self.watchdog_feed_callback)

    def optimal_action_callback(self, msg):
        # Process using current_person_heading
        if msg.data != [float(1000.0), float(1000.0)]:
            # x, y = msg.data  # Assuming msg.data contains the target (x, y) coordinates
            # axis0_degree, axis1_degree = self.control_rotation.rotateAnyPoint(x, y, self.current_relative_heading)
            # axis0_position = self.control_rotation.degreeToPose(axis0_degree)
            # axis1_position = self.control_rotation.degreeToPose(axis1_degree)
            axis0_position, axis1_position = msg.data 
            self.motor0.controller.input_pos = axis0_position
            self.motor1.controller.input_pos = axis1_position
        else:
            self.motor0.controller.input_pos = 0
            self.motor1.controller.input_pos = 0

    def relative_heading_callback(self, msg):
        # Update current person heading
        self.current_relative_heading = msg.data
        # print(self.current_relative_heading)

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