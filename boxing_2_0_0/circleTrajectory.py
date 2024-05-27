import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import math

class CircleTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('circle_trajectory_publisher')
        self.publisher_ = self.create_publisher(Point, 'circle_trajectory', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_circle_trajectory)
        self.radius = 100.0  # 원의 반지름
        self.angle = 0.0   # 시작 각도

    def publish_circle_trajectory(self):
        x = self.radius * math.cos(self.angle)
        y = self.radius * math.sin(self.angle)
        self.angle += 0.1  # 각도 증가

        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = 0.0  # z축은 0으로 설정
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    circle_trajectory_publisher = CircleTrajectoryPublisher()
    rclpy.spin(circle_trajectory_publisher)

    circle_trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
