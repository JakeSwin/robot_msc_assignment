#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Movement(Node):

    def __init__(self):
        super().__init__('movement')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            1)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.rotate = False
        self.turn = 0.0
        self.avg_left_diff = 0.0
        self.avg_left_prev = 0.0

    def scan_callback(self, msg):
        # Save scan left, compare with new and calculate gradiant
        # if gradiant is too high add an amount of rotation to next message
        min_front = min(msg.ranges[141:151])
        avg_left = sum(msg.ranges[257:267]) / 10
        avg_right = sum(msg.ranges[25:35]) / 10

        self.get_logger().info(f"Min Front: {min_front}, avg_left: {avg_left}, avg_right: {avg_right}")
        if self.avg_left_prev == 0.0:
            self.avg_left_prev = avg_left
        else:
            self.avg_left_diff = self.avg_left_prev - avg_left
            self.avg_left_prev = avg_left

        if min_front < 0.35:
            if not self.rotate:
                self.rotate = True
                self.turn = 0.4 if avg_left > avg_right else -0.4
        elif self.rotate and min_front > 0.9:
            self.turn = 0.0
            self.rotate = False

    def timer_callback(self):
        self.get_logger().info(f"avg_left_diff: {self.avg_left_diff}")
        if self.rotate:
            msg = Twist()
            msg.angular.z = self.turn + 0.3 if self.avg_left_diff > 0.0 else -0.3
            self.publisher.publish(msg)
        else:
            msg = Twist()
            msg.angular.z = 0.3 if self.avg_left_diff > 0.0 else -0.3
            msg.linear.x = 0.20
            self.publisher.publish(msg)


def main(args=None):
    print('Starting movement.py')
    rclpy.init(args=args)

    movement = Movement()
    rclpy.spin(movement)
    movement.destroy_node()

if __name__ == '__main__':
    main()