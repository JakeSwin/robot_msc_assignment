#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class DriveForward(Node):

    def __init__(self):
        super().__init__('forward')
        self.publisher = self.create_publisher(Twist, '/forward_vel', 1)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.35
        self.publisher.publish(msg)

class Rotate(Node):

    def __init__(self):
        super().__init__('rotate')
        self.publisher = self.create_publisher(Twist, '/rotate_vel', 1)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            1)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.rotate = False
        self.turn = 0.0

    def scan_callback(self, msg):
        min_front = min(msg.ranges[0:30] + msg.ranges[330:360])
        avg_left = sum(msg.ranges[85:95]) / 10
        avg_right = sum(msg.ranges[265:275]) / 10

        if min_front < 0.80:
            if not self.rotate:
                self.rotate = True
                self.turn = 0.2 if avg_left > avg_right else -0.2
        else:
            self.turn = 0.0
            self.rotate = False

    def timer_callback(self):
        if self.rotate:
            msg = Twist()
            msg.angular.z = self.turn
            self.publisher.publish(msg)


def main(args=None):
    print('Starting movement.py')
    rclpy.init(args=args)

    drive_forward = DriveForward()
    rotate = Rotate()

    executor = SingleThreadedExecutor()
    executor.add_node(drive_forward)
    executor.add_node(rotate)
    executor.spin()
    # while rclpy.ok():
    #     rclpy.spin_once(drive_forward)

    rotate.destroy_node()
    drive_forward.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()