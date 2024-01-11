#!/usr/bin/python3
# Resources Used: 
#   - https://github.com/ros-planning/navigation2/blob/main/nav2_simple_commander/nav2_simple_commander/example_waypoint_follower.py
#   - https://navigation.ros.org/commander_api/index.html

import rclpy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

def main():
    rclpy.init()
    
    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    goal_pose_1 = PoseStamped()
    goal_pose_1.header.frame_id = 'map'
    goal_pose_1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_1.pose.position.x = 1.15
    goal_pose_1.pose.position.y = -0.05
    goal_pose_1.pose.orientation.z = 0.0
    goal_pose_1.pose.orientation.w = 1.0

    goal_pose_2 = PoseStamped()
    goal_pose_2.header.frame_id = 'map'
    goal_pose_2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_2.pose.position.x = 0.8
    goal_pose_2.pose.position.y = -0.9
    goal_pose_2.pose.orientation.z = 1.0
    goal_pose_2.pose.orientation.w = 0.0

    goal_pose_3 = PoseStamped()
    goal_pose_3.header.frame_id = 'map'
    goal_pose_3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_3.pose.position.x = 0.0
    goal_pose_3.pose.position.y = -0.9
    goal_pose_3.pose.orientation.z = 1.0
    goal_pose_3.pose.orientation.w = 0.0

    goal_pose_4 = PoseStamped()
    goal_pose_4.header.frame_id = 'map'
    goal_pose_4.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_4.pose.position.x = -1.15
    goal_pose_4.pose.position.y = -0.9
    goal_pose_4.pose.orientation.z = 1.0
    goal_pose_4.pose.orientation.w = 0.0

    goal_pose_5 = PoseStamped()
    goal_pose_5.header.frame_id = 'map'
    goal_pose_5.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_5.pose.position.x = -0.65
    goal_pose_5.pose.position.y = -0.1
    goal_pose_5.pose.orientation.z = 0.0
    goal_pose_5.pose.orientation.w = 1.0

    goal_pose_6 = PoseStamped()
    goal_pose_6.header.frame_id = 'map'
    goal_pose_6.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose_6.pose.position.x = -0.4
    goal_pose_6.pose.position.y = -0.1
    goal_pose_6.pose.orientation.z = 0.0
    goal_pose_6.pose.orientation.w = 1.0

    navigator.followWaypoints([
        goal_pose_1, 
        goal_pose_2, 
        goal_pose_3, 
        goal_pose_4, 
        goal_pose_5, 
        goal_pose_6,
    ])

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        print(feedback)

    result = navigator.getResult()
    print(result)

    navigator.lifecycleShutdown()

    rclpy.shutdown()

    exit(0)

if __name__ == '__main__':
    main()