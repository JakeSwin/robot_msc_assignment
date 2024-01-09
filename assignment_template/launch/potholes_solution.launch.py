#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='assignment_template',
            executable='follow_path',
            output='screen'),
        Node(
            package='assignment_template',
            executable='yolo_pothole',
            output='screen'),
        Node(
            package='assignment_template',
            executable='pothole_clustering',
            output='screen'),
        Node(
            package='cpp_assignment',
            executable='find_pothole_poses',
            output='screen'),
    ])