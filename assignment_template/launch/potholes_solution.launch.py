#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    yolo_node = Node(
        package='assignment_template',
        executable='yolo_pothole',
        output='screen'
    )
    find_poses_node = Node(
        package='cpp_assignment',
        executable='find_pothole_poses',
        output='screen'
    )
    clustering_node = Node(
        package='assignment_template',
        executable='pothole_clustering',
        output='screen'
    )
    navigation_node = Node(
        package='assignment_template',
        executable='follow_path',
        output='screen'
    )

    return LaunchDescription([
        yolo_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=yolo_node,
                on_start=[LogInfo(msg="Started YOLO node. "), find_poses_node]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=find_poses_node,
                on_start=[LogInfo(msg="Started find poses node. "), clustering_node]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=clustering_node,
                on_start=[LogInfo(msg="Started clustering node. Now Starting navigation"), navigation_node]
            )
        )
    ])