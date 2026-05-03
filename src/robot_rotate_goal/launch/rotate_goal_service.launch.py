#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_rotate_goal',
            executable='rotate_goal_service.py',
            name='rotate_goal_service',
            output='screen',
            parameters=[{
                'map_frame': 'map',
                'base_frame': 'base_link',
                'goal_topic': '/goal_pose',
            }],
        )
    ])
