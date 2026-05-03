#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_waypoint_publisher')
    default_waypoints = os.path.join(pkg_share, 'config', 'test_waypoints.txt')

    return LaunchDescription([
        Node(
            package='robot_waypoint_publisher',
            executable='waypoint_goal_publisher.py',
            name='waypoint_goal_publisher',
            output='screen',
            parameters=[{
                'waypoints_file': default_waypoints,
                'goal_topic': '/goal_pose',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'xy_tolerance_m': 0.20,
                'yaw_tolerance_deg': 15.0,
                'tf_timeout_sec': 0.5,
                'check_rate_hz': 2.0,
                'required_consecutive_hits': 3,
                'loop_waypoints': False,
            }],
        )
    ])
