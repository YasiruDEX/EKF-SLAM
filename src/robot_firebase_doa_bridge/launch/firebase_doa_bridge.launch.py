#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_firebase_doa_bridge',
            executable='firebase_doa_bridge_node.py',
            name='firebase_doa_bridge_node',
            output='screen',
            parameters=[{
                'firebase_base_url': 'https://classroom-bot-a7454-default-rtdb.asia-southeast1.firebasedatabase.app',
                'mic_data_path': 'mic_data',
                'firebase_auth_token': '',
                'poll_interval_sec': 1.0,
                'request_timeout_sec': 2.0,
                'service_name': '/rotate_to_yaw',
                'min_call_period_sec': 2.0,
                'doa_offset_deg': 0.0,
                'log_data_updates': True,
            }],
        )
    ])
