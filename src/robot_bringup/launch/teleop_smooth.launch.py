#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    max_linear_accel = LaunchConfiguration('max_linear_accel')
    max_linear_decel = LaunchConfiguration('max_linear_decel')
    max_angular_accel = LaunchConfiguration('max_angular_accel')
    max_angular_decel = LaunchConfiguration('max_angular_decel')

    return LaunchDescription([
        DeclareLaunchArgument('max_linear_accel', default_value='0.3'),
        DeclareLaunchArgument('max_linear_decel', default_value='0.3'),
        DeclareLaunchArgument('max_angular_accel', default_value='0.6'),
        DeclareLaunchArgument('max_angular_decel', default_value='0.6'),
        Node(
            package='robot_bringup',
            executable='cmd_vel_smoother.py',
            name='cmd_vel_smoother',
            output='screen',
            parameters=[{
                'input_topic': '/cmd_vel_raw',
                'output_topic': '/cmd_vel',
                'max_linear_accel': max_linear_accel,
                'max_linear_decel': max_linear_decel,
                'max_angular_accel': max_angular_accel,
                'max_angular_decel': max_angular_decel,
            }],
        ),
    ])
