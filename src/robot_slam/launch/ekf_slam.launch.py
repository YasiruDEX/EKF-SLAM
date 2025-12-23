#!/usr/bin/env python3
"""
Launch file for the custom EKF-SLAM node.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    # Get EKF-SLAM params file path
    slam_params_file = os.path.join(
        get_package_share_directory('robot_slam'),
        'config',
        'ekf_slam_params.yaml'
    )

    # EKF-SLAM Node
    ekf_slam_node = Node(
        package='robot_slam',
        executable='ekf_slam_node',
        name='ekf_slam_node',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_slam_node,
    ])
