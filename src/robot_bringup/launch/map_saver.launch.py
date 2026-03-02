#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_name = LaunchConfiguration('map_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='/home/yasiru/aep_maps/classroom_map',
        description='Output map path without extension'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver_cli',
        output='screen',
        arguments=['-f', map_name],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        map_saver,
    ])
