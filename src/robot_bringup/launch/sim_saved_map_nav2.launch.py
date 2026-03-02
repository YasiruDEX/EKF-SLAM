#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')
    nav_share = get_package_share_directory('robot_nav')

    default_map = os.path.expanduser('~/aep_maps/classroom_map.yaml')
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Absolute path to saved map yaml file'
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'sim_bringup.launch.py')
        )
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
        }.items()
    )

    return LaunchDescription([
        map_arg,
        sim_launch,
        TimerAction(period=3.0, actions=[nav_launch]),
    ])
