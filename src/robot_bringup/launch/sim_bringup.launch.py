#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    # --- Paths ---
    desc_share = get_package_share_directory('robot_description')
    sim_share = get_package_share_directory('robot_sim')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    gazebo_ros_prefix = get_package_prefix('gazebo_ros')
    spawn_entity_script = os.path.join(gazebo_ros_prefix, 'lib', 'gazebo_ros', 'spawn_entity.py')

    urdf_xacro_path = os.path.join(desc_share, 'urdf', 'main.urdf.xacro')
    world_path = os.path.join(sim_share, 'worlds', 'world2.world')
    rviz_config = os.path.join(desc_share, 'config', 'rviz', 'nav.rviz')

    # --- Robot Description from XACRO ---
    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro_path]),
        value_type=str,
    )

    # --- Robot State Publisher ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }]
    )

    # --- Static TF: base_footprint -> base_link ---
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_basefootprint_baselink',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'base_link'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- Gazebo Simulation ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path
        }.items()
    )

    # --- Spawn Robot in Gazebo Classic (center of classroom) ---
    spawn_robot_node = ExecuteProcess(
        cmd=[
            '/usr/bin/python3',
            spawn_entity_script,
            '-topic', 'robot_description',
            '-entity', 'AEP_Robot',
            '-timeout', '120',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15',
            '-Y', '0.0'
        ],
        name='spawn_AEP_Robot',
        output='screen'
    )

    # --- RViz2 ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        gazebo,
        robot_state_publisher_node,
        static_tf_node,
        spawn_robot_node,
        rviz_node,
    ])

