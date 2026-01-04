#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    world_file = LaunchConfiguration('world_file')
    robot_name = LaunchConfiguration('robot_name')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    pkg_share_gazebo = get_package_share_directory('robot_gazebo')
    pkg_share_description = get_package_share_directory('robot_description')
    pkg_share_sim = get_package_share_directory('robot_sim')
    pkg_share_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    default_bridge_config = os.path.join(pkg_share_gazebo, 'config', 'ros_gz_bridge.yaml')
    model_sdf_path = os.path.join(pkg_share_gazebo, 'models', 'aep_robot', 'model.sdf')
    default_rviz_config = os.path.join(
        pkg_share_description, 'config', 'rviz', 'AEP_Robot.rviz')

    world_path = PathJoinSubstitution([
        pkg_share_sim,
        'worlds',
        world_file
    ])

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient (visualization)')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Flag to enable RViz')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=default_rviz_config,
        description='Full path to the RViz config file to use')

    declare_world_cmd = DeclareLaunchArgument(
        'world_file',
        default_value='classroom.sdf',
        description='World file name (e.g., classroom.sdf)')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='AEP_Robot',
        description='The name for the robot')

    declare_x_cmd = DeclareLaunchArgument(
        'x',
        default_value='2.0',
        description='x component of initial position, meters')

    declare_y_cmd = DeclareLaunchArgument(
        'y',
        default_value='-3.0',
        description='y component of initial position, meters')

    declare_z_cmd = DeclareLaunchArgument(
        'z',
        default_value='0.12',
        description='z component of initial position, meters')

    declare_roll_cmd = DeclareLaunchArgument(
        'roll',
        default_value='0.0',
        description='roll angle of initial orientation, radians')

    declare_pitch_cmd = DeclareLaunchArgument(
        'pitch',
        default_value='0.0',
        description='pitch angle of initial orientation, radians')

    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw',
        default_value='1.57',
        description='yaw angle of initial orientation, radians')

    urdf_xacro_path = os.path.join(pkg_share_description, 'urdf', 'aep_main.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro_path]),
        value_type=str
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_share_description, '..'))

    set_env_vars_models = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_share_gazebo, 'models'))

    start_gazebo_server_headless_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', [' -r -s -v 4 ', world_path])],
        condition=IfCondition(headless))

    start_gazebo_server_gui_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', [' -r -v 4 ', world_path])],
        condition=IfCondition(PythonExpression(['not ', headless])))


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

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': default_bridge_config}],
        output='screen')

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', model_sdf_path,
            '-name', robot_name,
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw
        ])

    basefootprint_namespace_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_aep_basefootprint_namespace',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'base_footprint',
            'AEP_Robot/base_footprint'
        ]
    )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_aep_lidar',
        arguments=[
            '0', '0.000420393431200835', '0.32565',
            '0', '0', '0',
            'AEP_Robot/base_footprint',
            'AEP_Robot/base_footprint/lidar'
        ]
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_aep_imu',
        arguments=[
            '0', '0', '0.15',
            '0', '0', '0',
            'AEP_Robot/base_footprint',
            'AEP_Robot/base_footprint/imu'
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_headless_cmd,
        declare_use_rviz_cmd,
        declare_rviz_config_file_cmd,
        declare_world_cmd,
        declare_robot_name_cmd,
        declare_x_cmd,
        declare_y_cmd,
        declare_z_cmd,
        declare_roll_cmd,
        declare_pitch_cmd,
        declare_yaw_cmd,
        set_env_vars_resources,
        set_env_vars_models,
        start_gazebo_server_headless_cmd,
        start_gazebo_server_gui_cmd,
        robot_state_publisher_node,
        bridge_node,
        spawn_robot_node,
        basefootprint_namespace_tf,
        lidar_tf,
        imu_tf,
        rviz_node,
    ])
