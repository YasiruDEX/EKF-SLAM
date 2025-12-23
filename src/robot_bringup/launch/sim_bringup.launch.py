#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
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
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    urdf_xacro_path = os.path.join(desc_share, 'urdf', 'main.urdf.xacro')
    world_path = os.path.join(sim_share, 'worlds', 'classroom.sdf')
    rviz_config = os.path.join(desc_share, 'config', 'rviz', 'nav.rviz')

    # --- Set Gazebo resource path for mesh loading ---
    # Get parent directory of robot_description share to use as model path
    # This allows Gazebo to resolve package://robot_description/meshes/...
    gz_resource_path = os.path.dirname(desc_share)
    
    # Also get current GZ_SIM_RESOURCE_PATH if it exists
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_gz_path:
        gz_resource_path = f"{gz_resource_path}:{existing_gz_path}"
    
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )

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
            '0', '0', '0',
            '0', '0', '0',
            'base_footprint',
            'base_link'
        ]
    )

    # --- Gazebo Simulation ---
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', world_path]
        }.items()
    )

    # --- Spawn Robot in Gazebo (center of classroom) ---
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_AEP_Robot',
        output='screen',
        arguments=[
            '-world', 'classroom_world',
            '-topic', 'robot_description',
            '-name', 'AEP_Robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15',
            '-Y', '0.0',
        ]
    )

    # --- ROS-Gazebo Bridge ---
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        parameters=[{'use_sim_time': use_sim_time}]
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
        set_gz_resource_path,
        gz_server,
        robot_state_publisher_node,
        static_tf_node,
        spawn_robot_node,
        bridge_node,
        rviz_node,
    ])

