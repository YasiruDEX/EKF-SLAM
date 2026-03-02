#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
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

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='simple_test.sdf',
        description='Name of the world file to load (e.g. simple_test.sdf, complex_world.sdf)'
    )

    # --- Paths ---
    turtlebot3_desc_share = get_package_share_directory('turtlebot3_description')
    sim_share = get_package_share_directory('robot_sim')
    slam_share = get_package_share_directory('robot_slam')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    desc_share = get_package_share_directory('robot_description')  # For RViz config

    urdf_file = os.path.join(desc_share, 'urdf', 'turtlebot3_waffle_gz.urdf.xacro')
    world_path = PathJoinSubstitution([sim_share, 'worlds', LaunchConfiguration('world')])
    rviz_config = os.path.join(desc_share, 'config', 'rviz', 'nav.rviz')
    slam_params_file = os.path.join(slam_share, 'config', 'ekf_slam_params.yaml')

    # --- Set Gazebo resource path for mesh loading ---
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    gz_resource_path = os.path.dirname(turtlebot3_desc_share)
    
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_gz_path:
        gz_resource_path = f"{gz_resource_path}:{existing_gz_path}"
    
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )

    # Set TURTLEBOT3_MODEL environment variable
    set_turtlebot3_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='waffle'
    )

    # --- Robot Description from URDF (processed with xacro, namespace='') ---
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' namespace:=', '']),
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
            'robot_description': robot_description
        }]
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

    # --- Spawn TurtleBot3 in Gazebo ---
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_turtlebot3',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'turtlebot3_waffle',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
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
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/world/complex_cylinder_world/dynamic_pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- SLAM Toolbox ---
    robot_slam_toolbox_share = get_package_share_directory('robot_slam_toolbox')
    
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
             os.path.join(robot_slam_toolbox_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(robot_slam_toolbox_share, 'config', 'mapper_params_online_async.yaml')
        }.items()
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
        world_arg,
        set_gz_resource_path,
        set_turtlebot3_model,
        gz_server,
        robot_state_publisher_node,
        spawn_robot_node,
        bridge_node,
        slam_toolbox_launch, # Replaces ekf_slam_node
        rviz_node,
    ])
