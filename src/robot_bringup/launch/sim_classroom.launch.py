#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz if true'
    )

    # Package paths
    bringup_share = get_package_share_directory('robot_bringup')
    sim_share = get_package_share_directory('robot_sim')
    desc_share = get_package_share_directory('robot_description')
    
    # Set Gazebo resource path so meshes can be found
    import os
    gz_resource_path = os.path.join(desc_share, '..')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] = gz_resource_path + ':' + os.environ['GZ_SIM_RESOURCE_PATH']
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = gz_resource_path

    # Include robot_sim world launch
    sim_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'classroom_world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    urdf_xacro_path = os.path.join(desc_share, 'urdf', 'aep_main.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro_path]),
        value_type=str
    )

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

    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )


    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
        ],
        output='screen'
    )


    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        arguments=[
        '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='scan_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
        ],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # RViz config path
    rviz_config = os.path.join(bringup_share, 'config', 'rviz', 'sim.rviz')

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(rviz),
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tf_bridge',
        arguments=[
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    odom_to_base_footprint_tf_node = Node(
        package='robot_bringup',
        executable='odom_to_base_footprint_tf.py',
        name='odom_to_base_footprint_tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

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
    

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        sim_world_launch,
        robot_state_publisher_node,
        clock_bridge,  
        tf_bridge,
        cmd_vel_bridge,
        odom_bridge,
        scan_bridge,
        joint_state_bridge,
        rviz_node,
        basefootprint_namespace_tf,
        lidar_tf,
        imu_tf,
        odom_to_base_footprint_tf_node,
    ])
