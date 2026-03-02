import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    urdf_path = os.path.join(desc_share, 'urdf', 'AEP_Robot.urdf')
    world_path = os.path.join(sim_share, 'worlds', 'classroom.sdf')

    # --- URDF / TF ---
    robot_description = ParameterValue(
        Command(['cat', ' ', urdf_path]),
        value_type=str
    )

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

    # --- Gazebo Classic ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path
        }.items()
    )

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_AEP_Robot',
        output='screen',
        arguments=[
            '-file', urdf_path,
            '-entity', 'AEP_Robot',
            '-x', '2.0',
            '-y', '-3.0',
            '-z', '0.1',
            '-Y', '1.57'
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        gazebo,
        robot_state_publisher_node,
        static_tf_node,
        spawn_robot_node,
    ])
