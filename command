source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_bringup sim_bringup.launch.py

source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_slam slam.launch.py

source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_nav navigation.launch.py

source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run nav2_map_server map_saver_cli -f ~/aep_maps/classroom_map



source /opt/ros/humble/setup.bash && so
urce install/setup.bash && ros2 launch robot_bringup sim_slam_nav2.launch.py


source /opt/ros/humble/setup.bash && so
urce install/setup.bash && ros2 launch robot_bringup sim_saved_map_nav2.launch.py





source /home/yasiru/Documents/github/autonomous_exam_proctoring_robot/install/setup.bash
ros2 launch robot_rotate_goal rotate_goal_service.launch.py



ros2 service call /rotate_to_yaw robot_rotate_goal/srv/RotateToYaw "{heading_deg: 90.0}"


source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_firebase_doa_bridge firebase_doa_bridge.launch.py


source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_waypoint_publisher waypoint_goal_publisher.launch.py
