source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_bringup sim_bringup.launch.py

source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_slam slam.launch.py

source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch robot_nav navigation.launch.py

source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run nav2_map_server map_saver_cli -f ~/aep_maps/classroom_map