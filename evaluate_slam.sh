#!/bin/bash
# evaluate_slam.sh

# Stop on error (disabled to allow cleanup on failure)
# set -e

# Cleanup previous results
rm -rf slam_eval_bag results
mkdir -p results

# Source workspace
source install/setup.sh

echo "------------------------------------------------"
echo "Starting SLAM Evaluation Pipeline"
echo "------------------------------------------------"

# 1. Launch Simulation (Background)
echo "[1/6] Launching Simulation..."
ros2 launch robot_bringup turtlebot3_sim.launch.py world:=complex_world.sdf > /dev/null 2>&1 &
SIM_PID=$!
sleep 20  # Wait for Sim/RViz/SLAM to initialize

# 2. Start Helper Nodes
echo "[2/6] Starting Helper Nodes..."
# TF to Pose converter (for estimated pose from SLAM)
python3 src/robot_slam_toolbox/scripts/tf_to_pose.py &
TF_PID=$!
# Extract ground truth from odometry (perfect in simulation)
python3 src/robot_slam_toolbox/scripts/odom_to_pose.py \
    --ros-args \
    -p input_topic:=/odom \
    -p output_topic:=/ground_truth_pose &
CONV_PID=$!
sleep 5

# 3. Start Recording
echo "[3/6] Recording Data (60s)..."
# Record ground truth and estimated pose
# Using timeout with SIGINT to properly close the bag
timeout --preserve-status -s SIGINT 45s ros2 bag record -o slam_eval_bag /ground_truth_pose /estimated_pose &
BAG_PID=$!

# 4. Move Robot
echo "[4/6] Moving Robot (Circle Trajectory)..."
# Publish velocity command for 60s
timeout --preserve-status 60s ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.2}}" > /dev/null 2>&1

# Wait for recording to finish
wait $BAG_PID || true

echo "Recording finished."

# 5. Cleanup Processes
echo "[5/6] Stopping Simulation..."
kill $TF_PID
kill $CONV_PID
kill $SIM_PID
pkill -f "ign gazebo"
pkill -f "rviz2"
pkill -f "robot_state_publisher"
pkill -f "parameter_bridge"
pkill -f "async_slam_toolbox_node"
sleep 5

# 6. Run EVO Analysis
echo "[6/6] Analyzing Data with EVO..."

BAG_FILE=$(find slam_eval_bag -name "*.db3" -o -name "*.mcap" | head -n 1)

if [ -z "$BAG_FILE" ]; then
    echo "Error: No bag file found!"
    exit 1
fi

echo "   Processing $BAG_FILE..."

# Absolute Trajectory Error (ATE)
evo_ape bag2 "$BAG_FILE" /ground_truth_pose /estimated_pose \
    -v --plot --plot_mode=xy \
    --t_max_diff 0.5 \
    --save_results results/ate.zip \
    --save_plot results/ate_plot.png

# Relative Pose Error (RPE)
evo_rpe bag2 "$BAG_FILE" /ground_truth_pose /estimated_pose \
    -v --plot --plot_mode=xy \
    --t_max_diff 0.5 \
    --save_results results/rpe.zip \
    --save_plot results/rpe_plot.png

# Trajectory Plot
evo_traj bag2 "$BAG_FILE" /ground_truth_pose /estimated_pose \
    --plot --plot_mode=xy \
    --save_plot results/traj_plot.png

echo "------------------------------------------------"
echo "Evaluation Complete!"
echo "Results saved in ./results/"
echo "------------------------------------------------"
