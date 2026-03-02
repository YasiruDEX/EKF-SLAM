#!/bin/bash
BAG_FILE=$(find slam_eval_bag -name "*.db3" -o -name "*.mcap" | head -n 1)
if [ -z "$BAG_FILE" ]; then
    echo "Error: No bag file found!"
    exit 1
fi
echo "Processing $BAG_FILE"

# Use 'bag2' for ROS 2 bags
# Removed --save_table if not supported
evo_ape bag2 "$BAG_FILE" /ground_truth_pose /estimated_pose -va --plot --plot_mode=xy --save_results results/ate.zip --save_plot results/ate_plot.png 

evo_rpe bag2 "$BAG_FILE" /ground_truth_pose /estimated_pose -va --plot --plot_mode=xy --save_results results/rpe.zip --save_plot results/rpe_plot.png 

evo_traj bag2 "$BAG_FILE" /ground_truth_pose /estimated_pose --plot --plot_mode=xy --save_plot results/traj_plot.png
