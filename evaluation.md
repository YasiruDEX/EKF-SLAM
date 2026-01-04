# 4. Evaluation

This section presents the quantitative evaluation of the custom EKF-SLAM implementation using standard SLAM metrics.

## 4.1 Evaluation Metrics

The following metrics are used to quantitatively assess SLAM performance:

### 4.1.1 Absolute Trajectory Error (ATE)

ATE measures the global consistency of the estimated trajectory by computing the root mean square error between estimated and ground truth poses:

$$\text{ATE} = \sqrt{\frac{1}{n} \sum_{i=1}^{n} \| \mathbf{p}_i^{est} - \mathbf{p}_i^{gt} \|^2}$$

Where:
- $\mathbf{p}_i^{est}$ is the estimated position at time $i$
- $\mathbf{p}_i^{gt}$ is the ground truth position at time $i$
- $n$ is the number of pose samples

**Interpretation**: Lower ATE indicates better global localization accuracy.

### 4.1.2 Relative Pose Error (RPE)

RPE measures the local accuracy (drift) by comparing relative motion between consecutive poses:

$$\text{RPE} = \sqrt{\frac{1}{m} \sum_{i=1}^{m} \| (\mathbf{T}_i^{gt})^{-1} \mathbf{T}_{i+\delta}^{gt} - (\mathbf{T}_i^{est})^{-1} \mathbf{T}_{i+\delta}^{est} \|^2}$$

Where:
- $\mathbf{T}_i$ represents the pose transformation at time $i$
- $\delta$ is the evaluation interval (default: 10 samples)

**Interpretation**: Lower RPE indicates less drift per unit distance traveled.

### 4.1.3 Maximum Error

Maximum translation and rotation errors across the entire trajectory:
- **Max Translation Error**: Largest position deviation from ground truth
- **Max Rotation Error**: Largest orientation deviation from ground truth

## 4.2 Evaluation Procedure

### Ground Truth Source

In Gazebo simulation, the `/odom` topic provides ground truth pose when using the differential drive plugin with no noise. For more accurate ground truth, the `/ground_truth/pose` topic can be used if available.

### Data Collection

1. Launch the simulation environment
2. Start the EKF-SLAM node
3. Start the evaluation node
4. Navigate the robot through the environment
5. Collect paired pose estimates over the trajectory

### Running the Evaluator

```bash
# Terminal 1: Launch simulation
ros2 launch robot_bringup robot_sim_bringup.launch.py

# Terminal 2: Launch EKF-SLAM
ros2 launch robot_slam ekf_slam.launch.py use_sim_time:=true

# Terminal 3: Run evaluator
ros2 run robot_slam slam_evaluator --ros-args \
    -p ground_truth_topic:=/odom \
    -p slam_pose_topic:=/slam/pose \
    -p output_dir:=/tmp/slam_eval
```

## 4.3 Results

### Summary Table

| Metric | Value | Unit |
|--------|-------|------|
| **ATE Translation** | ___ | m |
| **ATE Rotation** | ___ | deg |
| **RPE Translation** | ___ | m |
| **RPE Rotation** | ___ | deg |
| **Max Translation Error** | ___ | m |
| **Max Rotation Error** | ___ | deg |
| **Evaluation Duration** | ___ | s |
| **Number of Samples** | ___ | count |

*Note: Fill in values after running the evaluation.*

### Trajectory Comparison

The evaluator outputs CSV files containing:
- `poses_<timestamp>.csv`: Time-synchronized ground truth and SLAM poses
- `metrics_<timestamp>.csv`: Computed evaluation metrics

### Performance Analysis

**Translation Accuracy**: 
- ATE translation indicates overall position accuracy
- Values < 0.1m indicate excellent performance for indoor SLAM

**Rotation Accuracy**:
- ATE rotation indicates heading estimation accuracy
- Values < 5° indicate good orientation tracking

**Drift Assessment**:
- RPE measures accumulated error over trajectory segments
- Low RPE confirms effective scan matching correction

## 4.4 Comparison with slam_toolbox

For reference, the same evaluation can be performed against `slam_toolbox`:

```bash
# Run slam_toolbox instead of custom SLAM
ros2 launch slam_toolbox online_async_launch.py

# Modify evaluator to subscribe to slam_toolbox pose
ros2 run robot_slam slam_evaluator --ros-args \
    -p slam_pose_topic:=/slam_toolbox/pose
```

| Metric | Custom EKF-SLAM | slam_toolbox |
|--------|-----------------|--------------|
| ATE Translation (m) | ___ | ___ |
| ATE Rotation (deg) | ___ | ___ |
| RPE Translation (m) | ___ | ___ |
| CPU Usage (%) | ___ | ___ |
| Memory (MB) | ___ | ___ |

## 4.5 Discussion of Results

### Strengths
- Real-time performance with low computational overhead
- Smooth trajectory estimates due to EKF filtering
- Effective drift correction through scan matching

### Limitations
- No loop closure detection limits long-term accuracy
- Performance depends on environment geometry
- May accumulate drift in feature-poor areas

### Recommendations
- Use in environments with sufficient geometric features
- Consider hybrid approach with loop closure for larger areas
- Tune noise parameters for specific robot and sensor characteristics
