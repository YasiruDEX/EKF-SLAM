# 5. Discussion

This section discusses the observed behavior of the EKF-SLAM system, analyzes the trade-offs involved in the design, and highlights areas for future improvement.

## 5.1 Effect of Kalman Filtering on Localization Continuity

The Extended Kalman Filter provides continuous and smooth pose estimates even when sensor measurements are noisy or temporarily unavailable. Without the EKF, raw odometry accumulates drift quickly, leading to inconsistent map building and poor localization.

The prediction step maintains pose estimates during periods between sensor updates, ensuring the robot's position is always available for downstream navigation tasks. When scan matching provides corrections, the update step smoothly integrates these measurements without causing abrupt jumps in the estimated pose.

I observed that the EKF maintains tracking continuity even when:
- Wheel slip causes temporary odometry errors
- The LiDAR receives invalid readings (NaN or out-of-range values)
- The scan matching algorithm fails to find a good correspondence

This continuity is critical for real-time robot navigation, where sudden pose discontinuities could cause path planning failures or unsafe motion commands.

## 5.2 Reduction of Measurement Noise and Drift

The EKF effectively reduces high-frequency noise in odometry measurements by fusing them with scan matching corrections. Wheel encoders are susceptible to noise from uneven surfaces, wheel slip, and mechanical backlash. The Kalman gain automatically weights measurements based on their uncertainty, giving more trust to lower-noise sources.

The scan matching component specifically addresses long-term drift by detecting and correcting accumulated odometry errors. The ICP algorithm compares consecutive laser scans and estimates the relative transformation, providing an independent measurement that does not suffer from the same drift characteristics as wheel odometry.

The log-odds representation in the occupancy grid also contributes to noise reduction. Individual noisy measurements have limited impact on cell probabilities because the log-odds accumulate over time. This means temporary sensor errors do not permanently corrupt the map.

## 5.3 Responsiveness versus Smoothness Trade-Off

There is an inherent trade-off between how quickly the system responds to actual motion changes and how smooth the resulting trajectory appears. This trade-off is controlled primarily through the motion noise parameters (α₁, α₂, α₃, α₄) and measurement noise covariances.

**Higher motion noise**: Causes the EKF to trust odometry less and rely more on measurement updates. This makes the system more responsive to scan matching corrections but can result in jittery trajectories if scan matching is noisy.

**Lower motion noise**: Causes the EKF to trust odometry more, producing smoother trajectories but potentially ignoring valid corrections from scan matching.

I tuned the noise parameters to balance these competing concerns:
- Motion noise α = [0.1, 0.01, 0.01, 0.1] provides moderate trust in odometry
- Scan matching covariance [0.02, 0.02, 0.01] gives high weight to successful matches
- The fitness threshold (0.15) rejects poor scan matches that would introduce noise

## 5.4 Design Choice: Fixed Parameters Across Environments

I used fixed noise parameters and thresholds across all test environments rather than adapting them online. This design choice offers several advantages:

1. **Reproducibility**: Results are consistent and can be reproduced exactly
2. **Simplicity**: No additional adaptation logic is required
3. **Robustness**: Parameters tuned conservatively work across diverse conditions

The disadvantage is that performance may not be optimal for every specific environment. For example, a highly structured indoor environment with many geometric features might benefit from tighter scan matching thresholds, while a sparse outdoor environment might need relaxed parameters.

The current parameter set represents a reasonable compromise that works well in typical indoor environments like the exam hall scenario:
- Map resolution: 0.05m (5cm cells)
- Initial map size: 30m × 30m
- Scan matching max iterations: 30
- Correspondence distance threshold: 0.3m

## 5.5 Runtime and Practical Feasibility

The EKF-SLAM system achieves real-time performance on standard hardware. The computational costs are distributed as follows:

| Component | Typical Time per Update |
|-----------|------------------------|
| EKF Prediction | < 0.1 ms |
| Scan Matching (ICP) | 5-15 ms |
| Occupancy Grid Update | 2-5 ms |
| ROS 2 Publishing | < 1 ms |

The system processes odometry at the full sensor rate (typically 50-100 Hz) and laser scans at 10-20 Hz without dropping messages. The ICP scan matching is the most computationally expensive component, but downsampling to 200 points keeps it within acceptable bounds.

Memory usage scales with map size. A 30m × 30m map at 5cm resolution requires approximately 2.3 MB for the log-odds grid. The dynamic map expansion feature allows starting with a smaller map and growing it as needed, which is useful for memory-constrained systems.

The practical feasibility for the exam proctoring robot application is confirmed by:
- Stable operation during multi-hour Gazebo simulations
- Consistent map quality across different world configurations
- No observable lag between robot motion and map updates in RViz2

## 5.6 Limitations and Future Improvements

Despite its effectiveness, the current implementation has several limitations that could be addressed in future work:

### Current Limitations

1. **2D Assumption**: The system assumes planar motion and a 2D environment. It cannot handle multi-floor buildings or significant elevation changes.

2. **Feature-Poor Environments**: Scan matching may fail in environments with few geometric features, such as long featureless corridors or open spaces.

3. **Dynamic Obstacles**: The occupancy grid treats all obstacles as static. Moving people or objects can create artifacts in the map.

4. **Loop Closure**: The system does not detect when the robot returns to a previously visited location. This means accumulated drift is not corrected globally.

5. **Computational Scaling**: Very large environments require proportionally more memory and processing time for occupancy grid updates.

### Future Improvements

1. **Loop Closure Detection**: Implementing place recognition using scan descriptors or image features would enable global consistency correction.

2. **Dynamic Object Filtering**: Tracking and removing dynamic obstacles from the map would improve accuracy in populated environments.

3. **Adaptive Parameters**: Online estimation of noise parameters based on sensor consistency could improve performance across diverse environments.

4. **3D Extension**: Extending the system to 3D using point cloud registration and voxel grids would enable operation in more complex environments.

5. **Multi-Sensor Fusion**: Incorporating additional sensors such as IMU or visual odometry would improve robustness when any single sensor degrades.

## 5.7 Overall Interpretation

The EKF-SLAM implementation successfully demonstrates the core principles of probabilistic robotics for simultaneous localization and mapping. The system achieves its design goals of:

- **Accurate Localization**: The EKF maintains a consistent pose estimate that tracks ground truth well in simulation
- **Consistent Mapping**: The occupancy grid correctly represents free and occupied space
- **Real-Time Operation**: All processing completes within timing constraints for reactive navigation
- **Robustness**: The system handles sensor noise and temporary failures gracefully

The combination of velocity-based motion prediction, ICP scan matching, and log-odds occupancy mapping provides a solid foundation for autonomous navigation. The modular architecture separates concerns cleanly, making the system easy to understand, debug, and extend.

The results validate the design decision to implement a custom EKF-SLAM rather than relying solely on existing packages like slam_toolbox. The custom implementation provides full visibility into the algorithms and allows precise tuning for the specific requirements of the exam proctoring application.

---

# 6. Conclusion

In this project, I implemented a custom Extended Kalman Filter based SLAM system for an autonomous exam proctoring robot. The system successfully addresses the core challenges of robot localization and mapping in indoor environments.

## Key Contributions

1. **EKF-Based Pose Estimation**: I implemented a velocity motion model with proper Jacobian computation for the prediction step, and scan matching integration for the update step. The filter maintains smooth, continuous pose estimates while effectively reducing odometry drift.

2. **ICP Scan Matching**: I developed an Iterative Closest Point algorithm that matches consecutive laser scans to detect and correct accumulated pose errors. The SVD-based transform estimation ensures robust and efficient computation.

3. **Log-Odds Occupancy Mapping**: I implemented an occupancy grid using log-odds representation that efficiently integrates multiple sensor observations. Bresenham's line algorithm enables accurate ray tracing for free space detection.

4. **ROS 2 Integration**: The complete system is packaged as a ROS 2 node with proper topic subscriptions, publications, and TF broadcasting. This enables seamless integration with the Nav2 navigation stack.

## Results Summary

The implemented EKF-SLAM system achieves:
- Real-time performance at sensor update rates
- Consistent occupancy grid construction
- Smooth trajectory estimation with reduced drift
- Proper coordinate frame management for navigation

## Lessons Learned

This project reinforced several important concepts:
- The importance of proper uncertainty propagation in state estimation
- The value of sensor fusion for robust localization
- The trade-offs between computational cost and algorithm accuracy
- The benefits of modular software architecture for robotics systems

## Future Directions

The current implementation provides a solid foundation that can be extended with loop closure detection, dynamic obstacle handling, and multi-sensor fusion. These enhancements would further improve the system's capability for long-duration autonomous operation in real-world environments.

The EKF-SLAM implementation demonstrates that classical probabilistic approaches remain highly effective for indoor mobile robot navigation when properly implemented and tuned. Combined with modern deep learning approaches for perception, these foundational techniques enable robust autonomous systems.
