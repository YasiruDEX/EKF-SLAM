# Appendix A: AI Assistance Disclosure

**Tools Used:** Claude (Anthropic)

## Purpose and Scope of AI Assistance

AI assistance was used for specific aspects of the documentation and debugging:

### 1. LaTeX Equation Formatting
The AI assisted in formatting mathematical equations for the methodology section, including:
- EKF state vector and covariance matrix representation
- Velocity motion model equations for straight-line and arc motion
- Jacobian matrices for state prediction
- Kalman gain and update equations
- Log-odds probability conversions

### 2. Documentation Structure
The AI helped organize the report structure and create consistent formatting across methodology, implementation, and discussion sections.

### 3. Grammar and Technical Writing
AI assistance was used for proofreading and improving clarity of technical descriptions throughout the document.

### 4. ROS 2 Debugging and QoS Configuration
The AI assisted in resolving ROS 2 communication issues related to:
- QoS policy mismatches between publishers and subscribers
- TF transform timing and delays with simulation time
- Point cloud and laser scan visualization in RViz2
- Map topic reliability settings for late-joining subscribers

### 5. Code Troubleshooting
The AI helped debug implementation issues in the EKF-SLAM node, particularly related to coordinate frame transformations and timing synchronization.

---

## Prompts Used

### LaTeX and Equation Formatting

1. "Write the EKF state vector equation in LaTeX format with x, y, theta components"

2. "Format the velocity motion model prediction equations for both straight line and arc motion in LaTeX"

3. "Write the Jacobian matrix G for the velocity motion model in LaTeX with partial derivatives"

4. "Format the Kalman gain equation K = P * H^T * S^-1 in proper LaTeX notation"

5. "Write the covariance prediction equation P = G * P * G^T + V * M * V^T in LaTeX"

6. "Format the log-odds to probability conversion equation P = 1/(1 + exp(-l)) in LaTeX"

7. "Write the motion noise covariance matrix M with alpha parameters in LaTeX"

### Grammar and Writing Corrections

8. "Proofread this methodology section for grammar and technical clarity"

9. "Improve the flow of this implementation description while maintaining technical accuracy"

10. "Make this paragraph more concise while keeping the key technical details"

### ROS 2 and RViz Debugging

11. "RViz2 is not showing the /map topic even though rostopic echo shows data is being published. How do I fix this?"

12. "What QoS settings should I use for OccupancyGrid publisher so late-joining subscribers receive the map?"

13. "My TF transform from map to odom has timing issues with use_sim_time. The robot jumps around in RViz. How to fix?"

14. "LaserScan topic not displaying in RViz2 - publisher uses BEST_EFFORT but RViz expects RELIABLE. What QoS should I use?"

15. "How do I set TRANSIENT_LOCAL durability for map publisher so RViz shows the map immediately when opened?"

16. "The map→odom transform is delayed causing the robot to appear in wrong position. How to synchronize TF with odometry timestamp?"

17. "Point cloud from laser scan not rendering in RViz2. Getting QoS incompatible warning. How to configure sensor QoS?"

### Publishing Rate and Timing

18. "What is a good publishing rate for occupancy grid map topic? Currently at 10Hz and causing CPU issues"

19. "How to publish TF transform synchronized with odometry message timestamp instead of current time?"

20. "Map updates are lagging behind robot motion. Should I publish map on timer or triggered by scan callback?"

### Code Implementation Issues

21. "Getting numpy linalg error when computing Kalman gain - matrix is singular. How to handle this edge case?"

22. "Scan matching ICP not converging - returns identity transform. What convergence criteria should I check?"

23. "Bresenham line algorithm returning wrong cells when robot is at negative coordinates. How to fix world to grid conversion?"

24. "Quaternion to yaw conversion giving wrong angle. Show the correct formula for extracting yaw from quaternion"

25. "Occupancy grid origin position is wrong in RViz - map appears offset from robot. How to set origin correctly?"

---

## Nature of AI Contribution

The AI assistance was primarily used for:
- **Documentation**: Formatting equations and improving technical writing clarity
- **Debugging**: Resolving ROS 2 middleware and visualization issues
- **Reference**: Confirming correct mathematical formulations and coordinate transformations

The core algorithmic implementations (EKF prediction/update, ICP scan matching, log-odds mapping) were developed based on course materials and robotics textbooks, with AI assistance limited to debugging specific implementation issues and formatting the documentation.
