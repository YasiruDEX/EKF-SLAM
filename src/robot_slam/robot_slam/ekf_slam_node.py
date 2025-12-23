#!/usr/bin/env python3
"""
EKF-SLAM ROS2 Node
Implements Extended Kalman Filter based SLAM with occupancy grid mapping.

Subscriptions:
    - /scan (sensor_msgs/LaserScan): Laser scan data
    - /odom (nav_msgs/Odometry): Wheel odometry

Publications:
    - /map (nav_msgs/OccupancyGrid): Generated occupancy grid map
    - /slam/pose (geometry_msgs/PoseWithCovarianceStamped): Robot pose estimate
    - /tf (tf2_msgs/TFMessage): map -> odom transform
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import tf2_ros

from robot_slam.ekf_core import EKFCore
from robot_slam.occupancy_grid import OccupancyGrid as OccGrid
from robot_slam.scan_matcher import ScanMatcher


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles to quaternion."""
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    """Extract yaw from quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)


class EKFSlamNode(Node):
    """
    ROS2 Node for EKF-SLAM.
    
    Combines EKF-based pose estimation with occupancy grid mapping.
    Uses odometry for prediction and scan matching for correction.
    """
    
    def __init__(self):
        super().__init__('ekf_slam_node')
        
        
        # Declare parameters (use_sim_time is auto-declared by ROS2)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_size', 30.0)
        self.declare_parameter('map_publish_rate', 1.0)
        self.declare_parameter('pose_publish_rate', 10.0)
        self.declare_parameter('use_scan_matching', True)
        
        # Get parameters
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        map_resolution = self.get_parameter('map_resolution').value
        map_size = self.get_parameter('map_size').value
        self.use_scan_matching = self.get_parameter('use_scan_matching').value
        
        # Initialize EKF
        self.ekf = EKFCore()
        
        # Initialize occupancy grid
        self.occupancy_grid = OccGrid(
            width=map_size,
            height=map_size,
            resolution=map_resolution,
            origin_x=-map_size/2,
            origin_y=-map_size/2
        )
        
        # Initialize scan matcher
        self.scan_matcher = ScanMatcher(
            max_iterations=30,
            tolerance=1e-4,
            max_correspondence_distance=0.3
        )
        
        # State variables
        self.last_odom = None
        self.last_odom_time = None
        self.last_scan = None
        self.initialized = False
        
        # map -> odom transform (initially identity)
        self.map_to_odom_x = 0.0
        self.map_to_odom_y = 0.0
        self.map_to_odom_theta = 0.0
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            sensor_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            sensor_qos
        )
        
        # Publishers
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            map_qos
        )
        
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/slam/pose',
            10
        )
        
        # Timers
        map_rate = self.get_parameter('map_publish_rate').value
        self.map_timer = self.create_timer(1.0 / map_rate, self.publish_map)
        
        pose_rate = self.get_parameter('pose_publish_rate').value
        self.tf_timer = self.create_timer(1.0 / pose_rate, self.publish_tf)
        
        self.get_logger().info('EKF-SLAM node initialized')
        self.get_logger().info(f'  Map frame: {self.map_frame}')
        self.get_logger().info(f'  Odom frame: {self.odom_frame}')
        self.get_logger().info(f'  Base frame: {self.base_frame}')
        self.get_logger().info(f'  Scan matching: {self.use_scan_matching}')
        
        # TF publishing counter for debug
        self.tf_publish_count = 0
    
    def odom_callback(self, msg: Odometry):
        """Process odometry message for EKF prediction."""
        current_time = Time.from_msg(msg.header.stamp)
        
        # Get current odometry pose
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        odom_theta = quaternion_to_yaw(msg.pose.pose.orientation)
        
        if not self.initialized:
            # Initialize state from first odometry
            self.ekf.set_state([odom_x, odom_y, odom_theta])
            self.last_odom = (odom_x, odom_y, odom_theta)
            self.last_odom_time = current_time
            self.initialized = True
            self.get_logger().info(f'EKF initialized at ({odom_x:.2f}, {odom_y:.2f}, {odom_theta:.2f})')
            return
        
        # Calculate time delta
        dt = (current_time.nanoseconds - self.last_odom_time.nanoseconds) / 1e9
        if dt <= 0:
            return
        
        # Get velocities from odometry
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        
        # EKF prediction step
        self.ekf.predict(v, omega, dt)
        
        # Update last odometry
        self.last_odom = (odom_x, odom_y, odom_theta)
        self.last_odom_time = current_time
        
        # Update map -> odom transform based on EKF state
        state, _ = self.ekf.get_state()
        self.update_map_to_odom_transform(state, odom_x, odom_y, odom_theta)
        
        # Publish TF immediately (don't rely on timer with sim time)
        self.publish_tf(msg.header.stamp)
    
    def scan_callback(self, msg: LaserScan):
        """Process laser scan for mapping and optional scan matching."""
        if not self.initialized:
            return
        
        # Get current state
        state, cov = self.ekf.get_state()
        robot_x, robot_y, robot_theta = state
        
        # Scan matching update (if enabled)
        if self.use_scan_matching:
            dx, dy, dtheta, fitness, success = self.scan_matcher.match(
                np.array(msg.ranges),
                msg.angle_min,
                msg.angle_increment,
                range_min=msg.range_min,
                range_max=msg.range_max
            )
            
            if success and fitness < 0.15:
                # Apply scan matching correction
                self.ekf.update_with_scan_matching(dx, dy, dtheta)
                state, cov = self.ekf.get_state()
                robot_x, robot_y, robot_theta = state
        
        # Resize map if needed
        self.occupancy_grid.resize_if_needed(robot_x, robot_y)
        
        # Update occupancy grid with scan
        self.occupancy_grid.update_with_scan(
            robot_x, robot_y, robot_theta,
            np.array(msg.ranges),
            msg.angle_min,
            msg.angle_increment,
            msg.range_min,
            msg.range_max
        )
        
        self.last_scan = msg
        
        # Publish map every 10th scan (don't rely on timer with sim time)
        self.scan_count = getattr(self, 'scan_count', 0) + 1
        if self.scan_count % 10 == 0:
            self.publish_map()
    
    def update_map_to_odom_transform(self, ekf_state, odom_x, odom_y, odom_theta):
        """
        Update the map -> odom transform.
        
        The EKF state is in the map frame, and we need to compute
        the transform from map to odom such that:
        T_map_base = T_map_odom * T_odom_base
        """
        map_x, map_y, map_theta = ekf_state
        
        # Compute map -> odom transform
        # T_map_odom = T_map_base * T_base_odom
        # T_base_odom = inverse of T_odom_base
        
        cos_odom = np.cos(odom_theta)
        sin_odom = np.sin(odom_theta)
        
        # Robot pose relative to odom (T_odom_base)
        # We want T_map_odom = T_map_base * inv(T_odom_base)
        
        # inv(T_odom_base): rotate by -odom_theta, translate by -R.T @ t
        self.map_to_odom_theta = map_theta - odom_theta
        
        cos_diff = np.cos(self.map_to_odom_theta)
        sin_diff = np.sin(self.map_to_odom_theta)
        
        self.map_to_odom_x = map_x - (cos_diff * odom_x - sin_diff * odom_y)
        self.map_to_odom_y = map_y - (sin_diff * odom_x + cos_diff * odom_y)
    
    def publish_tf(self, timestamp=None):
        """Publish map -> odom transform."""
        t = TransformStamped()
        if timestamp is not None:
            t.header.stamp = timestamp
        else:
            t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        
        t.transform.translation.x = self.map_to_odom_x
        t.transform.translation.y = self.map_to_odom_y
        t.transform.translation.z = 0.0
        
        t.transform.rotation = euler_to_quaternion(0, 0, self.map_to_odom_theta)
        
        self.tf_broadcaster.sendTransform(t)
        
        # Debug logging every 50 publishes
        self.tf_publish_count += 1
        if self.tf_publish_count % 50 == 1:
            self.get_logger().info(f'TF published: map->odom at ({self.map_to_odom_x:.3f}, {self.map_to_odom_y:.3f}, {self.map_to_odom_theta:.3f})')
        
        # Also publish pose if initialized
        if self.initialized:
            self.publish_pose()
    
    def publish_pose(self):
        """Publish robot pose estimate."""
        state, cov = self.ekf.get_state()
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.map_frame
        
        pose_msg.pose.pose.position.x = state[0]
        pose_msg.pose.pose.position.y = state[1]
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation = euler_to_quaternion(0, 0, state[2])
        
        # Fill covariance (6x6 for x, y, z, roll, pitch, yaw)
        pose_cov = np.zeros((6, 6))
        pose_cov[0, 0] = cov[0, 0]  # x
        pose_cov[1, 1] = cov[1, 1]  # y
        pose_cov[5, 5] = cov[2, 2]  # yaw
        pose_cov[0, 1] = pose_cov[1, 0] = cov[0, 1]
        pose_cov[0, 5] = pose_cov[5, 0] = cov[0, 2]
        pose_cov[1, 5] = pose_cov[5, 1] = cov[1, 2]
        
        pose_msg.pose.covariance = pose_cov.flatten().tolist()
        
        self.pose_pub.publish(pose_msg)
    
    def publish_map(self):
        """Publish occupancy grid map."""
        if not self.initialized:
            return
        
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        
        msg.info.resolution = self.occupancy_grid.resolution
        msg.info.width = self.occupancy_grid.grid_width
        msg.info.height = self.occupancy_grid.grid_height
        msg.info.origin.position.x = self.occupancy_grid.origin_x
        msg.info.origin.position.y = self.occupancy_grid.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation = euler_to_quaternion(0, 0, 0)
        
        msg.data = self.occupancy_grid.get_occupancy_grid_msg_data().tolist()
        
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFSlamNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down EKF-SLAM node...')
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Ignore shutdown errors


if __name__ == '__main__':
    main()
