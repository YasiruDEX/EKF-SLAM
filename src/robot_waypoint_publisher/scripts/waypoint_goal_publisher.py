#!/usr/bin/python3

import math
from dataclasses import dataclass
from pathlib import Path
from typing import List

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class Waypoint:
    x: float
    y: float
    yaw_deg: float


def yaw_deg_to_quaternion(yaw_deg: float) -> Quaternion:
    yaw_rad = math.radians(yaw_deg)
    half_yaw = yaw_rad * 0.5
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half_yaw),
        w=math.cos(half_yaw),
    )


def quaternion_to_yaw_rad(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def shortest_angle_diff_rad(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


class WaypointGoalPublisher(Node):
    def __init__(self) -> None:
        super().__init__('waypoint_goal_publisher')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('xy_tolerance_m', 0.2)
        self.declare_parameter('yaw_tolerance_deg', 15.0)
        self.declare_parameter('tf_timeout_sec', 0.5)
        self.declare_parameter('check_rate_hz', 2.0)
        self.declare_parameter('required_consecutive_hits', 3)
        self.declare_parameter('loop_waypoints', False)

        self.waypoints_file = str(self.get_parameter('waypoints_file').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.xy_tolerance_m = float(self.get_parameter('xy_tolerance_m').value)
        self.yaw_tolerance_deg = float(self.get_parameter('yaw_tolerance_deg').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        self.check_rate_hz = float(self.get_parameter('check_rate_hz').value)
        self.required_consecutive_hits = int(self.get_parameter('required_consecutive_hits').value)
        self.loop_waypoints = bool(self.get_parameter('loop_waypoints').value)

        if not self.waypoints_file:
            raise RuntimeError('Parameter waypoints_file must be set to a valid file path.')

        self.waypoints = self._load_waypoints(Path(self.waypoints_file))
        if not self.waypoints:
            raise RuntimeError(f'No valid waypoints found in file: {self.waypoints_file}')

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_index = 0
        self.target_reached_hits = 0
        self.active_goal = None

        timer_period = 1.0 / max(self.check_rate_hz, 0.1)
        self.timer = self.create_timer(timer_period, self._tick)

        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints from {self.waypoints_file}. '
            f'Publishing to {self.goal_topic}.'
        )

        self._publish_current_goal()

    def _load_waypoints(self, file_path: Path) -> List[Waypoint]:
        if not file_path.exists():
            raise RuntimeError(f'Waypoint file does not exist: {file_path}')

        waypoints: List[Waypoint] = []
        for line_no, raw_line in enumerate(file_path.read_text().splitlines(), start=1):
            line = raw_line.strip()
            if not line or line.startswith('#'):
                continue

            cleaned = line.replace(',', ' ')
            parts = [part for part in cleaned.split() if part]
            if len(parts) not in (2, 3):
                self.get_logger().warn(
                    f'Skipping invalid waypoint line {line_no}: "{raw_line}" '
                    f'(expected: x y [yaw_deg])'
                )
                continue

            try:
                x = float(parts[0])
                y = float(parts[1])
                yaw_deg = float(parts[2]) if len(parts) == 3 else 0.0
            except ValueError:
                self.get_logger().warn(f'Skipping non-numeric waypoint line {line_no}: "{raw_line}"')
                continue

            waypoints.append(Waypoint(x=x, y=y, yaw_deg=yaw_deg))

        return waypoints

    def _make_goal_pose(self, wp: Waypoint) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = wp.x
        msg.pose.position.y = wp.y
        msg.pose.position.z = 0.0
        msg.pose.orientation = yaw_deg_to_quaternion(wp.yaw_deg)
        return msg

    def _publish_current_goal(self) -> None:
        wp = self.waypoints[self.current_index]
        goal = self._make_goal_pose(wp)
        self.active_goal = goal
        self.target_reached_hits = 0
        self.goal_pub.publish(goal)

        self.get_logger().info(
            f'Published goal {self.current_index + 1}/{len(self.waypoints)}: '
            f'x={wp.x:.2f}, y={wp.y:.2f}, yaw={wp.yaw_deg:.1f} deg'
        )

    def _tick(self) -> None:
        if self.active_goal is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException:
            return

        dx = self.active_goal.pose.position.x - transform.transform.translation.x
        dy = self.active_goal.pose.position.y - transform.transform.translation.y
        distance = math.hypot(dx, dy)

        current_yaw = quaternion_to_yaw_rad(transform.transform.rotation)
        target_yaw = quaternion_to_yaw_rad(self.active_goal.pose.orientation)
        yaw_error_deg = math.degrees(abs(shortest_angle_diff_rad(target_yaw, current_yaw)))

        within_xy = distance <= self.xy_tolerance_m
        within_yaw = yaw_error_deg <= self.yaw_tolerance_deg

        if within_xy and within_yaw:
            self.target_reached_hits += 1
        else:
            self.target_reached_hits = 0

        if self.target_reached_hits < self.required_consecutive_hits:
            return

        self.get_logger().info(
            f'Reached goal {self.current_index + 1}/{len(self.waypoints)} '
            f'(distance={distance:.2f}m, yaw_error={yaw_error_deg:.1f}deg).'
        )

        next_index = self.current_index + 1
        if next_index >= len(self.waypoints):
            if self.loop_waypoints:
                self.current_index = 0
                self._publish_current_goal()
            else:
                self.get_logger().info('All waypoints completed. Stopping waypoint publisher.')
                self.active_goal = None
                self.timer.cancel()
        else:
            self.current_index = next_index
            self._publish_current_goal()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointGoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
