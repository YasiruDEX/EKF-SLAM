#!/usr/bin/python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from robot_rotate_goal.srv import RotateToYaw


def quaternion_to_yaw(orientation: Quaternion) -> float:
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half_yaw = yaw * 0.5
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half_yaw),
        w=math.cos(half_yaw),
    )


class RotateGoalService(Node):
    def __init__(self) -> None:
        super().__init__('rotate_goal_service')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('lookup_timeout_sec', 1.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.lookup_timeout_sec = float(self.get_parameter('lookup_timeout_sec').value)

        self.goal_publisher = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.service = self.create_service(RotateToYaw, 'rotate_to_yaw', self.handle_rotate_request)

        self.get_logger().info(
            f'Service ready: /rotate_to_yaw -> publishes {self.goal_topic} '
            f'using {self.map_frame} -> {self.base_frame}'
        )

    def handle_rotate_request(self, request: RotateToYaw.Request, response: RotateToYaw.Response):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.lookup_timeout_sec),
            )
        except TransformException as exc:
            response.success = False
            response.message = f'Could not read current pose: {exc}'
            self.get_logger().error(response.message)
            return response

        current_position = transform.transform.translation
        current_orientation = transform.transform.rotation
        current_yaw = quaternion_to_yaw(current_orientation)
        goal_yaw = math.radians(request.heading_deg)

        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self.map_frame
        goal_pose.pose.position.x = current_position.x
        goal_pose.pose.position.y = current_position.y
        goal_pose.pose.position.z = current_position.z
        goal_pose.pose.orientation = yaw_to_quaternion(goal_yaw)

        self.goal_publisher.publish(goal_pose)

        response.success = True
        response.message = (
            f'Published goal pose with target heading {request.heading_deg:.1f} deg '
            f'({goal_yaw:.3f} rad).'
        )
        response.goal_pose = goal_pose

        self.get_logger().info(
            f'Current yaw: {current_yaw:.3f} rad, goal yaw: {goal_yaw:.3f} rad, '
            f'input heading: {request.heading_deg:.1f} deg, published on {self.goal_topic}'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RotateGoalService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
