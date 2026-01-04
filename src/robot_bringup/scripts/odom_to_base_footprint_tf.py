#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomToBaseFootprintTF(Node):
    def __init__(self) -> None:
        super().__init__('odom_to_base_footprint_tf')

        try:
            self.declare_parameter('use_sim_time', False)
        except ParameterAlreadyDeclaredException:
            pass

        self._tf_broadcaster = TransformBroadcaster(self)
        self._sub = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

    def _odom_cb(self, msg: Odometry) -> None:
        try:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'

            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation

            self._tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().error(f"Failed to publish TF: {e}")


def main() -> None:
    rclpy.init()
    node = OdomToBaseFootprintTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Node crashed: {e}")
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()


