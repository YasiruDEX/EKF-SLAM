#!/usr/bin/env python3
"""
Converts Odometry messages to PoseStamped for EVO evaluation.
In simulation, the odometry from the DiffDrive plugin is perfect ground truth.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class OdomToPose(Node):
    def __init__(self):
        super().__init__('odom_to_pose')
        self.declare_parameter('input_topic', '/odom')
        self.declare_parameter('output_topic', '/ground_truth_pose')

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.sub = self.create_subscription(Odometry, self.input_topic, self.callback, 10)
        self.pub = self.create_publisher(PoseStamped, self.output_topic, 10)

        self.get_logger().info(f"Converting {self.input_topic} (Odometry) -> {self.output_topic} (PoseStamped)")

    def callback(self, msg):
        out = PoseStamped()
        out.header = msg.header
        out.pose = msg.pose.pose
        self.pub.publish(out)

def main():
    rclpy.init()
    node = OdomToPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
