#!/usr/bin/env python3
"""
Extracts ground truth robot pose from Ignition's dynamic_pose/info topic.
The topic contains a Pose_V (vector of poses) message with all model poses.
We filter for the 'turtlebot3_waffle' model and publish it as PoseStamped.
"""
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped

class GroundTruthExtractor(Node):
    def __init__(self):
        super().__init__('ground_truth_extractor')
        self.declare_parameter('input_topic', '/world/complex_cylinder_world/dynamic_pose/info')
        self.declare_parameter('output_topic', '/ground_truth_pose')
        self.declare_parameter('model_name', 'turtlebot3_waffle')

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value

        self.sub = self.create_subscription(TFMessage, self.input_topic, self.callback, 10)
        self.pub = self.create_publisher(PoseStamped, self.output_topic, 10)

        self.get_logger().info(f"Extracting ground truth for '{self.model_name}' from {self.input_topic} -> {self.output_topic}")

    def callback(self, msg):
        for transform in msg.transforms:
            # The child_frame_id contains the model/link name
            if transform.child_frame_id == self.model_name:
                out = PoseStamped()
                # Use timestamp from the Ignition message (simulation time)
                out.header.stamp = transform.header.stamp
                out.header.frame_id = 'world'  # Ground truth is in world frame
                out.pose.position.x = transform.transform.translation.x
                out.pose.position.y = transform.transform.translation.y
                out.pose.position.z = transform.transform.translation.z
                out.pose.orientation = transform.transform.rotation
                self.pub.publish(out)
                return  # Found the model, no need to continue

def main():
    rclpy.init()
    node = GroundTruthExtractor()
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
