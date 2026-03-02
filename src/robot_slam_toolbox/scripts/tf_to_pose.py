#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
import sys

class TfToPose(Node):
    def __init__(self):
        super().__init__('tf_to_pose')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', 'map')
        self.declare_parameter('output_topic', 'estimated_pose')
        
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        topic_name = self.get_parameter('output_topic').get_parameter_value().string_value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.publisher = self.create_publisher(PoseStamped, topic_name, 10)
        self.timer = self.create_timer(0.1, self.on_timer) # 10Hz
        
        self.get_logger().info(f"Publishing {self.source_frame} -> {self.target_frame} to {topic_name}")

    def on_timer(self):
        try:
            # Look up transform
            t = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time())
            
            pose = PoseStamped()
            pose.header.stamp = t.header.stamp
            pose.header.frame_id = self.source_frame
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = t.transform.translation.z
            pose.pose.orientation = t.transform.rotation
            
            self.publisher.publish(pose)
            
        except Exception as e:
            # self.get_logger().warn(f"Could not transform: {e}")
            pass

def main():
    rclpy.init()
    node = TfToPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
