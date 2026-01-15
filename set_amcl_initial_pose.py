#!/usr/bin/env python3
"""
Helper script to set AMCL initial pose automatically.
This can be used as a fallback if parameter-based initialization doesn't work.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('set_initial_pose')
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Wait a bit for the publisher to be ready
        time.sleep(1.0)
        
        # Create and publish initial pose
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set initial pose to origin (0, 0, 0) with identity orientation
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
        # Set covariance (uncertainty in pose estimate)
        # Position covariance (x, y)
        msg.pose.covariance[0] = 0.25  # x variance
        msg.pose.covariance[7] = 0.25  # y variance
        # Orientation covariance (yaw)
        msg.pose.covariance[35] = 0.06853891945200942  # yaw variance (about 15 degrees)
        
        # Publish multiple times to ensure it's received
        for i in range(3):
            self.publisher.publish(msg)
            self.get_logger().info(
                f'Published initial pose to /initialpose (attempt {i+1}/3): '
                f'x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}, '
                f'yaw=0.0'
            )
            time.sleep(0.5)
        
        self.get_logger().info('Initial pose published. Shutting down.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
