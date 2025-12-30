#!/usr/bin/env python3
"""Converts odometry to a path for visualization in RViz."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        self.path = Path()
        self.path.header.frame_id = 'odom'
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.path_pub = self.create_publisher(Path, '/robot_path', 10)
        
        self.get_logger().info('Odometry to path converter started')
    
    def _odom_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        
        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(pose)
        
        # Limit path length to prevent memory issues
        if len(self.path.poses) > 10000:
            self.path.poses.pop(0)
        
        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = OdomToPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
