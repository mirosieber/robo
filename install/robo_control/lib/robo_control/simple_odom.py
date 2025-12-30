#!/usr/bin/env python3
"""
Simple odometry node for a front-steered rear-drive robot.
Subscribes to /joint_states, computes odometry using a bicycle model, 
and publishes odom->base_link TF and nav_msgs/Odometry.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class SimpleOdomNode(Node):
    def __init__(self):
        super().__init__('simple_odom')
        
        # Robot parameters
        self.wheelbase = 0.4  # distance between front and rear axles (m)
        self.wheel_radius = 0.1  # drive wheel radius (m)
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_drive_pos = None
        self.last_time = None
        
        # Subscribers
        self.js_sub = self.create_subscription(JointState, '/joint_states', self._js_callback, 10)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Simple odometry node started')
    
    def _js_callback(self, msg: JointState):
        now = self.get_clock().now()
        
        try:
            drive_idx = msg.name.index('drive_joint')
            steer_idx = msg.name.index('steer_joint')
        except ValueError:
            return
        
        drive_pos = msg.position[drive_idx]
        steer_angle = msg.position[steer_idx]
        
        if self.last_drive_pos is None or self.last_time is None:
            self.last_drive_pos = drive_pos
            self.last_time = now
            return
        
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        
        # Compute distance traveled by rear wheels
        d_drive = drive_pos - self.last_drive_pos
        distance = d_drive * self.wheel_radius
        
        # Bicycle model kinematics
        if abs(steer_angle) < 1e-6:
            # Straight motion
            dx = distance * math.cos(self.theta)
            dy = distance * math.sin(self.theta)
            dtheta = 0.0
        else:
            # Arc motion
            turn_radius = self.wheelbase / math.tan(steer_angle)
            dtheta = distance / turn_radius
            dx = turn_radius * (math.sin(self.theta + dtheta) - math.sin(self.theta))
            dy = turn_radius * (-math.cos(self.theta + dtheta) + math.cos(self.theta))
        
        self.x += dx
        self.y += dy
        self.theta += dtheta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # normalize
        
        # Estimate linear and angular velocity
        v_linear = distance / dt if dt > 0 else 0.0
        v_angular = dtheta / dt if dt > 0 else 0.0
        
        # Publish TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q_w = math.cos(self.theta / 2.0)
        q_z = math.sin(self.theta / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = q_z
        t.transform.rotation.w = q_w
        
        self.tf_broadcaster.sendTransform(t)
        
        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w
        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.angular.z = v_angular
        
        self.odom_pub.publish(odom)
        
        self.last_drive_pos = drive_pos
        self.last_time = now


def main():
    rclpy.init()
    node = SimpleOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
