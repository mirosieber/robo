#!/usr/bin/env python3
import math
import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class CircleDriver(Node):
    def __init__(self, speed, radius):
        super().__init__('circle_driver')
        self.speed = speed
        self.radius = radius
        
        # Calculate constant steering angle for circular motion
        # Using bicycle model: radius = wheelbase / tan(steer_angle)
        self.wheelbase = 0.4  # meters (distance between front and rear axles)
        self.steer_angle = math.atan(self.wheelbase / self.radius)
        
        qos = rclpy.qos.qos_profile_system_default
        self.drive_pub = self.create_publisher(Float64MultiArray, '/drive_controller/commands', qos)
        self.steer_pub = self.create_publisher(Float64MultiArray, '/steer_controller/commands', qos)

        # Publish at 50 Hz for smooth control
        self.timer = self.create_timer(0.02, self._on_timer)
        
        self.get_logger().info(f'Circle driver: radius={radius}m, speed={speed}, steer_angle={self.steer_angle:.3f}rad')

    def _on_timer(self):
        drive_msg = Float64MultiArray(data=[self.speed])
        steer_msg = Float64MultiArray(data=[self.steer_angle])

        self.drive_pub.publish(drive_msg)
        self.steer_pub.publish(steer_msg)

    def stop(self):
        # Ensure robot is commanded to stop
        stop_drive = Float64MultiArray(data=[0.0])
        stop_steer = Float64MultiArray(data=[0.0])
        self.drive_pub.publish(stop_drive)
        self.steer_pub.publish(stop_steer)


def main():
    parser = argparse.ArgumentParser(description='Drive the robot in a circle of specified radius.')
    parser.add_argument('--speed', type=float, default=1.0, help='Forward velocity command for drive joint')
    parser.add_argument('--radius', type=float, default=2.0, help='Circle radius in meters')
    parser.add_argument('--duration', type=float, default=None, help='Optional duration to run before stopping (seconds)')
    args = parser.parse_args()

    rclpy.init()
    node = CircleDriver(args.speed, args.radius)

    try:
        if args.duration is None:
            rclpy.spin(node)
        else:
            end_time = time.time() + args.duration
            while rclpy.ok() and time.time() < end_time:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
