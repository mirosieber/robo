#!/usr/bin/env python3
import math
import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class FigureEightDriver(Node):
    def __init__(self, speed, steer_amplitude, period):
        super().__init__('figure_eight_driver')
        self.speed = speed
        self.steer_amplitude = steer_amplitude
        self.omega = 2.0 * math.pi / period
        self.start_time = time.time()

        qos = rclpy.qos.qos_profile_system_default
        self.drive_pub = self.create_publisher(Float64MultiArray, '/drive_controller/commands', qos)
        self.steer_pub = self.create_publisher(Float64MultiArray, '/steer_controller/commands', qos)

        # Publish at 50 Hz for smooth curves
        self.timer = self.create_timer(0.02, self._on_timer)

    def _on_timer(self):
        t = time.time() - self.start_time
        steer_cmd = self.steer_amplitude * math.sin(self.omega * t)

        drive_msg = Float64MultiArray(data=[self.speed])
        steer_msg = Float64MultiArray(data=[steer_cmd])

        self.drive_pub.publish(drive_msg)
        self.steer_pub.publish(steer_msg)

    def stop(self):
        # Ensure robot is commanded to stop
        stop_drive = Float64MultiArray(data=[0.0])
        stop_steer = Float64MultiArray(data=[0.0])
        self.drive_pub.publish(stop_drive)
        self.steer_pub.publish(stop_steer)


def main():
    parser = argparse.ArgumentParser(description='Drive the robot in a smooth figure-eight pattern.')
    parser.add_argument('--speed', type=float, default=1.0, help='Forward velocity command for drive joint')
    parser.add_argument('--steer-amplitude', type=float, default=0.35, help='Steering amplitude in radians')
    parser.add_argument('--period', type=float, default=8.0, help='Seconds per full steering cycle')
    parser.add_argument('--duration', type=float, default=None, help='Optional duration to run before stopping (seconds)')
    args = parser.parse_args()

    rclpy.init()
    node = FigureEightDriver(args.speed, args.steer_amplitude, args.period)

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
