#!/usr/bin/env python3
"""
Drive the GO2 around objects in demo_world while recording camera feed.

Usage:
  # Terminal 1: launch gazebo (gazebo_launch.py already running)
  # Terminal 2: run this script
  python3 training/record_session.py

Records to ~/datasets/raw_bags/session_YYYYMMDD_HHMM/
"""

import subprocess
import sys
import time
import math
import signal
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# Waypoints that circle around the objects in demo_world:
#   person_standing at (2.0, 0.0)
#   construction_cone at (1.5, 1.5)
#   coke_can at (2.5, -1.0)
#   wooden_box at (0.0, 2.0)
#
# Drive patterns: approach each object from different angles
DRIVE_COMMANDS = [
    # (linear_x, angular_z, duration_secs, description)
    (0.3, 0.0, 3.0, "forward toward person"),
    (0.0, 0.5, 2.0, "turn left"),
    (0.3, 0.0, 3.0, "forward toward cone"),
    (0.0, -0.4, 2.5, "turn right"),
    (0.3, 0.0, 4.0, "forward toward coke_can"),
    (0.0, 0.6, 3.0, "turn left wide"),
    (0.3, 0.0, 3.0, "forward toward wooden_box"),
    (0.0, -0.5, 2.0, "turn right"),
    (0.3, 0.1, 4.0, "arc around objects"),
    (0.0, 0.5, 3.0, "turn left"),
    (0.3, -0.1, 4.0, "arc other direction"),
    (-0.2, 0.0, 2.0, "back up"),
    (0.0, 0.8, 4.0, "spin for coverage"),
    (0.3, 0.0, 3.0, "forward again"),
    (0.0, -0.6, 3.0, "turn right wide"),
    (0.3, 0.2, 5.0, "arc toward person from side"),
    (0.0, 0.0, 2.0, "pause"),
    (0.0, -0.8, 4.0, "spin other direction"),
    (0.3, 0.0, 3.0, "forward pass"),
    (0.3, -0.15, 5.0, "wide arc"),
]


class DriveRecorder(Node):
    def __init__(self):
        super().__init__("drive_recorder")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info("DriveRecorder ready")

    def drive(self, linear_x: float, angular_z: float, duration: float, desc: str):
        self.get_logger().info(f"  {desc}: lin={linear_x:.1f} ang={angular_z:.1f} for {duration:.0f}s")
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        start = time.time()
        while time.time() - start < duration:
            self.pub.publish(msg)
            time.sleep(0.1)  # wall-clock sleep, avoids sim_time rate issues

        # Brief stop between commands
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)
        time.sleep(0.3)


def main():
    rclpy.init()
    node = DriveRecorder()

    stamp = datetime.now().strftime("%Y%m%d_%H%M")
    bag_path = f"{sys.path[0]}/../../../datasets/raw_bags/session_{stamp}"
    bag_path_resolved = str(
        __import__("pathlib").Path(bag_path).expanduser().resolve()
    )
    # Use home dir directly
    bag_path_resolved = f"/home/yusuf/datasets/raw_bags/session_{stamp}"

    topics = [
        "/go2/camera/image_raw",
        "/go2/camera/depth/image_raw",
        "/go2/camera/camera_info",
        "/tf",
        "/tf_static",
        "/odom",
        "/clock",
    ]

    node.get_logger().info(f"Starting rosbag recording to {bag_path_resolved}")
    record_cmd = [
        "ros2", "bag", "record",
        "-o", bag_path_resolved,
        "--use-sim-time",
    ] + topics

    bag_proc = subprocess.Popen(record_cmd)
    time.sleep(2)  # let recording stabilize

    try:
        node.get_logger().info(f"Executing {len(DRIVE_COMMANDS)} drive commands...")
        for i, (lx, az, dur, desc) in enumerate(DRIVE_COMMANDS):
            node.get_logger().info(f"[{i+1}/{len(DRIVE_COMMANDS)}]")
            node.drive(lx, az, dur, desc)

        node.get_logger().info("Drive sequence complete. Recording for 3 more seconds...")
        time.sleep(3)

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        # Stop robot
        msg = Twist()
        node.pub.publish(msg)

        # Stop recording
        bag_proc.send_signal(signal.SIGINT)
        bag_proc.wait(timeout=10)
        node.get_logger().info(f"Recording saved to {bag_path_resolved}")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
