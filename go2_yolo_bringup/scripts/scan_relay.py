#!/usr/bin/env python3
"""
scan_relay.py — Subscribes to /scan with BEST_EFFORT QoS and republishes to
/scan_slam, filtering out stale scans.

FastDDS replays its writer history to new matched readers even across
BEST_EFFORT/RELIABLE QoS boundaries.  These old scans have timestamps from
before SLAM's TF buffer starts, so SLAM drops them with
"timestamp earlier than all data in the transform cache".

The staleness filter discards any scan older than MAX_AGE_S sim-seconds so
SLAM only ever receives scans whose timestamps fall within its TF buffer.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan

MAX_AGE_S = 1.0   # discard scans older than this many sim-seconds


class ScanRelay(Node):
    def __init__(self):
        super().__init__('scan_relay')

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(LaserScan, '/scan_slam', best_effort_qos)
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.relay_cb, best_effort_qos
        )
        self._dropped = 0

    def relay_cb(self, msg):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns == 0:
            return  # sim clock not yet received

        scan_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        age_s = (now_ns - scan_ns) / 1e9

        if age_s > MAX_AGE_S:
            self._dropped += 1
            if self._dropped % 20 == 1:
                self.get_logger().warn(
                    f'Dropped {self._dropped} stale scans (latest age={age_s:.2f}s)',
                    throttle_duration_sec=5.0,
                )
            return

        # Stamp to (now - 0.5s): safely behind the leading edge of the TF buffer
        # so tf2 interpolates rather than extrapolates.  0.5s is well within
        # both SLAM's 60s buffer and costmap's 10s default buffer, but large
        # enough to clear the startup window before TF data arrives.
        stamp_ns = now_ns - int(0.5e9)
        msg.header.stamp.sec = stamp_ns // 10**9
        msg.header.stamp.nanosec = stamp_ns % 10**9

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScanRelay()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
