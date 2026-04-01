#!/usr/bin/env python3
"""
auto_label.py — Live auto-labeler for Gazebo simulation.

Subscribes to the robot camera + ground truth odometry and projects known
world-frame object bounding boxes into 2D YOLO labels.

Usage:
  # Terminal 1: Gazebo sim running (gazebo_launch.py)
  # Terminal 2: Drive robot  (record_session.py or teleop)
  # Terminal 3: Run this script
  python3 training/auto_label.py --out ~/datasets/auto_labeled/session_01 --fps 3

Can also label existing frames from a bag (if ground truth was recorded):
  python3 training/auto_label.py bag \
      --bag ~/datasets/raw_bags/session_... \
      --out ~/datasets/auto_labeled/session_01

Outputs:
  <out>/images/frame_NNNNNN.jpg
  <out>/labels/frame_NNNNNN.txt   (YOLO: class x_c y_c w h)
"""

import argparse
import os
import time
import threading
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


# ── Object definitions (world frame) ────────────────────────────────────────

@dataclass
class WorldObject:
    name: str
    class_id: int
    x: float          # world X
    y: float          # world Y
    z_center: float   # center height of 3D bbox
    dx: float         # half-extent X
    dy: float         # half-extent Y
    dz: float         # half-extent Z


# Class mapping: 0=person, 1=cone, 2=coke_can, 3=box
OBJECTS = [
    # person_standing: mesh ~1.8m tall, ~0.5m wide, 0.35m deep. Origin at feet.
    WorldObject("person",   0, 2.0,  0.0, 0.90,  0.25, 0.20, 0.90),
    # construction_cone: scaled 10x mesh, ~0.35m tall, ~0.2m base diameter
    WorldObject("cone",     1, 1.5,  1.5, 0.175, 0.15, 0.15, 0.175),
    # coke_can: ~0.12m tall, ~0.033m radius
    WorldObject("coke_can", 2, 2.5, -1.0, 0.06,  0.035, 0.035, 0.06),
    # cardboard_box: 0.5 x 0.4 x 0.3 m, internal z offset 0.15
    WorldObject("box",      3, 0.0,  2.0, 0.15,  0.25, 0.20, 0.15),
]

CLASS_NAMES = ["person", "cone", "coke_can", "box"]

# ── Camera intrinsics (from CameraInfo) ─────────────────────────────────────
DEFAULT_FX = 554.38271282
DEFAULT_FY = 554.38271282
DEFAULT_CX = 320.5
DEFAULT_CY = 240.5
IMG_W = 640
IMG_H = 480


# ── Geometry helpers ────────────────────────────────────────────────────────

def quat_to_rot(qx, qy, qz, qw):
    """Quaternion (x, y, z, w) → 3x3 rotation matrix."""
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz),  2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),       1 - 2*(qx*qx + qy*qy)],
    ])


def build_base_to_optical():
    """Precompute 4x4 transform: base_link → camera_color_optical_frame.

    TF convention: a published transform (parent→child) maps points FROM
    child TO parent.  We need the *inverse* direction for projection:
    points from base_link (≡ trunk) into camera_color_optical_frame.

    Chain: base_link ≡ trunk → camera_link → camera_color_optical_frame
    """
    # TF trunk→camera_link: p_trunk = R1 @ p_camera_link + t1
    R1 = quat_to_rot(0.0, 0.0749, 0.0, 0.9972)
    t1 = np.array([0.28, 0.0, 0.12])

    # TF camera_link→optical: p_camera_link = R2 @ p_optical  (t2=0)
    R2 = quat_to_rot(-0.5, 0.5, -0.5, 0.5)

    # Combined child→parent: p_trunk = R1 @ (R2 @ p_optical) + t1
    # Invert to get parent→child (base_link → optical):
    #   p_optical = (R1@R2)^T @ (p_trunk - t1)
    R_fwd = (R1 @ R2).T
    t_fwd = -R_fwd @ t1

    T = np.eye(4)
    T[:3, :3] = R_fwd
    T[:3, 3] = t_fwd
    return T


BASE_TO_OPTICAL = build_base_to_optical()


def odom_msg_to_T(odom_msg):
    """Odometry message → 4x4 pose matrix (world → base_link)."""
    p = odom_msg.pose.pose.position
    o = odom_msg.pose.pose.orientation
    R = quat_to_rot(o.x, o.y, o.z, o.w)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def pose_to_T(tx, ty, tz, qx, qy, qz, qw):
    """Position + quaternion → 4x4 matrix."""
    R = quat_to_rot(qx, qy, qz, qw)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T


def project_object(obj: WorldObject, T_base_in_world: np.ndarray,
                   fx: float, fy: float, cx: float, cy: float,
                   depth_img: np.ndarray = None):
    """Project 3D bbox corners → 2D YOLO label. Returns tuple or None.

    If depth_img is provided, verifies the object is actually visible
    (not occluded by walls) by comparing expected vs measured depth.
    """
    # 8 corners of the 3D bbox in world frame
    corners = np.array([
        [obj.x + sx * obj.dx,
         obj.y + sy * obj.dy,
         obj.z_center + sz * obj.dz]
        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)
    ])

    # world → camera optical = base_to_optical @ inv(T_base_in_world) @ world
    T_world_to_optical = BASE_TO_OPTICAL @ np.linalg.inv(T_base_in_world)

    corners_h = np.hstack([corners, np.ones((8, 1))])
    corners_cam = (T_world_to_optical @ corners_h.T).T[:, :3]

    # All behind camera?
    if np.all(corners_cam[:, 2] <= 0.1):
        return None

    # Project each corner
    pts_2d = np.zeros((8, 2))
    for i in range(8):
        z = max(corners_cam[i, 2], 0.1)
        pts_2d[i, 0] = fx * corners_cam[i, 0] / z + cx
        pts_2d[i, 1] = fy * corners_cam[i, 1] / z + cy

    x_min = np.clip(np.min(pts_2d[:, 0]), 0, IMG_W)
    x_max = np.clip(np.max(pts_2d[:, 0]), 0, IMG_W)
    y_min = np.clip(np.min(pts_2d[:, 1]), 0, IMG_H)
    y_max = np.clip(np.max(pts_2d[:, 1]), 0, IMG_H)

    bw = x_max - x_min
    bh = y_max - y_min

    # Reject tiny, huge, or far-away detections
    if bw < 5 or bh < 5:
        return None
    if bw > IMG_W * 0.95 and bh > IMG_H * 0.95:
        return None

    obj_center_cam = T_world_to_optical @ np.array([obj.x, obj.y, obj.z_center, 1.0])
    expected_depth = obj_center_cam[2]
    if expected_depth > 8.0 or expected_depth < 0.3:
        return None

    # ── Depth-based occlusion check ─────────────────────────────────────
    if depth_img is not None:
        center_u = int((x_min + x_max) / 2)
        center_v = int((y_min + y_max) / 2)
        # Sample a small patch around the projected center
        r = 3
        u0 = max(0, center_u - r)
        u1 = min(IMG_W, center_u + r + 1)
        v0 = max(0, center_v - r)
        v1 = min(IMG_H, center_v + r + 1)
        patch = depth_img[v0:v1, u0:u1]
        valid = patch[patch > 0.01]  # ignore zero/invalid depth
        if len(valid) > 0:
            measured_depth = np.median(valid)
            # Object is occluded if the measured depth is significantly
            # shorter than expected (wall/obstacle is in the way).
            # Allow 30% tolerance for bbox edge effects.
            if measured_depth < expected_depth * 0.7:
                return None

    return (
        obj.class_id,
        (x_min + x_max) / 2.0 / IMG_W,
        (y_min + y_max) / 2.0 / IMG_H,
        bw / IMG_W,
        bh / IMG_H,
    )


# ═══════════════════════════════════════════════════════════════════════════
# Mode 1: Live collection from running Gazebo sim
# ═══════════════════════════════════════════════════════════════════════════

def run_live(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from nav_msgs.msg import Odometry

    class AutoLabeler(Node):
        def __init__(self):
            super().__init__("auto_labeler")
            self.out = Path(args.out)
            (self.out / "images").mkdir(parents=True, exist_ok=True)
            (self.out / "labels").mkdir(parents=True, exist_ok=True)

            self.target_period = 1.0 / args.fps
            self.last_save = 0.0
            self.idx = 0
            self.fx, self.fy, self.cx, self.cy = DEFAULT_FX, DEFAULT_FY, DEFAULT_CX, DEFAULT_CY
            self.latest_odom = None
            self.latest_depth = None
            self.lock = threading.Lock()
            self.counts = {n: 0 for n in CLASS_NAMES}
            self.occluded_count = 0

            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(CameraInfo, "/go2/camera/camera_info",
                                     self._cam_info_cb, 10)
            self.create_subscription(Odometry, "/odom/ground_truth",
                                     self._odom_cb, qos)
            self.create_subscription(Image, "/go2/camera/image_raw",
                                     self._img_cb, qos)
            self.create_subscription(Image, "/go2/camera/depth/image_raw",
                                     self._depth_cb, qos)
            self.get_logger().info(f"Auto-labeler ready → {self.out} @ {args.fps} FPS")

        def _cam_info_cb(self, msg):
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]

        def _odom_cb(self, msg):
            with self.lock:
                self.latest_odom = msg

        def _depth_cb(self, msg):
            depth = _decode_depth_image(msg)
            if depth is not None:
                with self.lock:
                    self.latest_depth = depth

        def _img_cb(self, msg):
            now = time.monotonic()
            if now - self.last_save < self.target_period:
                return
            with self.lock:
                odom = self.latest_odom
                depth = self.latest_depth
            if odom is None:
                return

            img = _decode_ros_image(msg)
            if img is None:
                return

            T = odom_msg_to_T(odom)
            labels = []
            for obj in OBJECTS:
                r = project_object(obj, T, self.fx, self.fy, self.cx, self.cy,
                                   depth_img=depth)
                if r is not None:
                    labels.append(r)
                    self.counts[CLASS_NAMES[r[0]]] += 1

            name = f"frame_{self.idx:06d}"
            cv2.imwrite(
                str(self.out / "images" / f"{name}.jpg"),
                img, [cv2.IMWRITE_JPEG_QUALITY, 95]
            )
            with open(self.out / "labels" / f"{name}.txt", "w") as f:
                for cid, xc, yc, w, h in labels:
                    f.write(f"{cid} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}\n")

            self.idx += 1
            self.last_save = now

            if self.idx % 20 == 0:
                self.get_logger().info(
                    f"Frames: {self.idx} | Labels: {dict(self.counts)} | "
                    f"This: {len(labels)} objs"
                )

        def summary(self):
            self.get_logger().info(
                f"\nDone! {self.idx} frames, labels: {dict(self.counts)}\n"
                f"Output: {self.out}"
            )

    rclpy.init()
    node = AutoLabeler()
    try:
        if args.duration > 0:
            end = time.monotonic() + args.duration
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        rclpy.shutdown()


# ═══════════════════════════════════════════════════════════════════════════
# Mode 2: Label from a recorded bag (requires ground truth odom in bag)
# ═══════════════════════════════════════════════════════════════════════════

def run_bag(args):
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore

    store = get_typestore(Stores.ROS2_HUMBLE)
    out = Path(args.out)
    (out / "images").mkdir(parents=True, exist_ok=True)
    (out / "labels").mkdir(parents=True, exist_ok=True)

    ns_per_frame = int(1e9 / args.fps)
    fx, fy, cx, cy = DEFAULT_FX, DEFAULT_FY, DEFAULT_CX, DEFAULT_CY
    counts = {n: 0 for n in CLASS_NAMES}

    # First pass: collect all ground truth poses
    print("Pass 1: Reading ground truth odometry...")
    gt_poses = []  # (timestamp, T_4x4)
    with Reader(args.bag) as reader:
        gt_conns = [c for c in reader.connections if c.topic == args.odom_topic]
        if not gt_conns:
            avail = [c.topic for c in reader.connections]
            raise SystemExit(
                f"Topic '{args.odom_topic}' not in bag.\nAvailable: {avail}"
            )
        for conn, ts, raw in reader.messages(connections=gt_conns):
            msg = store.deserialize_cdr(raw, conn.msgtype)
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            T = pose_to_T(p.x, p.y, p.z, o.x, o.y, o.z, o.w)
            gt_poses.append((ts, T))

    print(f"  {len(gt_poses)} ground truth poses")
    if not gt_poses:
        raise SystemExit("No ground truth poses found!")

    # Check if poses are all zero
    all_zero = all(
        abs(gt_poses[i][1][0, 3]) < 0.001 and abs(gt_poses[i][1][1, 3]) < 0.001
        for i in range(min(10, len(gt_poses)))
    )
    if all_zero:
        raise SystemExit(
            "Ground truth poses are all at origin! "
            "The bag doesn't contain real odometry. Use 'live' mode instead."
        )

    gt_timestamps = np.array([p[0] for p in gt_poses])

    def nearest_pose(ts):
        idx = np.searchsorted(gt_timestamps, ts)
        idx = min(idx, len(gt_poses) - 1)
        return gt_poses[idx][1]

    # Second pass: extract frames + label
    print("Pass 2: Extracting frames and labeling...")
    frame_idx = 0
    last_t = 0

    with Reader(args.bag) as reader:
        # Also grab camera info if available
        ci_conns = [c for c in reader.connections if c.topic == "/go2/camera/camera_info"]
        if ci_conns:
            for conn, ts, raw in reader.messages(connections=ci_conns):
                msg = store.deserialize_cdr(raw, conn.msgtype)
                fx, fy = msg.k[0], msg.k[4]
                cx, cy = msg.k[2], msg.k[5]
                break

    with Reader(args.bag) as reader:
        img_conns = [c for c in reader.connections
                     if c.topic == "/go2/camera/image_raw"]
        for conn, ts, raw in reader.messages(connections=img_conns):
            if ts - last_t < ns_per_frame:
                continue

            msg = store.deserialize_cdr(raw, conn.msgtype)
            img = _bag_imgmsg_to_cv2(msg)
            if img is None:
                continue

            T = nearest_pose(ts)
            labels = []
            for obj in OBJECTS:
                r = project_object(obj, T, fx, fy, cx, cy)
                if r is not None:
                    labels.append(r)
                    counts[CLASS_NAMES[r[0]]] += 1

            name = f"frame_{frame_idx:06d}"
            cv2.imwrite(
                str(out / "images" / f"{name}.jpg"),
                img, [cv2.IMWRITE_JPEG_QUALITY, 95]
            )
            with open(out / "labels" / f"{name}.txt", "w") as f:
                for cid, xc, yc, w, h in labels:
                    f.write(f"{cid} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}\n")

            frame_idx += 1
            last_t = ts

            if frame_idx % 50 == 0:
                print(f"  {frame_idx} frames...")

    print(f"\nDone! {frame_idx} frames, labels: {dict(counts)}")
    print(f"Output: {out}")


# ── Image decoders ──────────────────────────────────────────────────────────

def _decode_ros_image(msg) -> np.ndarray | None:
    """Decode a live ROS Image message."""
    h, w = msg.height, msg.width
    data = bytes(msg.data)
    if msg.encoding == "rgb8":
        return cv2.cvtColor(
            np.frombuffer(data, np.uint8).reshape(h, w, 3), cv2.COLOR_RGB2BGR
        )
    elif msg.encoding == "bgr8":
        return np.frombuffer(data, np.uint8).reshape(h, w, 3)
    return None


def _decode_depth_image(msg) -> np.ndarray | None:
    """Decode a depth image to float32 array (meters)."""
    h, w = msg.height, msg.width
    data = bytes(msg.data)
    if msg.encoding == "32FC1":
        return np.frombuffer(data, dtype=np.float32).reshape(h, w)
    elif msg.encoding == "16UC1":
        raw = np.frombuffer(data, dtype=np.uint16).reshape(h, w)
        return raw.astype(np.float32) / 1000.0  # mm → m
    return None


def _bag_imgmsg_to_cv2(msg) -> np.ndarray | None:
    """Decode a rosbags deserialized Image message."""
    h, w = msg.height, msg.width
    data = bytes(msg.data)
    if msg.encoding in ("rgb8", "bgr8"):
        arr = np.frombuffer(data, np.uint8).reshape(h, w, 3)
        if msg.encoding == "rgb8":
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        return arr
    return None


# ═══════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Gazebo auto-labeler: project 3D object positions → 2D YOLO labels"
    )
    sub = parser.add_subparsers(dest="mode")

    # Live mode (default when no subcommand)
    live = sub.add_parser("live", help="Collect from running Gazebo sim")
    live.add_argument("--out", default=os.path.expanduser(
        "~/datasets/auto_labeled/session_01"))
    live.add_argument("--fps", type=float, default=3.0)
    live.add_argument("--duration", type=float, default=0.0,
                      help="Auto-stop after N seconds (0 = Ctrl+C)")

    # Bag mode
    bag = sub.add_parser("bag", help="Label from recorded bag with ground truth")
    bag.add_argument("--bag", required=True, help="Path to ROS2 bag")
    bag.add_argument("--out", required=True, help="Output directory")
    bag.add_argument("--fps", type=float, default=3.0)
    bag.add_argument("--odom-topic", default="/odom/ground_truth")

    args = parser.parse_args()

    if args.mode == "bag":
        run_bag(args)
    else:
        # Default to live mode
        if not hasattr(args, 'out'):
            args.out = os.path.expanduser("~/datasets/auto_labeled/session_01")
        if not hasattr(args, 'fps'):
            args.fps = 3.0
        if not hasattr(args, 'duration'):
            args.duration = 0.0
        run_live(args)


if __name__ == "__main__":
    main()
