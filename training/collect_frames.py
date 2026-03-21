#!/usr/bin/env python3
"""
collect_frames.py — Extract frames from a ROS2 bag for dataset collection.

Usage:
  # 1. Record a collection session on the robot:
  #    ros2 bag record /go2/camera/image_raw -o ~/datasets/raw_bags/session_01

  # 2. Extract frames at target FPS (default 3 fps avoids near-duplicate frames):
  #    python3 collect_frames.py \
  #        --bag ~/datasets/raw_bags/session_01 \
  #        --out ~/datasets/raw_frames/session_01 \
  #        --fps 3 \
  #        --topic /go2/camera/image_raw

  # 3. Label extracted frames with Label Studio or Roboflow, then organize into:
  #    ~/datasets/go2_perception/train|val|test/{images,labels}/

Dependencies:
  pip install rosbags opencv-python  # rosbags is pure-Python, no ROS install needed
"""

import argparse
import os
import cv2
import numpy as np
from pathlib import Path


def extract_from_bag(bag_path: str, out_dir: str, topic: str, fps: float):
    """Extract frames from a ROS2 bag using the rosbags library (no ROS required)."""
    try:
        from rosbags.rosbag2 import Reader
        from rosbags.typesys import Stores, get_typestore
    except ImportError:
        raise SystemExit(
            "Install rosbags: pip install rosbags\n"
            "  (pure-Python ROS2 bag reader — no ROS installation needed)"
        )

    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    ns_per_frame = int(1e9 / fps)

    frame_idx = 0
    last_t = 0

    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            available = [c.topic for c in reader.connections]
            raise SystemExit(
                f"Topic '{topic}' not found in bag.\nAvailable: {available}"
            )

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            if timestamp - last_t < ns_per_frame:
                continue

            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            img = _imgmsg_to_cv2(msg)
            if img is None:
                continue

            out_file = out_path / f"frame_{frame_idx:06d}.jpg"
            cv2.imwrite(str(out_file), img, [cv2.IMWRITE_JPEG_QUALITY, 95])
            frame_idx += 1
            last_t = timestamp

            if frame_idx % 50 == 0:
                print(f"  Extracted {frame_idx} frames...")

    print(f"Done. {frame_idx} frames written to {out_dir}")


def _imgmsg_to_cv2(msg) -> np.ndarray | None:
    """Convert a sensor_msgs/Image message to a cv2 BGR array."""
    encoding = msg.encoding
    h, w = msg.height, msg.width
    data = bytes(msg.data)

    if encoding in ("bgr8", "rgb8", "mono8"):
        channels = 1 if encoding == "mono8" else 3
        arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, channels)
        if encoding == "rgb8":
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif encoding == "mono8":
            arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        return arr
    elif encoding in ("16UC1", "32FC1"):
        # Depth frame — normalise for visualisation only (not used for labeling)
        dt = np.uint16 if encoding == "16UC1" else np.float32
        arr = np.frombuffer(data, dtype=dt).reshape(h, w)
        arr_norm = cv2.normalize(arr, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        return cv2.cvtColor(arr_norm, cv2.COLOR_GRAY2BGR)
    else:
        print(f"  Unsupported encoding: {encoding} — skipping frame")
        return None


def split_dataset(frames_dir: str, dataset_root: str,
                  train_frac: float = 0.8, val_frac: float = 0.1):
    """
    After labeling, call this to split labeled frames into train/val/test.
    Expects frames_dir to contain *.jpg + matching *.txt (YOLO labels).
    """
    import random
    from shutil import copy2

    frames = sorted(Path(frames_dir).glob("*.jpg"))
    labeled = [f for f in frames if (f.with_suffix(".txt")).exists()]

    if not labeled:
        print("No labeled frames found (no .txt files alongside .jpg). Label first.")
        return

    random.seed(42)
    random.shuffle(labeled)
    n = len(labeled)
    n_train = int(n * train_frac)
    n_val = int(n * val_frac)

    splits = {
        "train": labeled[:n_train],
        "val":   labeled[n_train:n_train + n_val],
        "test":  labeled[n_train + n_val:],
    }

    root = Path(dataset_root)
    for split, files in splits.items():
        (root / split / "images").mkdir(parents=True, exist_ok=True)
        (root / split / "labels").mkdir(parents=True, exist_ok=True)
        for img_path in files:
            lbl_path = img_path.with_suffix(".txt")
            copy2(img_path, root / split / "images" / img_path.name)
            copy2(lbl_path, root / split / "labels" / lbl_path.name)
        print(f"  {split}: {len(files)} images")

    print(f"Split complete: {n} labeled frames → {dataset_root}")


def main():
    parser = argparse.ArgumentParser(description="ROS2 bag frame extractor for dataset collection")
    sub = parser.add_subparsers(dest="cmd", required=True)

    # extract subcommand
    ext = sub.add_parser("extract", help="Extract frames from a ROS2 bag")
    ext.add_argument("--bag",   required=True, help="Path to ROS2 bag directory")
    ext.add_argument("--out",   required=True, help="Output directory for JPEG frames")
    ext.add_argument("--topic", default="/go2/camera/image_raw", help="Image topic")
    ext.add_argument("--fps",   type=float, default=3.0, help="Target extraction FPS (default 3)")

    # split subcommand
    spl = sub.add_parser("split", help="Split labeled frames into train/val/test")
    spl.add_argument("--frames",  required=True, help="Directory with labeled .jpg + .txt files")
    spl.add_argument("--dataset", default="/home/carebear/datasets/go2_perception",
                     help="Dataset root (contains train/val/test dirs)")
    spl.add_argument("--train-frac", type=float, default=0.8)
    spl.add_argument("--val-frac",   type=float, default=0.1)

    args = parser.parse_args()

    if args.cmd == "extract":
        extract_from_bag(args.bag, args.out, args.topic, args.fps)
    elif args.cmd == "split":
        split_dataset(args.frames, args.dataset, args.train_frac, args.val_frac)


if __name__ == "__main__":
    main()
