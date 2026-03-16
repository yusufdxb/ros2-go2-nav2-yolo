#!/usr/bin/env python3
"""
GO2 Nav2 YOLO Demo — Detector Node
Same logic as the TurtleBot3 version but with topic names
matching the GO2 Gazebo camera plugin output:
  /go2/camera/image_raw
  /go2/camera/depth/image_raw
  /go2/camera/camera_info

These are remapped in yolo_nav_launch.py so the node code
itself uses the generic /camera/* names.
"""

# This file is identical to nav2_yolo_detector/detector_node.py
# Topic remapping is handled in the launch file — no code changes needed.
# Kept here for clarity / completeness of the package.

import numpy as np
import cv2
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from go2_yolo_msgs.msg import DetectedObject, DetectedObjectArray


COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
]

PALETTE = [
    (0, 255, 80), (255, 80, 0), (0, 120, 255), (255, 220, 0),
    (180, 0, 255), (0, 255, 220), (255, 0, 120), (80, 255, 0),
]


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector_node")

        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.45)
        self.declare_parameter("max_depth_m", 10.0)
        self.declare_parameter("depth_sample_fraction", 0.3)
        self.declare_parameter("publish_visualization", True)

        model_path = self.get_parameter("model_path").value
        self.conf_thresh = self.get_parameter("confidence_threshold").value
        self.max_depth = self.get_parameter("max_depth_m").value
        self.depth_frac = self.get_parameter("depth_sample_fraction").value
        self.publish_vis = self.get_parameter("publish_visualization").value

        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Topics remapped in launch file to /go2/camera/* 
        self.rgb_sub = self.create_subscription(Image, "/camera/image_raw", self.rgb_callback, qos)
        self.depth_sub = self.create_subscription(Image, "/camera/depth/image_raw", self.depth_callback, qos)
        self.info_sub = self.create_subscription(CameraInfo, "/camera/camera_info", self.info_callback, 10)

        self.detections_pub = self.create_publisher(DetectedObjectArray, "/detected_objects", 10)
        self.vis_pub = self.create_publisher(Image, "/detector/visualization", 10)

        self.latest_depth = None
        self.get_logger().info("GO2 DetectorNode ready.")

    def info_callback(self, msg: CameraInfo):
        if self.fx is None:
            self.fx = msg.k[0]; self.fy = msg.k[4]
            self.cx = msg.k[2]; self.cy = msg.k[5]
            self.get_logger().info(f"Camera info loaded: fx={self.fx:.1f}")

    def depth_callback(self, msg: Image):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def rgb_callback(self, msg: Image):
        if self.fx is None or self.latest_depth is None:
            return

        rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model(rgb, verbose=False)[0]

        out = DetectedObjectArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "camera_color_optical_frame"

        depth = self.latest_depth.copy()
        vis = rgb.copy() if self.publish_vis else None

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            if conf < self.conf_thresh:
                continue

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx_bb, cy_bb = (x1 + x2) // 2, (y1 + y2) // 2

            roi_w = max(1, int((x2 - x1) * self.depth_frac))
            roi_h = max(1, int((y2 - y1) * self.depth_frac))
            rx1 = max(0, cx_bb - roi_w // 2); rx2 = min(depth.shape[1], cx_bb + roi_w // 2)
            ry1 = max(0, cy_bb - roi_h // 2); ry2 = min(depth.shape[0], cy_bb + roi_h // 2)

            roi_depth = depth[ry1:ry2, rx1:rx2].astype(np.float32)
            valid = roi_depth[(roi_depth > 0) & (roi_depth < self.max_depth * 1000)]
            if valid.size == 0:
                continue

            depth_m = float(np.median(valid)) / 1000.0
            X = (cx_bb - self.cx) * depth_m / self.fx
            Y = (cy_bb - self.cy) * depth_m / self.fy
            Z = depth_m

            label = COCO_NAMES[cls_id] if cls_id < len(COCO_NAMES) else str(cls_id)

            obj = DetectedObject()
            obj.class_name = label; obj.class_id = cls_id; obj.confidence = conf
            obj.position.x = X; obj.position.y = Y; obj.position.z = Z
            obj.bbox_x1 = x1; obj.bbox_y1 = y1; obj.bbox_x2 = x2; obj.bbox_y2 = y2
            out.objects.append(obj)

            if vis is not None:
                color = PALETTE[cls_id % len(PALETTE)]
                cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
                cv2.putText(vis, f"{label} {conf:.2f} | {depth_m:.1f}m",
                            (x1, max(y1 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        self.detections_pub.publish(out)
        if vis is not None:
            vis_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            vis_msg.header = out.header
            self.vis_pub.publish(vis_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
