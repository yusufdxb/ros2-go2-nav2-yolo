#!/usr/bin/env python3
"""
GO2 Custom Perception Node — detector_node.py
==============================================
Full pipeline:
  Camera RGB + Depth → YOLOv8 (custom model) → BoT-SORT tracker
  → EMA position smoother → Target Lock state machine
  → /detected_objects (DetectedObjectArray)

Custom classes:
  0  owner_person   — target human to follow
  1  wrist_marker   — AprilTag36h11 / ArUco on wrist
  2  phone_marker   — handheld phone/tablet
  3  follow_marker  — colored vest / paddle / hat

Target Lock state machine:
  WAITING       No target seen; accept first qualifying detection
  TRACKING      Target seen; waiting for hit-count gate to confirm
  LOCKED        Target locked to a specific track_id
  SEARCHING     Locked track lost; sweeping for reacquisition
  REACQUIRING   Candidate found after search; confirming before relocking

Topics consumed:
  /camera/image_raw          — remapped from /go2/camera/image_raw in launch
  /camera/depth/image_raw    — remapped similarly
  /camera/camera_info

Topics published:
  /detected_objects           — DetectedObjectArray (all passing detections)
  /detector/target            — DetectedObjectArray (locked target only)
  /detector/state             — std_msgs/String (state machine status)
  /detector/visualization     — Image (annotated RGB, optional)

ROS2 parameters:
  model_path              str    Path to .pt/.torchscript/.onnx weights
  confidence_threshold    float  Minimum detection confidence [0.45]
  max_depth_m             float  Maximum valid depth reading [10.0]
  depth_sample_fraction   float  Fraction of bbox used for median depth [0.3]
  publish_visualization   bool   Publish annotated image [true]
  use_tracker             bool   Enable BoT-SORT tracking [true]
  tracker_config          str    Tracker config file [botsort.yaml]
  ema_alpha               float  EMA smoothing factor [0.4]
  min_track_hits          int    Frames before announcing new track [3]
  target_class            str    Class name to lock onto [owner_person]
  lock_conf_threshold     float  Minimum confidence to initiate lock [0.6]
  reacquire_hits          int    Frames of stable new track before relocking [5]
  search_timeout_s        float  Seconds before SEARCHING → WAITING [12.0]
  half_precision          bool   FP16 inference (RTX GPU only) [false]
"""

from __future__ import annotations

import collections
import time
from enum import Enum

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from go2_yolo_msgs.msg import DetectedObject, DetectedObjectArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from ultralytics import YOLO

# ── Palette ───────────────────────────────────────────────────────────────────

PALETTE = {
    0: (0,   255,  80),   # owner_person  — green
    1: (255,  80,   0),   # wrist_marker  — orange
    2: (0,   120, 255),   # phone_marker  — blue
    3: (255, 220,   0),   # follow_marker — yellow
}
LOCK_COLOR  = (0, 255, 255)   # cyan border for locked target
STATE_COLOR = (200, 200, 200) # grey text for state overlay


# ── State machine ─────────────────────────────────────────────────────────────

class LockState(str, Enum):
    WAITING      = "WAITING"
    TRACKING     = "TRACKING"
    LOCKED       = "LOCKED"
    SEARCHING    = "SEARCHING"
    REACQUIRING  = "REACQUIRING"


# ── EMA position smoother ─────────────────────────────────────────────────────

class PositionSmoother:
    """Per-track exponential moving average smoother for 3D positions."""

    def __init__(self, alpha: float = 0.4):
        self.alpha = alpha
        self._state: dict[int, np.ndarray] = {}

    def update(self, key: int, xyz: np.ndarray) -> np.ndarray:
        if key not in self._state:
            self._state[key] = xyz.copy()
        else:
            self._state[key] = self.alpha * xyz + (1.0 - self.alpha) * self._state[key]
        return self._state[key]

    def last(self, key: int) -> np.ndarray | None:
        return self._state.get(key)

    def evict(self, active_keys: set[int]):
        for k in list(self._state):
            if k not in active_keys:
                del self._state[k]


# ── Target lock manager ───────────────────────────────────────────────────────

class TargetLock:
    """
    Manages the locking state machine for a single target class.

    State transitions:
      WAITING      → TRACKING     : first high-conf detection of target class
      TRACKING     → LOCKED       : hit count reaches min_track_hits
      LOCKED       → SEARCHING    : locked track_id disappears from frame
      SEARCHING    → REACQUIRING  : any target-class detection reappears
      SEARCHING    → WAITING      : search_timeout_s expires
      REACQUIRING  → LOCKED       : candidate stable for reacquire_hits frames
      REACQUIRING  → SEARCHING    : candidate disappears before confirmation
    """

    def __init__(
        self,
        min_track_hits: int   = 3,
        reacquire_hits: int   = 5,
        search_timeout_s: float = 12.0,
        lock_conf_thresh: float = 0.6,
    ):
        self.min_track_hits   = min_track_hits
        self.reacquire_hits   = reacquire_hits
        self.search_timeout_s = search_timeout_s
        self.lock_conf_thresh = lock_conf_thresh

        self.state: LockState      = LockState.WAITING
        self.locked_track_id: int  = -1
        self._hit_counts: dict[int, int] = collections.defaultdict(int)
        self._candidate_id: int    = -1
        self._candidate_hits: int  = 0
        self._search_start: float  = 0.0

    # ── public API ────────────────────────────────────────────────────────────

    def update(self, detections: list[dict]) -> int | None:
        """
        Feed current-frame detections (dicts with keys: track_id, conf, class_name).
        Returns the track_id to publish as the locked target, or None.
        Must be called once per frame even when detections is empty.
        """
        now = time.monotonic()

        if self.state == LockState.WAITING:
            return self._handle_waiting(detections)

        elif self.state == LockState.TRACKING:
            return self._handle_tracking(detections)

        elif self.state == LockState.LOCKED:
            return self._handle_locked(detections, now)

        elif self.state == LockState.SEARCHING:
            return self._handle_searching(detections, now)

        elif self.state == LockState.REACQUIRING:
            return self._handle_reacquiring(detections, now)

        return None

    def reset(self):
        """Force reset to WAITING (e.g., operator command)."""
        self.state           = LockState.WAITING
        self.locked_track_id = -1
        self._hit_counts.clear()
        self._candidate_id   = -1
        self._candidate_hits = 0

    # ── state handlers ────────────────────────────────────────────────────────

    def _handle_waiting(self, dets: list[dict]) -> None:
        for d in dets:
            if d["conf"] >= self.lock_conf_thresh:
                self._hit_counts[d["track_id"]] += 1
                if self._hit_counts[d["track_id"]] >= self.min_track_hits:
                    self.locked_track_id = d["track_id"]
                    self.state = LockState.LOCKED
                    return self.locked_track_id
                self.state = LockState.TRACKING
        return None

    def _handle_tracking(self, dets: list[dict]) -> None:
        if not dets:
            self._hit_counts.clear()
            self.state = LockState.WAITING
            return None
        for d in dets:
            self._hit_counts[d["track_id"]] += 1
            if (self._hit_counts[d["track_id"]] >= self.min_track_hits
                    and d["conf"] >= self.lock_conf_thresh):
                self.locked_track_id = d["track_id"]
                self.state = LockState.LOCKED
                return self.locked_track_id
        return None

    def _handle_locked(self, dets: list[dict], now: float) -> int | None:
        ids_this_frame = {d["track_id"] for d in dets}
        if self.locked_track_id in ids_this_frame:
            return self.locked_track_id
        # locked track not seen → start searching
        self.state = LockState.SEARCHING
        self._search_start = now
        self._candidate_id = -1
        self._candidate_hits = 0
        return None

    def _handle_searching(self, dets: list[dict], now: float) -> None:
        if now - self._search_start > self.search_timeout_s:
            self.reset()
            return None
        if dets:
            best = max(dets, key=lambda d: d["conf"])
            self._candidate_id   = best["track_id"]
            self._candidate_hits = 1
            self.state = LockState.REACQUIRING
        return None

    def _handle_reacquiring(self, dets: list[dict], now: float) -> None:
        if now - self._search_start > self.search_timeout_s:
            self.reset()
            return None
        ids_this_frame = {d["track_id"] for d in dets}
        if self._candidate_id not in ids_this_frame:
            # candidate disappeared — keep searching
            self.state = LockState.SEARCHING
            return None
        self._candidate_hits += 1
        if self._candidate_hits >= self.reacquire_hits:
            self.locked_track_id = self._candidate_id
            self.state = LockState.LOCKED
            return self.locked_track_id
        return None


# ── Main ROS2 node ────────────────────────────────────────────────────────────

class DetectorNode(Node):

    def __init__(self):
        super().__init__("detector_node")

        # ── declare parameters ────────────────────────────────────────────────
        self.declare_parameter("model_path",           "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.45)
        self.declare_parameter("max_depth_m",          10.0)
        self.declare_parameter("depth_sample_fraction", 0.3)
        self.declare_parameter("publish_visualization", True)
        self.declare_parameter("use_tracker",           True)
        self.declare_parameter("tracker_config",       "botsort.yaml")
        self.declare_parameter("ema_alpha",             0.4)
        self.declare_parameter("min_track_hits",        3)
        self.declare_parameter("target_class",         "owner_person")
        self.declare_parameter("lock_conf_threshold",   0.6)
        self.declare_parameter("reacquire_hits",        5)
        self.declare_parameter("search_timeout_s",     12.0)
        self.declare_parameter("half_precision",        False)

        # ── read parameters ───────────────────────────────────────────────────
        model_path        = self.get_parameter("model_path").value
        self.conf_thresh  = self.get_parameter("confidence_threshold").value
        self.max_depth    = self.get_parameter("max_depth_m").value
        self.depth_frac   = self.get_parameter("depth_sample_fraction").value
        self.publish_vis  = self.get_parameter("publish_visualization").value
        self.use_tracker  = self.get_parameter("use_tracker").value
        tracker_cfg       = self.get_parameter("tracker_config").value
        ema_alpha         = self.get_parameter("ema_alpha").value
        min_track_hits    = self.get_parameter("min_track_hits").value
        self.target_class = self.get_parameter("target_class").value
        lock_conf_thresh  = self.get_parameter("lock_conf_threshold").value
        reacquire_hits    = self.get_parameter("reacquire_hits").value
        search_timeout    = self.get_parameter("search_timeout_s").value
        self.half         = self.get_parameter("half_precision").value

        # ── load model ────────────────────────────────────────────────────────
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        if self.half:
            self.model.model.half()
            self.get_logger().info("FP16 inference enabled")
        self.tracker_cfg = tracker_cfg

        # Class name lookup — uses model's own names (custom or COCO)
        self.class_names: dict[int, str] = (
            self.model.names if hasattr(self.model, "names") and self.model.names else {}
        )
        self.get_logger().info(f"Model classes: {self.class_names}")

        # ── camera state ──────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.fx = self.fy = self.cx = self.cy = None
        self.latest_depth: np.ndarray | None = None

        # ── tracking / smoothing / locking ────────────────────────────────────
        self.smoother  = PositionSmoother(alpha=ema_alpha)
        self.lock_mgr  = TargetLock(
            min_track_hits   = min_track_hits,
            reacquire_hits   = reacquire_hits,
            search_timeout_s = search_timeout,
            lock_conf_thresh = lock_conf_thresh,
        )

        # Performance stats
        self._frame_times: collections.deque = collections.deque(maxlen=30)
        self._fps_log_count = 0

        # ── subscriptions ─────────────────────────────────────────────────────
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.rgb_sub   = self.create_subscription(
            Image, "/camera/image_raw", self.rgb_callback, qos)
        self.depth_sub = self.create_subscription(
            Image, "/camera/depth/image_raw", self.depth_callback, qos)
        self.info_sub  = self.create_subscription(
            CameraInfo, "/camera/camera_info", self.info_callback, 10)
        # Runtime command: reset target lock
        self.cmd_sub   = self.create_subscription(
            String, "/detector/command", self._cmd_callback, 10)

        # ── publishers ────────────────────────────────────────────────────────
        self.detections_pub = self.create_publisher(DetectedObjectArray, "/detected_objects", 10)
        self.target_pub     = self.create_publisher(DetectedObjectArray, "/detector/target",  10)
        self.state_pub      = self.create_publisher(String,              "/detector/state",   10)
        self.vis_pub        = self.create_publisher(Image,               "/detector/visualization", 10)

        # Periodic state publisher (1 Hz) so navigator sees state even between frames
        self.create_timer(1.0, self._publish_state)

        mode = f"tracker={tracker_cfg}" if self.use_tracker else "predict-only"
        self.get_logger().info(
            f"GO2 DetectorNode ready. target='{self.target_class}' mode={mode}"
        )

    # ── callbacks ─────────────────────────────────────────────────────────────

    def info_callback(self, msg: CameraInfo):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(
                f"Camera intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} "
                f"cx={self.cx:.1f} cy={self.cy:.1f}"
            )

    def depth_callback(self, msg: Image):
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

    def _cmd_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == "reset_lock":
            self.lock_mgr.reset()
            self.get_logger().info("Target lock reset by command")
        elif cmd.startswith("set_target:"):
            new_cls = cmd.split(":", 1)[1].strip()
            self.target_class = new_cls
            self.lock_mgr.reset()
            self.get_logger().info(f"Target class changed to '{new_cls}'")

    def rgb_callback(self, msg: Image):
        if self.fx is None or self.latest_depth is None:
            return

        t0 = time.monotonic()
        rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # ── YOLO inference ────────────────────────────────────────────────────
        if self.use_tracker:
            results = self.model.track(
                rgb,
                persist    = True,
                conf       = self.conf_thresh,
                tracker    = self.tracker_cfg,
                verbose    = False,
                half       = self.half,
            )[0]
        else:
            results = self.model(
                rgb,
                conf    = self.conf_thresh,
                verbose = False,
                half    = self.half,
            )[0]

        depth = self.latest_depth.copy()
        vis   = rgb.copy() if self.publish_vis else None
        stamp = self.get_clock().now().to_msg()

        out_all    = DetectedObjectArray()
        out_all.header.stamp    = stamp
        out_all.header.frame_id = "camera_color_optical_frame"

        # Accumulate target-class detections for lock manager
        target_dets_this_frame: list[dict] = []
        active_track_ids: set[int] = set()

        for box in results.boxes:
            cls_id   = int(box.cls[0])
            conf     = float(box.conf[0])
            track_id = (
                int(box.id[0])
                if (self.use_tracker and box.id is not None)
                else -1
            )
            active_track_ids.add(track_id)

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx_bb = (x1 + x2) // 2
            cy_bb = (y1 + y2) // 2

            # ── Depth fusion ──────────────────────────────────────────────────
            depth_m = self._sample_depth(depth, x1, y1, x2, y2, cx_bb, cy_bb)
            if depth_m is None:
                continue

            raw_xyz = np.array([
                (cx_bb - self.cx) * depth_m / self.fx,
                (cy_bb - self.cy) * depth_m / self.fy,
                depth_m,
            ])

            # EMA smoothing (per track_id when tracking, per class when not)
            smooth_key = track_id if track_id >= 0 else cls_id
            xyz = self.smoother.update(smooth_key, raw_xyz)

            label = self.class_names.get(cls_id, str(cls_id))

            obj = self._make_detection_msg(
                label, cls_id, conf, track_id, xyz, x1, y1, x2, y2,
                lock_score=0.0
            )
            out_all.objects.append(obj)

            # Collect target-class detections for lock manager
            if label == self.target_class:
                target_dets_this_frame.append({
                    "track_id": track_id,
                    "conf":     conf,
                    "class_name": label,
                    "obj":      obj,
                })

            if vis is not None:
                self._draw_box(vis, x1, y1, x2, y2, cls_id, label, conf,
                               track_id, depth_m, is_locked=False)

        # ── Evict stale smoother state ────────────────────────────────────────
        self.smoother.evict(active_track_ids)

        # ── Target lock update ────────────────────────────────────────────────
        locked_id = self.lock_mgr.update(target_dets_this_frame)

        out_target = DetectedObjectArray()
        out_target.header = out_all.header

        if locked_id is not None:
            for td in target_dets_this_frame:
                if td["track_id"] == locked_id:
                    td["obj"].lock_score = 1.0
                    out_target.objects.append(td["obj"])
                    if vis is not None:
                        x1, y1 = td["obj"].bbox_x1, td["obj"].bbox_y1
                        x2, y2 = td["obj"].bbox_x2, td["obj"].bbox_y2
                        cv2.rectangle(vis, (x1-3, y1-3), (x2+3, y2+3), LOCK_COLOR, 3)
                    break

        # ── Publish ───────────────────────────────────────────────────────────
        self.detections_pub.publish(out_all)
        self.target_pub.publish(out_target)

        if vis is not None:
            self._draw_state_overlay(vis)
            vis_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            vis_msg.header = out_all.header
            self.vis_pub.publish(vis_msg)

        # ── FPS tracking ──────────────────────────────────────────────────────
        self._frame_times.append(time.monotonic() - t0)
        self._fps_log_count += 1
        if self._fps_log_count >= 30:
            avg_ms = 1000 * np.mean(self._frame_times)
            fps    = 1000.0 / avg_ms if avg_ms > 0 else 0.0
            self.get_logger().info(
                f"Inference: {avg_ms:.1f} ms/frame ({fps:.1f} FPS) "
                f"| state={self.lock_mgr.state.value} locked_id={self.lock_mgr.locked_track_id}"
            )
            self._fps_log_count = 0

    # ── helpers ───────────────────────────────────────────────────────────────

    def _sample_depth(self, depth: np.ndarray,
                      x1: int, y1: int, x2: int, y2: int,
                      cx_bb: int, cy_bb: int) -> float | None:
        """Return median depth in metres from the central ROI of the bounding box."""
        roi_w = max(1, int((x2 - x1) * self.depth_frac))
        roi_h = max(1, int((y2 - y1) * self.depth_frac))
        rx1 = max(0, cx_bb - roi_w // 2)
        rx2 = min(depth.shape[1], cx_bb + roi_w // 2)
        ry1 = max(0, cy_bb - roi_h // 2)
        ry2 = min(depth.shape[0], cy_bb + roi_h // 2)

        roi = depth[ry1:ry2, rx1:rx2].astype(np.float32)
        # Gazebo: 32FC1 in metres; RealSense D435i: 16UC1 in millimetres
        if depth.dtype == np.uint16:
            roi = roi / 1000.0          # mm → m
        valid = roi[(roi > 0.05) & (roi < self.max_depth)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    @staticmethod
    def _make_detection_msg(
        label: str, cls_id: int, conf: float,
        track_id: int, xyz: np.ndarray,
        x1: int, y1: int, x2: int, y2: int,
        lock_score: float,
    ) -> DetectedObject:
        obj = DetectedObject()
        obj.class_name  = label
        obj.class_id    = cls_id
        obj.confidence  = conf
        obj.track_id    = track_id
        obj.lock_score  = lock_score
        obj.position.x  = float(xyz[0])
        obj.position.y  = float(xyz[1])
        obj.position.z  = float(xyz[2])
        obj.bbox_x1 = x1
        obj.bbox_y1 = y1
        obj.bbox_x2 = x2
        obj.bbox_y2 = y2
        return obj

    def _draw_box(self, vis: np.ndarray,
                  x1: int, y1: int, x2: int, y2: int,
                  cls_id: int, label: str, conf: float,
                  track_id: int, depth_m: float,
                  is_locked: bool):
        color = PALETTE.get(cls_id, (200, 200, 200))
        cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
        tid_str = f"#{track_id}" if track_id >= 0 else ""
        text = f"{label}{tid_str} {conf:.2f} | {depth_m:.1f}m"
        cv2.putText(vis, text, (x1, max(y1 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

    def _draw_state_overlay(self, vis: np.ndarray):
        state_str = (
            f"STATE: {self.lock_mgr.state.value}  "
            f"LOCK_ID: {self.lock_mgr.locked_track_id}  "
            f"TARGET: {self.target_class}"
        )
        cv2.putText(vis, state_str, (8, vis.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, STATE_COLOR, 1, cv2.LINE_AA)

    def _publish_state(self):
        msg = String()
        msg.data = (
            f"{self.lock_mgr.state.value}|"
            f"lock_id={self.lock_mgr.locked_track_id}|"
            f"target={self.target_class}"
        )
        self.state_pub.publish(msg)


# ── entry point ───────────────────────────────────────────────────────────────

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
