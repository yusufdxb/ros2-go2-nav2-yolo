#!/usr/bin/env python3
"""
multi_agent_sim_detector.py — Multi-person simulation detector with occlusion.

Replaces sim_person_detector.py for the advanced demo scenario. Instead of
publishing a single fixed position, this node:

  1. Subscribes to /agent_manager/agent_states (PoseArray, world/map frame)
     to know where all agents are.
  2. Subscribes to /robot_pose (or uses TF: map → base_footprint) to know
     where the robot is, so it can compute line-of-sight.
  3. Performs a simplified 2D line-of-sight (LOS) occlusion check against
     a list of known obstacle rectangles in the world.
  4. Publishes /detected_objects with:
       - All visible agents (not occluded)
       - target agent labeled 'owner_person', decoys labeled 'person'
       - confidence = 1.0 for visible targets
  5. When the target is occluded, publishes an empty /detected_objects so
     the navigator stops issuing goals (and the predictor fills in).

LOS occlusion model:
  For each agent, cast a 2D ray from the robot center to the agent center.
  If the ray intersects any obstacle rectangle (expanded by robot_radius),
  the agent is marked occluded and not published.

Obstacle rectangles are hardcoded to match advanced_demo_world.world.

Published topics:
  /detected_objects      — DetectedObjectArray (visible agents only)
  /sim_detector/status   — String (VISIBLE / OCCLUDED / SEARCHING)

ROS2 parameters:
  target_model_name   str    Gazebo model name of the designated target ['person_1']
  detection_rate_hz   float  Detection publish rate [10.0]
  los_check           bool   Enable line-of-sight occlusion [true]
  robot_radius_m      float  Robot radius for LOS ray inflation [0.3]
  use_sim_time        bool   [true]
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import String, Header
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration

from go2_yolo_msgs.msg import DetectedObject, DetectedObjectArray


# ── Obstacle rectangles for LOS check ────────────────────────────────────────
# Each entry: (cx, cy, half_width, half_height) in world/map frame (metres).
# Derived from advanced_demo_world.world geometry (outer + inner walls, boxes).

OBSTACLE_RECTS: list[tuple[float, float, float, float]] = [
    # Outer boundary walls (thick enough for robust ray check)
    (  0.0,  10.0, 13.0,  0.15),   # North
    (  0.0, -10.0, 13.0,  0.15),   # South
    (-13.0,   0.0,  0.15, 10.0),   # West
    ( 13.0,   0.0,  0.15, 10.0),   # East
    # Central E-W dividing wall (two segments with gap in middle)
    ( -6.0,   2.0,  3.0,  0.075),  # West segment
    (  6.0,   2.0,  3.0,  0.075),  # East segment
    # N-S inner wall (NW zone separator)
    ( -7.0,   6.0,  0.075, 3.0),
    # E corridor short wall
    (  5.0,   0.0,  0.075, 2.0),
    # Box obstacles
    (  3.0,   5.0,  0.5,  0.5),
    ( -3.0,  -4.0,  0.5,  0.5),
    (  7.0,  -5.0,  0.5,  0.5),
    ( -7.0,  -5.0,  0.5,  0.5),
    (  0.0,  -7.0,  1.0,  0.5),
]

# Names of all person models in the world (match model names in the SDF)
ALL_AGENT_MODELS = ["person_1", "person_2", "person_3", "person_4"]
TARGET_CLASS     = "owner_person"
DECOY_CLASS      = "person"


# ── Geometry helpers ──────────────────────────────────────────────────────────

def _ray_intersects_rect(
    rx: float, ry: float,     # ray origin
    tx: float, ty: float,     # ray end point (target)
    cx: float, cy: float,     # rect center
    hw: float, hh: float,     # half-width, half-height
    pad: float = 0.0,         # extra padding on all sides
) -> bool:
    """Slab-based 2D ray-AABB intersection test."""
    hw += pad
    hh += pad

    dx, dy = tx - rx, ty - ry
    # Parametric ray: P(t) = R + t*D for t in [0, 1]
    # Test against each axis slab

    t_min, t_max = 0.0, 1.0

    for r, d, c, h in [(rx, dx, cx, hw), (ry, dy, cy, hh)]:
        if abs(d) < 1e-9:
            # Ray parallel to slab — check if inside
            if r < c - h or r > c + h:
                return False
        else:
            t1 = (c - h - r) / d
            t2 = (c + h - r) / d
            if t1 > t2:
                t1, t2 = t2, t1
            t_min = max(t_min, t1)
            t_max = min(t_max, t2)
            if t_min > t_max:
                return False

    return True


def _is_occluded(
    rx: float, ry: float,   # robot position
    tx: float, ty: float,   # target position
    pad: float,
) -> bool:
    """Check if any obstacle blocks the ray from robot to target."""
    for (cx, cy, hw, hh) in OBSTACLE_RECTS:
        if _ray_intersects_rect(rx, ry, tx, ty, cx, cy, hw, hh, pad=pad):
            return True
    return False


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class MultiAgentSimDetector(Node):

    def __init__(self):
        super().__init__("multi_agent_sim_detector")

        self.declare_parameter("target_model_name", "person_1")
        self.declare_parameter("detection_rate_hz", 10.0)
        self.declare_parameter("los_check",          True)
        self.declare_parameter("robot_radius_m",     0.3)

        self.target_name    = self.get_parameter("target_model_name").value
        detection_hz        = self.get_parameter("detection_rate_hz").value
        self.los_check      = self.get_parameter("los_check").value
        self.robot_pad      = self.get_parameter("robot_radius_m").value

        # Latest agent pose data keyed by model index (same order as PoseArray)
        self._agent_poses: list[tuple[float, float]] = []  # (x, y) per agent
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0

        # TF for robot pose
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.create_subscription(
            PoseArray, "/agent_manager/agent_states", self._agents_callback, 10
        )

        # Publishers
        self.det_pub    = self.create_publisher(DetectedObjectArray, "/detected_objects", 10)
        self.status_pub = self.create_publisher(String, "/sim_detector/status", 10)

        # Detection timer
        self.create_timer(1.0 / detection_hz, self._publish_detections)

        self.get_logger().info(
            f"MultiAgentSimDetector ready: target='{self.target_name}' "
            f"LOS={'on' if self.los_check else 'off'}"
        )

    def _agents_callback(self, msg: PoseArray):
        self._agent_poses = [
            (p.position.x, p.position.y) for p in msg.poses
        ]

    def _get_robot_pose(self) -> tuple[float, float]:
        """Lookup robot position in map frame via TF."""
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_footprint",
                rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            return self._robot_x, self._robot_y  # fall back to last known

    def _publish_detections(self):
        if not self._agent_poses:
            return

        robot_x, robot_y = self._get_robot_pose()

        out = DetectedObjectArray()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = "map"

        target_visible  = False
        target_occluded = False

        for idx, (name) in enumerate(ALL_AGENT_MODELS):
            if idx >= len(self._agent_poses):
                continue

            ax, ay = self._agent_poses[idx]

            # LOS occlusion check
            if self.los_check:
                if _is_occluded(robot_x, robot_y, ax, ay, pad=self.robot_pad):
                    if name == self.target_name:
                        target_occluded = True
                    continue    # skip occluded agent

            is_target = (name == self.target_name)
            class_name = TARGET_CLASS if is_target else DECOY_CLASS
            class_id   = 0 if is_target else 1
            conf       = 1.0

            obj = DetectedObject()
            obj.class_name  = class_name
            obj.class_id    = class_id
            obj.confidence  = conf
            obj.track_id    = idx          # use idx as stable sim track ID
            obj.lock_score  = 1.0 if is_target else 0.0
            obj.position.x  = ax
            obj.position.y  = ay
            obj.position.z  = 0.0
            # No bbox in sim mode
            obj.bbox_x1 = obj.bbox_y1 = obj.bbox_x2 = obj.bbox_y2 = 0

            out.objects.append(obj)

            if is_target:
                target_visible = True

        self.det_pub.publish(out)

        # Status
        if target_visible:
            status = "VISIBLE"
        elif target_occluded:
            status = "OCCLUDED"
        else:
            status = "SEARCHING"

        self.status_pub.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = MultiAgentSimDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
