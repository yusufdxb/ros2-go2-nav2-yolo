#!/usr/bin/env python3
"""
target_predictor.py — Kalman filter target predictor for occlusion recovery.

Sits between the detector and the navigator. Subscribes to /detected_objects,
maintains a Kalman filter per target class, and publishes /detector/predicted_target
which contains:
  - The real detection when the target IS visible (pass-through)
  - A synthetic detection at the Kalman-predicted position when the target
    is NOT visible (confidence degraded by time since last observation)

State machine:
  VISIBLE     → target detected this cycle → update filter, publish real detection
  COASTING    → target not detected → publish predicted position at decayed confidence
  LOST        → prediction confidence fell below min_coast_conf → publish nothing
               (causes navigator to stop issuing goals)

Kalman filter model:
  State:        [x, y, vx, vy]    (position + velocity in map frame)
  Measurement:  [x, y]            (from DetectedObject.position, assumed map frame)
  Process:      constant velocity (F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
  Measurement:  H = [[1,0,0,0],[0,1,0,0]]

Published topics:
  /detector/predicted_target    — DetectedObjectArray
  /detector/predictor_state     — String (VISIBLE/COASTING/LOST + age)

ROS2 parameters:
  target_class        str    Class name to track ['owner_person']
  coast_timeout_s     float  How long to coast before declaring LOST [5.0]
  min_coast_conf      float  Minimum confidence to publish a predicted detection [0.25]
  process_noise_q     float  Process noise for velocity [0.5]
  measurement_noise_r float  Measurement noise for position [0.2]
  prediction_hz       float  Rate to publish predictions during coasting [10.0]
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
import rclpy.time
from go2_yolo_msgs.msg import DetectedObject, DetectedObjectArray
from rclpy.node import Node
from std_msgs.msg import String

# ── 2D Kalman filter (constant velocity) ─────────────────────────────────────

class ConstantVelocityKalman:
    """
    4-state Kalman filter: [x, y, vx, vy].
    Measurement: [x, y].
    """

    def __init__(self, q: float = 0.5, r: float = 0.2):
        # State vector
        self.x = np.zeros((4, 1))           # [x, y, vx, vy]

        # State covariance — start uncertain about velocity
        self.P = np.diag([1.0, 1.0, 5.0, 5.0])

        # Measurement matrix H — we observe only [x, y]
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=float)

        # Measurement noise
        self.R = np.eye(2) * r

        # Process noise (set per prediction step based on dt)
        self._q = q

    def initialize(self, x: float, y: float):
        self.x = np.array([[x], [y], [0.0], [0.0]])
        self.P = np.diag([0.1, 0.1, 2.0, 2.0])

    def predict(self, dt: float):
        """Advance state by dt seconds."""
        F = np.array([
            [1, 0, dt,  0],
            [0, 1,  0, dt],
            [0, 0,  1,  0],
            [0, 0,  0,  1],
        ], dtype=float)

        # Process noise Q — larger noise on velocity
        q = self._q
        Q = (q * dt) * np.diag([dt**2 / 4, dt**2 / 4, 1.0, 1.0])

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z_x: float, z_y: float):
        """Incorporate a new measurement [z_x, z_y]."""
        z = np.array([[z_x], [z_y]])
        y = z - self.H @ self.x                      # innovation
        S = self.H @ self.P @ self.H.T + self.R      # innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)     # Kalman gain

        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

    @property
    def pos_x(self) -> float:
        return float(self.x[0, 0])

    @property
    def pos_y(self) -> float:
        return float(self.x[1, 0])

    @property
    def vel_x(self) -> float:
        return float(self.x[2, 0])

    @property
    def vel_y(self) -> float:
        return float(self.x[3, 0])

    @property
    def position_uncertainty(self) -> float:
        """Trace of position sub-block of P — larger = more uncertain."""
        return float(self.P[0, 0] + self.P[1, 1])


# ── Predictor state ───────────────────────────────────────────────────────────

class PredictorState:
    VISIBLE  = "VISIBLE"
    COASTING = "COASTING"
    LOST     = "LOST"


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class TargetPredictor(Node):

    def __init__(self):
        super().__init__("target_predictor")

        self.declare_parameter("target_class",        "owner_person")
        self.declare_parameter("coast_timeout_s",     5.0)
        self.declare_parameter("min_coast_conf",      0.25)
        self.declare_parameter("process_noise_q",     0.5)
        self.declare_parameter("measurement_noise_r", 0.2)
        self.declare_parameter("prediction_hz",       10.0)

        self.target_class    = self.get_parameter("target_class").value
        self.coast_timeout   = self.get_parameter("coast_timeout_s").value
        self.min_coast_conf  = self.get_parameter("min_coast_conf").value
        q                    = self.get_parameter("process_noise_q").value
        r                    = self.get_parameter("measurement_noise_r").value
        pred_hz              = self.get_parameter("prediction_hz").value

        self.kf    = ConstantVelocityKalman(q=q, r=r)
        self.state = PredictorState.LOST

        self._initialized      = False
        self._last_detection_t = 0.0
        self._last_predict_t   = time.monotonic()

        # Last known detection (to copy metadata for synthetic messages)
        self._last_obj: DetectedObject | None = None

        # Subscriptions
        self.create_subscription(
            DetectedObjectArray, "/detected_objects", self._det_callback, 10
        )

        # Publishers
        self.pred_pub  = self.create_publisher(
            DetectedObjectArray, "/detector/predicted_target", 10
        )
        self.state_pub = self.create_publisher(String, "/detector/predictor_state", 10)

        # Coast timer — runs continuously, handles COASTING state publishing
        self.create_timer(1.0 / pred_hz, self._coast_tick)

        self.get_logger().info(
            f"TargetPredictor ready: target='{self.target_class}' "
            f"coast_timeout={self.coast_timeout}s"
        )

    # ── Detection input ───────────────────────────────────────────────────────

    def _det_callback(self, msg: DetectedObjectArray):
        """Process incoming detections and update Kalman filter."""
        now = time.monotonic()

        # Find best target-class detection
        candidates = [
            o for o in msg.objects if o.class_name == self.target_class
        ]
        if not candidates:
            # No detection this frame — transition to COASTING if was VISIBLE
            if self.state == PredictorState.VISIBLE:
                self.state = PredictorState.COASTING
                self.get_logger().info(
                    f"Target lost — entering COASTING "
                    f"(last pos: {self.kf.pos_x:.2f}, {self.kf.pos_y:.2f})"
                )
            return

        best = max(candidates, key=lambda o: o.confidence)
        z_x = best.position.x
        z_y = best.position.y

        dt = now - self._last_predict_t
        if dt > 0.5:
            dt = 0.05  # cap large initial DT

        if not self._initialized:
            self.kf.initialize(z_x, z_y)
            self._initialized = True
        else:
            self.kf.predict(dt)
            self.kf.update(z_x, z_y)

        self._last_predict_t   = now
        self._last_detection_t = now
        self._last_obj         = best
        self.state             = PredictorState.VISIBLE

        # Publish immediately with real detection (pass-through)
        self._publish(best, confidence=best.confidence, predicted=False)

    # ── Coast timer ───────────────────────────────────────────────────────────

    def _coast_tick(self):
        """Called at prediction_hz. Handles COASTING and LOST states."""
        now = time.monotonic()

        if self.state == PredictorState.VISIBLE:
            # Detection callback handles publishing while visible
            self._publish_state(now)
            return

        if not self._initialized:
            self.state = PredictorState.LOST
            self._publish_state(now)
            return

        time_since_last = now - self._last_detection_t
        if time_since_last > self.coast_timeout:
            self.state = PredictorState.LOST
            self._publish_state(now)
            return

        # Advance Kalman prediction
        dt = now - self._last_predict_t
        dt = max(0.001, min(dt, 0.2))   # clamp to sane range
        self.kf.predict(dt)
        self._last_predict_t = now

        # Confidence decays linearly with time since last observation
        coast_frac = time_since_last / self.coast_timeout
        confidence = max(self.min_coast_conf, 1.0 - coast_frac)

        if confidence < self.min_coast_conf:
            self.state = PredictorState.LOST
            self._publish_state(now)
            return

        self.state = PredictorState.COASTING

        # Build a synthetic DetectedObject at the predicted position
        synth = DetectedObject()
        if self._last_obj is not None:
            synth.class_name = self._last_obj.class_name
            synth.class_id   = self._last_obj.class_id
            synth.track_id   = self._last_obj.track_id
            synth.bbox_x1    = self._last_obj.bbox_x1
            synth.bbox_y1    = self._last_obj.bbox_y1
            synth.bbox_x2    = self._last_obj.bbox_x2
            synth.bbox_y2    = self._last_obj.bbox_y2
        else:
            synth.class_name = self.target_class
            synth.class_id   = 0
            synth.track_id   = -1

        synth.confidence  = confidence
        synth.lock_score  = confidence
        synth.position.x  = self.kf.pos_x
        synth.position.y  = self.kf.pos_y
        synth.position.z  = 0.0

        self._publish(synth, confidence=confidence, predicted=True)
        self._publish_state(now)

    # ── Publishing helpers ────────────────────────────────────────────────────

    def _publish(self, obj: DetectedObject, confidence: float, predicted: bool):
        out = DetectedObjectArray()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        obj.confidence      = confidence
        out.objects         = [obj]
        self.pred_pub.publish(out)

    def _publish_state(self, now: float):
        age = now - self._last_detection_t
        msg = String()
        msg.data = (
            f"{self.state}"
            f"|age={age:.1f}s"
            f"|pos=({self.kf.pos_x:.2f},{self.kf.pos_y:.2f})"
            f"|vel=({self.kf.vel_x:.2f},{self.kf.vel_y:.2f})"
            f"|unc={self.kf.position_uncertainty:.3f}"
        )
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetPredictor()
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
