#!/usr/bin/env python3
"""
GO2 Nav2 YOLO Demo — Navigator Node (improved moving-target version)
=====================================================================
Key difference from TurtleBot3 version: uses base_footprint (published by CHAMP).
TF chain: camera_color_optical_frame → trunk → base_link → base_footprint → odom → map

Improvements over original:
  - Adaptive goal cooldown: faster replanning when target is moving fast
  - Target velocity estimate: goals are placed ahead of the target if moving
  - Dead-zone hysteresis: robot stops sending goals once within goal_offset_m
  - Goal staleness timeout: cancels a nav goal if robot is stuck too long
  - Detection topic is parameterised: can switch to /detector/predicted_target
    to use Kalman predictions during occlusions

ROS2 parameters:
  target_class         str    Class name to follow ['owner_person']
  min_confidence       float  Min detection confidence [0.5]
  goal_offset_m        float  Stop this far from target [0.8]
  replan_distance_m    float  Only replan if target moved this far [0.3]
  goal_cooldown_s      float  Minimum seconds between goal sends [2.0]
  max_goal_age_s       float  Cancel goal if stuck navigating for this long [30.0]
  use_velocity_lead    bool   Place goal ahead of moving target [true]
  velocity_lead_s      float  Seconds of look-ahead for target position [1.0]
  detection_topic      str    Topic to consume detections from [/detected_objects]
  use_sim_time         bool   [true]
"""

import math
import collections

import rclpy
import rclpy.time
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import tf2_ros

from go2_yolo_msgs.msg import DetectedObjectArray


class NavigatorNode(Node):

    def __init__(self):
        super().__init__("navigator_node")

        # ── parameters ────────────────────────────────────────────────────────
        self.declare_parameter("target_class",      "owner_person")
        self.declare_parameter("min_confidence",    0.5)
        self.declare_parameter("goal_offset_m",     0.8)
        self.declare_parameter("replan_distance_m", 0.3)
        self.declare_parameter("goal_cooldown_s",   2.0)
        self.declare_parameter("max_goal_age_s",    30.0)
        self.declare_parameter("use_velocity_lead", True)
        self.declare_parameter("velocity_lead_s",   1.0)
        self.declare_parameter("detection_topic",   "/detected_objects")

        self.target_class    = self.get_parameter("target_class").value
        self.min_conf        = self.get_parameter("min_confidence").value
        self.goal_offset     = self.get_parameter("goal_offset_m").value
        self.replan_dist     = self.get_parameter("replan_distance_m").value
        self.base_cooldown   = self.get_parameter("goal_cooldown_s").value
        self.max_goal_age    = self.get_parameter("max_goal_age_s").value
        self.use_vel_lead    = self.get_parameter("use_velocity_lead").value
        self.vel_lead_s      = self.get_parameter("velocity_lead_s").value
        det_topic            = self.get_parameter("detection_topic").value

        # ── TF ────────────────────────────────────────────────────────────────
        # SafeBuffer silently drops malformed transforms (known tf2_ros/Humble
        # bug: "transform.translation must have members x, y, z" on some msgs)
        class _SafeBuffer(tf2_ros.Buffer):
            def set_transform(self, transform, authority):
                try:
                    super().set_transform(transform, authority)
                except TypeError:
                    pass
            def set_transform_static(self, transform, authority):
                try:
                    super().set_transform_static(transform, authority)
                except TypeError:
                    pass

        self.tf_buffer   = _SafeBuffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Nav2 action client ────────────────────────────────────────────────
        self.nav_client      = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.current_goal    = None
        self.nav_active      = False
        self._goal_handle    = None
        self._last_goal_time = 0.0
        self._goal_sent_time = 0.0

        # ── Target velocity estimation ────────────────────────────────────────
        # Sliding window of (time, x, y) in map frame — used to estimate velocity
        self._pos_history: collections.deque = collections.deque(maxlen=8)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.detections_sub = self.create_subscription(
            DetectedObjectArray, det_topic, self.detections_callback, 10
        )
        self.target_sub = self.create_subscription(
            String, "/navigator/target_class", self._target_callback, 10
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.status_pub = self.create_publisher(String, "/navigator/status", 10)

        # Staleness watchdog — checks if we're stuck
        self.create_timer(2.0, self._watchdog)

        self.get_logger().info(
            f"GO2 NavigatorNode ready. target='{self.target_class}' "
            f"det_topic='{det_topic}'"
        )
        self._publish_status("WAITING_FOR_DETECTION")

    # ── Target class runtime override ────────────────────────────────────────

    def _target_callback(self, msg: String):
        new_target = msg.data.strip()
        if new_target != self.target_class:
            self.get_logger().info(
                f"Target class changed: '{self.target_class}' → '{new_target}'"
            )
            self.target_class = new_target
            self._pos_history.clear()
            self._cancel_current_goal()

    # ── Main detection callback ───────────────────────────────────────────────

    def detections_callback(self, msg: DetectedObjectArray):
        candidates = [
            o for o in msg.objects
            if o.class_name == self.target_class and o.confidence >= self.min_conf
        ]
        if not candidates:
            return

        best = max(candidates, key=lambda o: o.confidence)

        # ── Transform detection to map frame ──────────────────────────────────
        cam_pose = PoseStamped()
        cam_pose.header = msg.header
        cam_pose.pose.position.x = best.position.x
        cam_pose.pose.position.y = best.position.y
        cam_pose.pose.position.z = best.position.z
        cam_pose.pose.orientation.w = 1.0

        # If already in map frame (sim detectors publish in map), skip TF
        if msg.header.frame_id == "map":
            map_pose = cam_pose
        else:
            try:
                map_pose = self.tf_buffer.transform(
                    cam_pose, "map", timeout=Duration(seconds=1.0)
                )
            except Exception as e:
                self.get_logger().warn(f"TF transform to map failed: {e}")
                return

        # ── Robot pose ────────────────────────────────────────────────────────
        try:
            robot_tf = self.tf_buffer.lookup_transform(
                "map", "base_footprint",
                rclpy.time.Time(clock_type=ClockType.ROS_TIME), timeout=Duration(seconds=1.0)
            )
            robot_x = robot_tf.transform.translation.x
            robot_y = robot_tf.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"Robot TF lookup failed: {e}")
            return

        tx = map_pose.pose.position.x
        ty = map_pose.pose.position.y

        # ── Update target velocity estimate ───────────────────────────────────
        now_s = self.get_clock().now().nanoseconds / 1e9
        self._pos_history.append((now_s, tx, ty))
        vx, vy = self._estimate_velocity()

        # ── Distance checks ───────────────────────────────────────────────────
        dist_to_target = math.hypot(tx - robot_x, ty - robot_y)

        # Dead-zone: within approach distance — no goal needed
        if dist_to_target <= self.goal_offset + 0.1:
            if self.nav_active:
                self._cancel_current_goal()
                self._publish_status("ARRIVED")
            return

        # ── Velocity lead: aim at where target will be ────────────────────────
        if self.use_vel_lead and (abs(vx) + abs(vy)) > 0.1:
            # Lead by vel_lead_s of predicted motion, capped at 1.5 m
            lead_x = vx * self.vel_lead_s
            lead_y = vy * self.vel_lead_s
            lead_mag = math.hypot(lead_x, lead_y)
            if lead_mag > 1.5:
                scale = 1.5 / lead_mag
                lead_x *= scale
                lead_y *= scale
            tx += lead_x
            ty += lead_y

        # ── Yaw from robot toward (led) target ────────────────────────────────
        yaw = math.atan2(ty - robot_y, tx - robot_x)

        # Offset: stop goal_offset_m in front of target
        gx = tx - self.goal_offset * math.cos(yaw)
        gy = ty - self.goal_offset * math.sin(yaw)

        # ── Adaptive cooldown: faster when target is moving ───────────────────
        speed = math.hypot(vx, vy)
        adaptive_cooldown = self.base_cooldown
        if speed > 0.5:
            # Scale cooldown from base_cooldown down to 0.5s at high speed
            adaptive_cooldown = max(0.5, self.base_cooldown - speed)

        if now_s - self._last_goal_time < adaptive_cooldown:
            return

        # ── Replan distance gate ──────────────────────────────────────────────
        if self.current_goal is not None and self.nav_active:
            dx = gx - self.current_goal[0]
            dy = gy - self.current_goal[1]
            if math.hypot(dx, dy) < self.replan_dist:
                return
            self._cancel_current_goal()

        # ── Build and send goal ───────────────────────────────────────────────
        goal_pose = PoseStamped()
        goal_pose.header.stamp    = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = gx
        goal_pose.pose.position.y = gy
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.current_goal    = (gx, gy)
        self._goal_sent_time = now_s
        self._send_nav_goal(goal_pose)

    # ── Velocity estimation ───────────────────────────────────────────────────

    def _estimate_velocity(self) -> tuple[float, float]:
        """Estimate target velocity using linear regression over position history."""
        if len(self._pos_history) < 3:
            return 0.0, 0.0

        # Use oldest and most recent samples for simple finite difference
        t0, x0, y0 = self._pos_history[0]
        tn, xn, yn = self._pos_history[-1]
        dt = tn - t0
        if dt < 0.1:
            return 0.0, 0.0
        return (xn - x0) / dt, (yn - y0) / dt

    # ── Nav2 goal management ──────────────────────────────────────────────────

    def _send_nav_goal(self, pose: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn("Nav2 not available — will retry on next detection")
            self.nav_active = False
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        gx = pose.pose.position.x
        gy = pose.pose.position.y
        vx, vy = self._estimate_velocity()
        speed = math.hypot(vx, vy)

        self.get_logger().info(
            f"Navigating to '{self.target_class}' goal=({gx:.2f},{gy:.2f}) "
            f"target_speed={speed:.2f}m/s"
        )
        self._publish_status(f"NAVIGATING|target={self.target_class}|speed={speed:.2f}")

        future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        future.add_done_callback(self._goal_accepted_cb)
        self.nav_active = True
        self._last_goal_time = self.get_clock().now().nanoseconds / 1e9

    def _goal_accepted_cb(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self.nav_active = False
            return
        self._goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, fb):
        self.get_logger().debug(
            f"Distance remaining: {fb.feedback.distance_remaining:.2f}m"
        )

    def _result_cb(self, future):
        self.nav_active   = False
        self.current_goal = None
        self.get_logger().info("GO2 reached target goal!")
        self._publish_status("ARRIVED")

    def _cancel_current_goal(self):
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()
        self.nav_active   = False
        self.current_goal = None

    # ── Staleness watchdog ────────────────────────────────────────────────────

    def _watchdog(self):
        """Cancel goal if we've been navigating for too long (robot stuck)."""
        if not self.nav_active:
            return
        now_s = self.get_clock().now().nanoseconds / 1e9
        if now_s - self._goal_sent_time > self.max_goal_age:
            self.get_logger().warn(
                f"Goal age exceeded {self.max_goal_age:.0f}s — cancelling (robot may be stuck)"
            )
            self._cancel_current_goal()
            self._publish_status("STUCK_TIMEOUT")

    # ── Status publisher ──────────────────────────────────────────────────────

    def _publish_status(self, s: str):
        msg = String()
        msg.data = s
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
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
