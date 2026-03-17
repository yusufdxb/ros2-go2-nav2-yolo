#!/usr/bin/env python3
"""
GO2 Nav2 YOLO Demo — Navigator Node
Key difference from TurtleBot3 version: uses base_footprint (published by CHAMP).
TF chain: camera_color_optical_frame → trunk → base_link → base_footprint → odom → map

Fixes applied:
  - yaw computed from robot position → object (not map origin → object)
  - TF timeout increased to 1.0s to survive SLAM startup latency
  - Nav2 server wait increased to 10.0s with retry on next detection
"""

import math
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import tf2_ros
import tf2_geometry_msgs

from go2_yolo_msgs.msg import DetectedObjectArray


class NavigatorNode(Node):
    def __init__(self):
        super().__init__("navigator_node")

        self.declare_parameter("target_class", "person")
        self.declare_parameter("min_confidence", 0.5)
        self.declare_parameter("goal_offset_m", 0.8)
        self.declare_parameter("replan_distance_m", 0.4)

        self.target_class = self.get_parameter("target_class").value
        self.min_conf = self.get_parameter("min_confidence").value
        self.goal_offset = self.get_parameter("goal_offset_m").value
        self.replan_dist = self.get_parameter("replan_distance_m").value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.current_goal_pose = None
        self.nav_active = False
        self._goal_handle = None

        self.detections_sub = self.create_subscription(
            DetectedObjectArray, "/detected_objects", self.detections_callback, 10
        )

        # Allow changing target class at runtime
        self.target_sub = self.create_subscription(
            String, "/navigator/target_class", self._target_callback, 10
        )

        self.status_pub = self.create_publisher(String, "/navigator/status", 10)

        self.get_logger().info(
            f"GO2 NavigatorNode ready. Target: '{self.target_class}'"
        )
        self._publish_status("WAITING_FOR_DETECTION")

    def _target_callback(self, msg: String):
        new_target = msg.data.strip()
        if new_target != self.target_class:
            self.get_logger().info(f"Target class changed: '{self.target_class}' → '{new_target}'")
            self.target_class = new_target
            self._cancel_current_goal()

    def detections_callback(self, msg: DetectedObjectArray):
        candidates = [
            o for o in msg.objects
            if o.class_name == self.target_class and o.confidence >= self.min_conf
        ]
        if not candidates:
            return

        best = max(candidates, key=lambda o: o.confidence)

        cam_pose = PoseStamped()
        cam_pose.header = msg.header
        cam_pose.pose.position.x = best.position.x
        cam_pose.pose.position.y = best.position.y
        cam_pose.pose.position.z = best.position.z
        cam_pose.pose.orientation.w = 1.0

        try:
            map_pose = self.tf_buffer.transform(
                cam_pose, "map", timeout=Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f"TF transform to map failed: {e}")
            return

        # Get robot's current position in map frame for correct yaw calculation
        try:
            robot_tf = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time(), timeout=Duration(seconds=1.0)
            )
            robot_x = robot_tf.transform.translation.x
            robot_y = robot_tf.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"TF lookup for robot pose failed: {e}")
            return

        # Approach offset: yaw from robot → object (not map origin → object)
        yaw = math.atan2(
            map_pose.pose.position.y - robot_y,
            map_pose.pose.position.x - robot_x,
        )
        map_pose.pose.position.x -= self.goal_offset * math.cos(yaw)
        map_pose.pose.position.y -= self.goal_offset * math.sin(yaw)
        map_pose.pose.orientation.z = math.sin(yaw / 2.0)
        map_pose.pose.orientation.w = math.cos(yaw / 2.0)

        if self.current_goal_pose is not None and self.nav_active:
            dx = map_pose.pose.position.x - self.current_goal_pose.pose.position.x
            dy = map_pose.pose.position.y - self.current_goal_pose.pose.position.y
            if math.sqrt(dx*dx + dy*dy) < self.replan_dist:
                return
            self._cancel_current_goal()

        self.current_goal_pose = map_pose
        self._send_nav_goal(map_pose)

    def _send_nav_goal(self, pose: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn("Nav2 not available yet — will retry on next detection")
            self.nav_active = False
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(
            f"GO2 navigating to '{self.target_class}' at "
            f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
        )
        self._publish_status(f"NAVIGATING_TO_{self.target_class.upper()}")

        future = self.nav_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_accepted_cb)
        self.nav_active = True

    def _goal_accepted_cb(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self.nav_active = False
            return
        self._goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, fb):
        self.get_logger().debug(f"Distance remaining: {fb.feedback.distance_remaining:.2f}m")

    def _result_cb(self, future):
        self.nav_active = False
        self.current_goal_pose = None
        self.get_logger().info("GO2 reached target!")
        self._publish_status("ARRIVED")

    def _cancel_current_goal(self):
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()
        self.nav_active = False

    def _publish_status(self, s: str):
        msg = String(); msg.data = s
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
