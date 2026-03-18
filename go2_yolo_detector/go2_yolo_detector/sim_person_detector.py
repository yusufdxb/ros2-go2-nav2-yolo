#!/usr/bin/env python3
"""
sim_person_detector.py — Gazebo ground truth person detector for simulation.

Reads the person_standing model's world position from /gazebo/model_states
and publishes it as a DetectedObjectArray in the map frame.

Replaces the YOLOv8 detector for simulation where software rendering
(llvmpipe) produces images too synthetic for YOLO to detect. The navigator
node is unaware of the switch — it receives the same DetectedObjectArray
message type and TFs the position from map frame to map frame (identity).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from gazebo_msgs.msg import ModelStates
from go2_yolo_msgs.msg import DetectedObject, DetectedObjectArray


class SimPersonDetector(Node):
    def __init__(self):
        super().__init__("sim_person_detector")

        self.declare_parameter("model_name", "person_standing")
        self.declare_parameter("confidence", 1.0)

        self.model_name = self.get_parameter("model_name").value
        self.confidence = float(self.get_parameter("confidence").value)

        self.pub = self.create_publisher(DetectedObjectArray, "/detected_objects", 10)
        self.sub = self.create_subscription(
            ModelStates,
            "/gazebo/model_states",
            self.model_states_cb,
            10,
        )

        self.get_logger().info(
            f"SimPersonDetector ready. Tracking Gazebo model: '{self.model_name}'"
        )

    def model_states_cb(self, msg: ModelStates):
        if self.model_name not in msg.name:
            return

        idx = msg.name.index(self.model_name)
        pose = msg.pose[idx]

        obj = DetectedObject()
        obj.class_name = "person"
        obj.class_id = 0        # COCO person class ID
        obj.confidence = self.confidence
        obj.position.x = pose.position.x
        obj.position.y = pose.position.y
        obj.position.z = pose.position.z
        # bbox fields unused by navigator — leave as zero

        out = DetectedObjectArray()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"   # navigator TFs this to map (identity)
        out.objects = [obj]

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SimPersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
