#!/usr/bin/env python3
"""
sim_person_detector.py — Gazebo ground truth person detector for simulation.

Publishes the person_standing model's fixed world position as a
DetectedObjectArray in the map frame at 2 Hz.

The person_standing model is placed at (2, 0, 0) in demo_world.world and
does not move, so polling /gazebo/model_states is unnecessary. This timer-
based approach avoids a dependency on the gazebo_ros_state plugin.

Replaces the YOLOv8 detector for simulation where software rendering
(llvmpipe) produces images too synthetic for YOLO to detect. The navigator
node is unaware of the switch — it receives the same DetectedObjectArray
message type.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from go2_yolo_msgs.msg import DetectedObject, DetectedObjectArray


class SimPersonDetector(Node):
    def __init__(self):
        super().__init__("sim_person_detector")

        self.declare_parameter("person_x", 2.0)
        self.declare_parameter("person_y", 0.0)
        self.declare_parameter("person_z", 0.0)
        self.declare_parameter("confidence", 1.0)

        self.person_x = float(self.get_parameter("person_x").value)
        self.person_y = float(self.get_parameter("person_y").value)
        self.person_z = float(self.get_parameter("person_z").value)
        self.confidence = float(self.get_parameter("confidence").value)

        self.pub = self.create_publisher(DetectedObjectArray, "/detected_objects", 10)
        self.create_timer(0.5, self.publish_detection)  # 2 Hz

        self.get_logger().info(
            f"SimPersonDetector ready. Publishing person at "
            f"({self.person_x}, {self.person_y}, {self.person_z})"
        )

    def publish_detection(self):
        obj = DetectedObject()
        obj.class_name = "person"
        obj.class_id = 0        # COCO person class ID
        obj.confidence = self.confidence
        obj.position.x = self.person_x
        obj.position.y = self.person_y
        obj.position.z = self.person_z

        out = DetectedObjectArray()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
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
