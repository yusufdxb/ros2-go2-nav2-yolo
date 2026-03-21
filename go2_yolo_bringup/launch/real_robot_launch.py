"""
real_robot_launch.py — Launch the GO2 YOLO perception + navigation stack
on real hardware (no Gazebo).

Architecture:
  Desktop GPU machine runs:
    - detector_node (YOLOv8 + BoT-SORT + target lock)
    - navigator_node (Nav2 goal publisher)

  Robot machine runs:
    - CHAMP controller (locomotion)
    - Nav2 (planning)
    - SLAM (mapping)
  Camera streams over ROS2 DDS (LAN).

Prerequisites:
  Both machines: export ROS_DOMAIN_ID=42
  Desktop:       source ~/go2_sim_env.sh  (sets RMW, CYCLONEDDS_URI)
  Weights:       ~/models/go2_perception/best.pt  (or set model_path arg)

Usage:
  ros2 launch go2_yolo_bringup real_robot_launch.py
  ros2 launch go2_yolo_bringup real_robot_launch.py model_path:=/path/to/best.pt
  ros2 launch go2_yolo_bringup real_robot_launch.py target_class:=owner_person confidence_threshold:=0.55
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_PKG = "go2_yolo_bringup"
_DEFAULT_WEIGHTS = str(Path.home() / "models/go2_perception/best.pt")
_DEFAULT_PARAMS  = str(
    Path(__file__).parent.parent / "config" / "detector_params.yaml"
)


def generate_launch_description():

    # ── Declare overridable arguments ─────────────────────────────────────────
    args = [
        DeclareLaunchArgument("model_path",           default_value=_DEFAULT_WEIGHTS,
                              description="Path to YOLO .pt/.torchscript/.onnx weights"),
        DeclareLaunchArgument("target_class",         default_value="owner_person",
                              description="Class name to lock and follow"),
        DeclareLaunchArgument("confidence_threshold", default_value="0.50"),
        DeclareLaunchArgument("use_tracker",          default_value="true"),
        DeclareLaunchArgument("half_precision",       default_value="false"),
        DeclareLaunchArgument("publish_visualization",default_value="true"),
        DeclareLaunchArgument("params_file",          default_value=_DEFAULT_PARAMS,
                              description="YAML parameter file (detector_params.yaml)"),
    ]

    model_path            = LaunchConfiguration("model_path")
    target_class          = LaunchConfiguration("target_class")
    confidence_threshold  = LaunchConfiguration("confidence_threshold")
    use_tracker           = LaunchConfiguration("use_tracker")
    half_precision        = LaunchConfiguration("half_precision")
    publish_visualization = LaunchConfiguration("publish_visualization")
    params_file           = LaunchConfiguration("params_file")

    # ── Detector node ─────────────────────────────────────────────────────────
    # Remaps generic /camera/* topics to the GO2 RealSense topic names.
    # Adjust remappings if your camera driver uses different names.
    detector = Node(
        package    = "go2_yolo_detector",
        executable = "detector_node",
        name       = "detector_node",
        parameters = [
            params_file,  # base params from YAML
            # CLI overrides (take priority over YAML when provided)
            {
                "model_path":            model_path,
                "target_class":          target_class,
                "confidence_threshold":  confidence_threshold,
                "use_tracker":           use_tracker,
                "half_precision":        half_precision,
                "publish_visualization": publish_visualization,
                "use_sim_time":          False,
            },
        ],
        remappings = [
            # RealSense D435i (via realsense2_ros)
            ("/camera/image_raw",       "/go2/camera/color/image_raw"),
            ("/camera/depth/image_raw", "/go2/camera/depth/image_rect_raw"),
            ("/camera/camera_info",     "/go2/camera/color/camera_info"),
            # Fallback: if using the GO2's built-in depth camera directly,
            # change the right-hand sides to match your driver's topic names.
        ],
        output = "screen",
    )

    # ── Navigator node ────────────────────────────────────────────────────────
    # Subscribes to /detected_objects published by detector_node.
    # Sends goals to Nav2 running on the robot or desktop.
    navigator = Node(
        package    = "go2_yolo_navigator",
        executable = "navigator_node",
        name       = "navigator_node",
        parameters = [
            params_file,
            {
                "target_class": target_class,
                "use_sim_time": False,
            },
        ],
        output = "screen",
    )

    return LaunchDescription(args + [detector, navigator])
