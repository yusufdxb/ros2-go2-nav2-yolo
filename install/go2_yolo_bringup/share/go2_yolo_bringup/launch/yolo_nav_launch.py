"""
GO2 Nav2 YOLO Demo — YOLO + Navigator Launch
Starts the YOLOv8 detector and the Nav2 goal publisher.

Run AFTER both gazebo_launch.py and navigation_launch.py are running.

Usage:
  ros2 launch go2_yolo_bringup yolo_nav_launch.py
  ros2 launch go2_yolo_bringup yolo_nav_launch.py target_class:=chair
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    target_class = LaunchConfiguration("target_class")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_target = DeclareLaunchArgument(
        "target_class", default_value="person",
        description="YOLO class to navigate toward (e.g. person, chair, bottle)"
    )
    declare_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")

    detector = Node(
        package="go2_yolo_detector",
        executable="detector_node",
        name="detector_node",
        parameters=[
            {"model_path": "yolov8n.pt"},
            {"confidence_threshold": 0.45},
            {"max_depth_m": 10.0},
            {"publish_visualization": True},
            {"use_sim_time": use_sim_time},
        ],
        # Remap to the GO2's Gazebo camera topics
        remappings=[
            ("/camera/image_raw",       "/go2/camera/image_raw"),
            ("/camera/depth/image_raw", "/go2/camera/depth/image_raw"),
            ("/camera/camera_info",     "/go2/camera/camera_info"),
        ],
        output="screen",
    )

    navigator = Node(
        package="go2_yolo_navigator",
        executable="navigator_node",
        name="navigator_node",
        parameters=[
            {"target_class": target_class},
            {"min_confidence": 0.5},
            {"goal_offset_m": 0.8},
            {"replan_distance_m": 0.4},
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    return LaunchDescription([
        declare_target,
        declare_sim_time,
        detector,
        navigator,
    ])
