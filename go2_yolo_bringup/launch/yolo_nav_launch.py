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

    # Simulation: use Gazebo ground truth instead of YOLO (llvmpipe rendering
    # is too synthetic for YOLOv8 to detect the person_standing model).
    # For real hardware, swap this node back to detector_node with YOLO.
    detector = Node(
        package="go2_yolo_detector",
        executable="sim_person_detector",
        name="sim_person_detector",
        parameters=[
            {"model_name": "person_standing"},
            {"confidence": 1.0},
            {"use_sim_time": use_sim_time},
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
