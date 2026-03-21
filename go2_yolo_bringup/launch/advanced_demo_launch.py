"""
advanced_demo_launch.py — Full advanced demo scene launch.

Launches the complete multi-agent simulation stack:
  1. Agent manager        — moves 4 person models in Gazebo
  2. Multi-agent detector — publishes /detected_objects with LOS occlusion
  3. Target predictor     — Kalman filter for occlusion coasting
  4. Navigator node       — follows target with velocity-lead + Kalman fallback
  5. Demo scenario        — timed phase orchestration script

Run AFTER gazebo_launch.py + navigation_launch.py are fully up.

Usage:
  ros2 launch go2_yolo_bringup advanced_demo_launch.py
  ros2 launch go2_yolo_bringup advanced_demo_launch.py world_file:=advanced_demo_world.world
  ros2 launch go2_yolo_bringup advanced_demo_launch.py target_model:=person_2
  ros2 launch go2_yolo_bringup advanced_demo_launch.py phase_duration_s:=30.0

Monitor:
  ros2 topic echo /agent_manager/narrative
  ros2 topic echo /navigator/status
  ros2 topic echo /detector/predictor_state
  ros2 topic echo /sim_detector/status
  ros2 topic echo /demo/phase
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Arguments ──────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument("use_sim_time",       default_value="true"),
        DeclareLaunchArgument("target_model",       default_value="person_1",
                              description="Gazebo model name of the target person"),
        DeclareLaunchArgument("target_class",       default_value="owner_person",
                              description="Class name the navigator and predictor track"),
        DeclareLaunchArgument("walk_speed",         default_value="0.7",
                              description="Agent walking speed in m/s"),
        DeclareLaunchArgument("phase_duration_s",   default_value="20.0",
                              description="Default duration per demo phase"),
        DeclareLaunchArgument("los_check",          default_value="true",
                              description="Enable line-of-sight occlusion in detector"),
        DeclareLaunchArgument("use_velocity_lead",  default_value="true",
                              description="Navigator uses velocity lead to intercept moving targets"),
        DeclareLaunchArgument("detection_topic",    default_value="/detector/predicted_target",
                              description="Topic navigator subscribes to (predicted or raw)"),
    ]

    use_sim_time      = LaunchConfiguration("use_sim_time")
    target_model      = LaunchConfiguration("target_model")
    target_class      = LaunchConfiguration("target_class")
    walk_speed        = LaunchConfiguration("walk_speed")
    phase_duration    = LaunchConfiguration("phase_duration_s")
    los_check         = LaunchConfiguration("los_check")
    use_vel_lead      = LaunchConfiguration("use_velocity_lead")
    detection_topic   = LaunchConfiguration("detection_topic")

    # ── 1. Agent manager — moves all 4 persons in Gazebo ─────────────────
    agent_manager = Node(
        package    = "go2_yolo_detector",
        executable = "agent_manager",
        name       = "agent_manager",
        parameters = [
            {
                "target_id":    target_model,
                "walk_speed_ms": walk_speed,
                "update_hz":    20.0,
                "use_sim_time": use_sim_time,
            }
        ],
        output = "screen",
    )

    # ── 2. Multi-agent sim detector — LOS occlusion awareness ────────────
    sim_detector = Node(
        package    = "go2_yolo_detector",
        executable = "multi_agent_sim_detector",
        name       = "multi_agent_sim_detector",
        parameters = [
            {
                "target_model_name": target_model,
                "detection_rate_hz": 10.0,
                "los_check":         los_check,
                "robot_radius_m":    0.3,
                "use_sim_time":      use_sim_time,
            }
        ],
        output = "screen",
    )

    # ── 3. Target predictor — Kalman filter for occlusion coasting ────────
    predictor = Node(
        package    = "go2_yolo_detector",
        executable = "target_predictor",
        name       = "target_predictor",
        parameters = [
            {
                "target_class":         target_class,
                "coast_timeout_s":      5.0,
                "min_coast_conf":       0.25,
                "process_noise_q":      0.5,
                "measurement_noise_r":  0.2,
                "prediction_hz":        10.0,
                "use_sim_time":         use_sim_time,
            }
        ],
        output = "screen",
    )

    # ── 4. Navigator — velocity-lead, uses predicted detections ───────────
    navigator = Node(
        package    = "go2_yolo_navigator",
        executable = "navigator_node",
        name       = "navigator_node",
        parameters = [
            {
                "target_class":       target_class,
                "min_confidence":     0.25,     # low enough to use predictions
                "goal_offset_m":      0.8,
                "replan_distance_m":  0.3,
                "goal_cooldown_s":    2.0,
                "max_goal_age_s":     30.0,
                "use_velocity_lead":  use_vel_lead,
                "velocity_lead_s":    1.0,
                "detection_topic":    detection_topic,
                "use_sim_time":       use_sim_time,
            }
        ],
        output = "screen",
    )

    # NOTE: demo_scenario runs separately:
    #   ros2 run go2_yolo_bringup demo_scenario.py

    return LaunchDescription(args + [
        agent_manager,
        sim_detector,
        predictor,
        navigator,
    ])
