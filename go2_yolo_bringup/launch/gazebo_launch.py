"""
GO2 Nav2 YOLO Demo — Gazebo Launch
Spawns the Unitree GO2 in Gazebo Classic using the CHAMP quadruped controller.

This is based on arjun-sadananda/go2_nav2_ros2 with additions:
  - RGB-D camera plugin for YOLOv8
  - Depth image topic remapped for the detector node

Usage:
  ros2 launch go2_yolo_bringup gazebo_launch.py
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_file = LaunchConfiguration("world_file")

    declare_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    declare_world = DeclareLaunchArgument(
        "world_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("go2_yolo_bringup"), "worlds", "demo_world.world"
        ]),
    )

    # ── Robot description (xacro → URDF) ────────────────────────────────
    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([
                FindPackageShare("go2_description"), "xacro", "go2_robot.xacro"
            ]),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # ── Gazebo ────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"
            ])
        ]),
        launch_arguments={
            "world": world_file,
            "verbose": "false",
        }.items(),
    )

    # ── Spawn GO2 ─────────────────────────────────────────────────────────
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "go2",
            "-topic", "/robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.5",
        ],
        output="screen",
    )

    # ── CHAMP bringup (quadruped controller + state estimation + EKF) ────
    # NOTE: champ_config is the go2-specific config package from
    # arjun-sadananda/go2_nav2_ros2. Clone alongside this repo.
    champ_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("champ_bringup"), "launch", "bringup.launch.py"
            ])
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_name": "go2",
            "gazebo": "true",
            "rviz": "false",
            "config_path": PathJoinSubstitution([
                FindPackageShare("go2_yolo_bringup"), "config"
            ]),
        }.items(),
    )

    # ── ros2_control controller manager ──────────────────────────────────
    load_joint_effort_controller = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "joint_group_effort_controller",
        ],
        output="screen",
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    return LaunchDescription([
        declare_sim_time,
        declare_world,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        champ_bringup,
        load_joint_state_broadcaster,
        load_joint_effort_controller,
    ])
