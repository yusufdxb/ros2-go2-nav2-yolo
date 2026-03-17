"""
GO2 Nav2 YOLO Demo — Gazebo Launch
Spawns the Unitree GO2 in Gazebo Classic using the CHAMP quadruped controller.

This is based on arjun-sadananda/go2_nav2_ros2 with additions:
  - RGB-D camera plugin for YOLOv8
  - Depth image topic remapped for the detector node

Usage:
  ros2 launch go2_yolo_bringup gazebo_launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_file = LaunchConfiguration("world_file")

    # Eagerly resolve package paths so they can be used as param-file paths
    # (PathJoinSubstitution can evaluate to '.' when passed through
    #  launch_arguments into included files that use LaunchConfiguration
    #  directly as a --params-file path).
    go2_yolo_share = get_package_share_directory("go2_yolo_bringup")
    go2_desc_share = get_package_share_directory("go2_description")

    declare_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    declare_world = DeclareLaunchArgument(
        "world_file",
        default_value=os.path.join(go2_yolo_share, "worlds", "demo_world.world"),
    )

    # ── Gazebo ────────────────────────────────────────────────────────────
    # NOTE: robot_state_publisher is started by champ_bringup (via
    # champ_description/description.launch.py) with description_path set to
    # go2_robot.xacro below. Running a second RSP here causes competing TF
    # publications for base_link which breaks SLAM toolbox's transform cache.
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
            "-timeout", "60",
        ],
        output="screen",
    )

    # ── CHAMP bringup (quadruped controller + state estimation + EKF) ────
    # Use os.path.join (not PathJoinSubstitution) for the three config paths
    # because bringup.launch.py passes them directly as --params-file values
    # via LaunchConfiguration(); PathJoinSubstitution evaluates too late and
    # lands as '.' in that context.
    champ_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_bringup"),
                "launch", "bringup.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_name": "go2",
            "gazebo": "true",
            "rviz": "false",
            "description_path": os.path.join(
                go2_desc_share, "xacro", "go2_robot.xacro"
            ),
            "joints_map_path": os.path.join(
                go2_yolo_share, "config", "joints", "joints.yaml"
            ),
            "links_map_path": os.path.join(
                go2_yolo_share, "config", "links", "links.yaml"
            ),
            "gait_config_path": os.path.join(
                go2_yolo_share, "config", "gait", "gait.yaml"
            ),
        }.items(),
    )

    # ── ros2_control controllers — spawned after robot is in Gazebo ──────
    # controller_manager becomes available only after gazebo_ros2_control
    # plugin loads (i.e., after spawn_entity exits successfully).
    # Controller names must match go2_description/config/go2_ros_control.yaml
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_states_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    joint_effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_effort_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Chain: spawn_robot exits → load joint_states_controller
    #        joint_states_controller exits → load joint_group_effort_controller
    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    delay_jec = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_effort_controller_spawner],
        )
    )

    return LaunchDescription([
        declare_sim_time,
        declare_world,
        gazebo,
        spawn_robot,
        champ_bringup,
        delay_jsb,
        delay_jec,
    ])
