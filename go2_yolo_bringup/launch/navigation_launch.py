"""
GO2 Nav2 YOLO Demo — Navigation Launch
Starts Nav2 and SLAM Toolbox for the GO2 simulation.

Run AFTER gazebo_launch.py is fully up (robot is spawned and walking).

Usage:
  ros2 launch go2_yolo_bringup navigation_launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_rviz = DeclareLaunchArgument("use_rviz", default_value="true")

    # ── Scan relay — subscribes BEST_EFFORT so DDS won't replay historical ──
    # Gazebo's scan publisher uses RELIABLE QoS, which causes DDS to buffer
    # all scans and dump the full history when SLAM subscribes. The relay
    # subscribes BEST_EFFORT (no history replay) and republishes to /scan_slam.
    scan_relay = Node(
        package="go2_yolo_bringup",
        executable="scan_relay.py",
        name="scan_relay",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── SLAM Toolbox (online async — builds map while navigating) ─────────
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch", "online_async_launch.py"
            ])
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": PathJoinSubstitution([
                FindPackageShare("go2_yolo_bringup"), "config", "slam_params.yaml"
            ]),
        }.items(),
    )

    # ── Nav2 ──────────────────────────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch", "navigation_launch.py"
            ])
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": PathJoinSubstitution([
                FindPackageShare("go2_yolo_bringup"), "config", "nav2_params.yaml"
            ]),
        }.items(),
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d", PathJoinSubstitution([
                FindPackageShare("go2_yolo_bringup"), "rviz", "go2_nav2_yolo.rviz"
            ])
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    # Delay SLAM by 10s so the odom→base_footprint TF chain is fully
    # established before SLAM subscribes to /scan. Without this delay,
    # SLAM's MessageFilter queue fills immediately because the odom frame
    # doesn't exist yet when SLAM first starts.
    slam_delayed = TimerAction(period=10.0, actions=[slam])

    return LaunchDescription([
        declare_sim_time,
        declare_rviz,
        scan_relay,
        nav2,
        slam_delayed,
        rviz,
    ])
