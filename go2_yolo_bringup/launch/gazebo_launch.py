"""
GO2 Nav2 YOLO Demo — Gazebo Launch
Spawns the Unitree GO2 in Gazebo Classic using the CHAMP quadruped controller.

This launch file runs robot_state_publisher directly (instead of via
champ_description) so the URDF can be pre-processed:
  - The <?xml?> declaration is stripped because gazebo_ros2_control in
    Humble fails to parse it via rcl's parameter override parser.
  - The full URDF (including <ros2_control> tags) is written to a temp
    file for spawn_entity.py -file, avoiding the topic-based path where
    RSP's URDF parser strips non-standard elements.

Usage:
  ros2 launch go2_yolo_bringup gazebo_launch.py
"""

import os
import re
import subprocess
import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_description_path(go2_desc_share: str) -> str:
    env_override = os.environ.get("GO2_DESCRIPTION_XACRO")
    if env_override:
        return env_override

    installed_path = os.path.join(go2_desc_share, "xacro", "go2_robot.xacro")

    launch_path = Path(__file__).resolve()
    for parent in launch_path.parents:
        candidate = parent / "src" / "go2_description" / "xacro" / "go2_robot.xacro"
        if candidate.is_file():
            return str(candidate)

    return installed_path


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    go2_yolo_share = get_package_share_directory("go2_yolo_bringup")
    go2_desc_share = get_package_share_directory("go2_description")

    declare_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    declare_world = DeclareLaunchArgument(
        "world_file",
        default_value=os.path.join(go2_yolo_share, "worlds", "demo_world.world"),
    )
    declare_description_path = DeclareLaunchArgument(
        "description_path",
        default_value=_default_description_path(go2_desc_share),
    )

    # ── Generate URDF at launch time ─────────────────────────────────────
    description_xacro = _default_description_path(go2_desc_share)
    urdf_xml = subprocess.check_output(
        ["xacro", description_xacro], text=True
    )
    # Strip <?xml?> declaration AND all XML comments from the URDF.
    # gazebo_ros2_control 0.4.x (Humble) passes robot_description through
    # rcl's CLI argument parser ('--param robot_description:=...') which
    # chokes on XML comments containing '--', ':=', and other sequences.
    urdf_xml_clean = re.sub(r"<!--.*?-->", "", urdf_xml, flags=re.DOTALL)
    urdf_xml_clean = "\n".join(
        line for line in urdf_xml_clean.splitlines()
        if line.strip() and not line.strip().startswith("<?xml")
    )

    # Write full URDF (with ros2_control tags) for spawn_entity
    urdf_file = os.path.join(tempfile.gettempdir(), "go2_spawn.urdf")
    with open(urdf_file, "w") as f:
        f.write(urdf_xml_clean)

    # ── Gazebo ────────────────────────────────────────────────────────────
    world_file_path = os.path.join(go2_yolo_share, "worlds", "demo_world.world")

    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            world_file_path,
            "-slibgazebo_ros_init.so",
            "-slibgazebo_ros_factory.so",
            "-slibgazebo_ros_force_system.so",
            "-slibgazebo_ros_state.so",
        ],
        additional_env={
            "LIBGL_DRI3_DISABLE": "1",
            "OGRE_RTT_MODE": "Copy",
            "DISPLAY": ":1",
        },
        output="screen",
    )

    gzclient = ExecuteProcess(
        cmd=["gzclient", "--gui-client-plugin=libgazebo_ros_eol_gui.so"],
        additional_env={
            "LIBGL_DRI3_DISABLE": "1",
            "OGRE_RTT_MODE": "Copy",
            "DISPLAY": ":1",
        },
        output="screen",
    )

    delayed_gzclient = TimerAction(period=6.0, actions=[gzclient])

    # ── Robot State Publisher ─────────────────────────────────────────────
    # Run RSP directly with the clean URDF (declaration stripped) so
    # gazebo_ros2_control can read robot_description without parser errors.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": urdf_xml_clean},
            {"use_tf_static": False},
            {"publish_frequency": 200.0},
            {"ignore_timestamp": True},
            {"use_sim_time": True},
        ],
        output="screen",
    )

    # ── Spawn GO2 ─────────────────────────────────────────────────────────
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "go2",
            "-file", urdf_file,
            "-x", "0.0", "-y", "0.0", "-z", "0.5",
            "-timeout", "60",
        ],
        output="screen",
    )

    # ── CHAMP nodes (controller + state estimation + EKF) ─────────────────
    # Run CHAMP nodes directly (not via bringup.launch.py which includes
    # its own RSP that would conflict with ours).
    quadruped_controller = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"gazebo": True},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": True},
            {"joint_controller_topic": "joint_group_effort_controller/joint_trajectory"},
            {"urdf": urdf_xml},
            os.path.join(go2_yolo_share, "config", "joints", "joints.yaml"),
            os.path.join(go2_yolo_share, "config", "links", "links.yaml"),
            os.path.join(go2_yolo_share, "config", "gait", "gait.yaml"),
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    state_estimator = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"orientation_from_imu": False},
            {"urdf": urdf_xml},
            os.path.join(go2_yolo_share, "config", "joints", "joints.yaml"),
            os.path.join(go2_yolo_share, "config", "links", "links.yaml"),
            os.path.join(go2_yolo_share, "config", "gait", "gait.yaml"),
        ],
    )

    base_to_footprint_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="base_to_footprint_ekf",
        output="screen",
        parameters=[
            {"base_link_frame": "base_link"},
            {"use_sim_time": True},
            os.path.join(
                get_package_share_directory("champ_base"),
                "config", "ekf", "base_to_footprint.yaml",
            ),
        ],
        remappings=[("odometry/filtered", "odom/local")],
    )

    footprint_to_odom_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="footprint_to_odom_ekf",
        output="screen",
        parameters=[
            {"base_link_frame": "base_link"},
            {"use_sim_time": True},
            os.path.join(
                get_package_share_directory("champ_base"),
                "config", "ekf", "footprint_to_odom.yaml",
            ),
        ],
        remappings=[("odometry/filtered", "odom")],
    )

    # ── ros2_control controllers ──────────────────────────────────────────
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

    delayed_spawn = TimerAction(period=10.0, actions=[spawn_robot])

    return LaunchDescription([
        declare_sim_time,
        declare_world,
        declare_description_path,
        gzserver,
        delayed_gzclient,
        robot_state_publisher,
        quadruped_controller,
        state_estimator,
        base_to_footprint_ekf,
        footprint_to_odom_ekf,
        delayed_spawn,
        delay_jsb,
        delay_jec,
    ])
