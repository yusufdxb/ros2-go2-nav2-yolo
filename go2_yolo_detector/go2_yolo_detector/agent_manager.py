#!/usr/bin/env python3
"""
agent_manager.py — Dynamic agent (person) movement system for Gazebo simulation.

Moves multiple person_standing models in Gazebo along waypoint paths to
simulate humans walking through the environment. Uses the Gazebo ROS state
plugin's /set_entity_state service to teleport models to successive positions
at each timer tick, creating smooth motion at human walking speed.

Architecture:
  - Each agent has a list of named waypoints and cycles through them
  - A shared ROS timer ticks at 20 Hz; each tick advances all agents
  - Agent positions published to /agent_manager/agent_states (PoseArray)
    and /agent_manager/target_pose (PoseStamped) for the designated target
  - target_id parameter names which model is the designated target

Published topics:
  /agent_manager/agent_states   — geometry_msgs/PoseArray (all agents, world frame)
  /agent_manager/target_pose    — geometry_msgs/PoseStamped (target agent only)
  /agent_manager/narrative      — std_msgs/String (human-readable state description)

ROS2 parameters:
  target_id        str    Model name of the designated target ['person_1']
  walk_speed_ms    float  Walking speed in m/s [0.7]
  waypoint_tol_m   float  Distance to switch to next waypoint [0.15]
  update_hz        float  Position update frequency [20.0]
  freeze_agents    bool   Pause all agent motion (for demo scripting) [false]

Requires: libgazebo_ros_state.so loaded (added in gazebo_launch.py via
  -slibgazebo_ros_state.so argument to gzserver).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from std_msgs.msg import String

try:
    from gazebo_msgs.srv import SetEntityState
    from gazebo_msgs.msg import EntityState
    _GAZEBO_AVAILABLE = True
except ImportError:
    _GAZEBO_AVAILABLE = False


# ── Waypoint definitions (world frame, x, y) ─────────────────────────────────
# Each agent cycles through its waypoints indefinitely.
# Designed for the advanced_demo_world.world (24m × 20m arena).

AGENT_WAYPOINTS: dict[str, list[tuple[float, float]]] = {
    "person_1": [           # Target — crosses the central corridor
        ( 4.0,  6.0),
        ( 4.0,  0.5),
        ( 2.0, -1.0),
        ( 8.0, -3.0),
        ( 9.0, -7.0),
        ( 4.0, -7.0),
        ( 2.0, -2.0),
        ( 0.5,  1.0),
        ( 4.0,  6.0),
    ],
    "person_2": [           # Decoy — stays mostly in NW zone
        (-8.0,  7.0),
        (-4.0,  5.0),
        (-6.0,  3.0),
        (-9.0,  5.0),
        (-8.0,  7.0),
    ],
    "person_3": [           # Decoy — patrols SW open area
        (-4.0, -5.0),
        (-2.0, -8.0),
        (-8.0, -8.0),
        (-9.0, -5.0),
        (-5.0, -3.0),
        (-4.0, -5.0),
    ],
    "person_4": [           # Decoy — patrols SE zone
        ( 8.0, -5.0),
        ( 9.0, -8.0),
        ( 6.0, -8.0),
        ( 6.0, -3.0),
        ( 8.0, -5.0),
    ],
}

AGENT_INIT_Z = 0.0          # ground level for person_standing model


# ── Agent state dataclass ─────────────────────────────────────────────────────

@dataclass
class AgentState:
    name: str
    waypoints: list[tuple[float, float]]
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    wp_idx: int = 0
    frozen: bool = False

    def __post_init__(self):
        if self.waypoints:
            self.x, self.y = self.waypoints[0]

    @property
    def current_waypoint(self) -> tuple[float, float]:
        return self.waypoints[self.wp_idx % len(self.waypoints)]

    def advance(self, speed_ms: float, dt: float, waypoint_tol: float):
        """Move toward current waypoint; advance to next if close enough."""
        if self.frozen or not self.waypoints:
            return

        tx, ty = self.current_waypoint
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)

        if dist < waypoint_tol:
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            return

        step = min(speed_ms * dt, dist)
        self.yaw = math.atan2(dy, dx)
        self.x += (dx / dist) * step
        self.y += (dy / dist) * step


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class AgentManager(Node):

    def __init__(self):
        super().__init__("agent_manager")

        self.declare_parameter("target_id",     "person_1")
        self.declare_parameter("walk_speed_ms",  0.7)
        self.declare_parameter("waypoint_tol_m", 0.15)
        self.declare_parameter("update_hz",      20.0)
        self.declare_parameter("freeze_agents",  False)

        self.target_id    = self.get_parameter("target_id").value
        self.walk_speed   = self.get_parameter("walk_speed_ms").value
        self.wp_tol       = self.get_parameter("waypoint_tol_m").value
        update_hz         = self.get_parameter("update_hz").value
        self.freeze_all   = self.get_parameter("freeze_agents").value

        self.dt = 1.0 / update_hz

        # ── Build agent states ────────────────────────────────────────────────
        self.agents: dict[str, AgentState] = {}
        for name, wps in AGENT_WAYPOINTS.items():
            self.agents[name] = AgentState(name=name, waypoints=wps)

        # ── Gazebo service client ─────────────────────────────────────────────
        self._gz_client = None
        if _GAZEBO_AVAILABLE:
            self._gz_client = self.create_client(SetEntityState, "/set_entity_state")
            # Non-blocking check — service may appear later; _send_to_gazebo
            # guards on is_service_ready() each call so this is safe.
            if self._gz_client.service_is_ready():
                self.get_logger().info("Gazebo /set_entity_state service connected")
            else:
                self.get_logger().info(
                    "/set_entity_state not ready yet — will retry each tick"
                )
        else:
            self.get_logger().warn(
                "gazebo_msgs not available — running in publish-only mode (no Gazebo movement)"
            )

        # ── Publishers ────────────────────────────────────────────────────────
        self.states_pub    = self.create_publisher(PoseArray,    "/agent_manager/agent_states",  10)
        self.target_pub    = self.create_publisher(PoseStamped,  "/agent_manager/target_pose",   10)
        self.narrative_pub = self.create_publisher(String,       "/agent_manager/narrative",     10)

        # Subscriptions for runtime demo control
        self.create_subscription(String, "/agent_manager/command", self._cmd_callback, 10)

        # ── Main timer ────────────────────────────────────────────────────────
        self.create_timer(self.dt, self._update)

        self.get_logger().info(
            f"AgentManager ready: {len(self.agents)} agents, target='{self.target_id}', "
            f"speed={self.walk_speed} m/s"
        )

    # ── Timer callback ────────────────────────────────────────────────────────

    def _update(self):
        now = self.get_clock().now().to_msg()

        # Advance all agent positions
        for agent in self.agents.values():
            if not self.freeze_all:
                agent.advance(self.walk_speed, self.dt, self.wp_tol)
            self._send_to_gazebo(agent)

        # Publish PoseArray for all agents
        pa = PoseArray()
        pa.header.stamp    = now
        pa.header.frame_id = "map"
        for agent in self.agents.values():
            p = Pose()
            p.position.x = agent.x
            p.position.y = agent.y
            p.position.z = AGENT_INIT_Z
            half_yaw = agent.yaw / 2.0
            p.orientation.z = math.sin(half_yaw)
            p.orientation.w = math.cos(half_yaw)
            pa.poses.append(p)
        self.states_pub.publish(pa)

        # Publish target pose separately
        if self.target_id in self.agents:
            t = self.agents[self.target_id]
            ps = PoseStamped()
            ps.header.stamp    = now
            ps.header.frame_id = "map"
            ps.pose.position.x = t.x
            ps.pose.position.y = t.y
            ps.pose.position.z = AGENT_INIT_Z
            half_yaw = t.yaw / 2.0
            ps.pose.orientation.z = math.sin(half_yaw)
            ps.pose.orientation.w = math.cos(half_yaw)
            self.target_pub.publish(ps)

    # ── Gazebo model teleportation ─────────────────────────────────────────────

    def _send_to_gazebo(self, agent: AgentState):
        if self._gz_client is None or not _GAZEBO_AVAILABLE:
            return
        if not self._gz_client.service_is_ready():
            return

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = agent.name
        req.state.reference_frame = "world"
        req.state.pose.position.x = agent.x
        req.state.pose.position.y = agent.y
        req.state.pose.position.z = AGENT_INIT_Z
        half_yaw = agent.yaw / 2.0
        req.state.pose.orientation.z = math.sin(half_yaw)
        req.state.pose.orientation.w = math.cos(half_yaw)

        # Fire-and-forget async call — don't block the timer
        future = self._gz_client.call_async(req)
        future.add_done_callback(self._gz_cb)

    def _gz_cb(self, future):
        try:
            result = future.result()
            if not result.success:
                self.get_logger().debug("set_entity_state returned failure")
        except Exception as e:
            self.get_logger().debug(f"set_entity_state call failed: {e}")

    # ── Runtime command handler ────────────────────────────────────────────────

    def _cmd_callback(self, msg: String):
        """
        Accepts commands on /agent_manager/command:
          freeze                      — stop all agent movement
          unfreeze                    — resume all agent movement
          freeze:<name>               — stop specific agent
          set_target:<name>           — change the designated target
          teleport:<name>:<x>:<y>     — instant teleport agent to position
          set_speed:<value>           — change walk speed for all agents
        """
        cmd = msg.data.strip()
        parts = cmd.split(":")

        if parts[0] == "freeze" and len(parts) == 1:
            self.freeze_all = True
            self._publish_narrative("All agents frozen")

        elif parts[0] == "unfreeze":
            self.freeze_all = False
            self._publish_narrative("All agents moving")

        elif parts[0] == "freeze" and len(parts) == 2:
            name = parts[1]
            if name in self.agents:
                self.agents[name].frozen = True
                self._publish_narrative(f"{name} frozen")

        elif parts[0] == "set_target" and len(parts) == 2:
            self.target_id = parts[1]
            self._publish_narrative(f"Target changed to {self.target_id}")

        elif parts[0] == "teleport" and len(parts) == 4:
            name, x, y = parts[1], float(parts[2]), float(parts[3])
            if name in self.agents:
                self.agents[name].x = x
                self.agents[name].y = y
                self._publish_narrative(f"{name} teleported to ({x:.1f}, {y:.1f})")

        elif parts[0] == "set_speed" and len(parts) == 2:
            self.walk_speed = float(parts[1])
            self._publish_narrative(f"Walk speed set to {self.walk_speed:.2f} m/s")

        elif parts[0] == "accelerate_target":
            self.agents[self.target_id].frozen = False
            self.walk_speed = float(parts[1]) if len(parts) > 1 else self.walk_speed * 1.5
            self._publish_narrative(f"Target speed: {self.walk_speed:.2f} m/s")

    def _publish_narrative(self, text: str):
        msg = String()
        msg.data = text
        self.narrative_pub.publish(msg)
        self.get_logger().info(f"[Demo] {text}")


def main(args=None):
    rclpy.init(args=args)
    node = AgentManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
