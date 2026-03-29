#!/usr/bin/env python3
"""
demo_scenario.py — Orchestration script for the GO2 advanced Gazebo demo.

Implements a scripted 7-phase demonstration sequence that showcases:
  Phase 1 — INITIALISATION   Robot and all agents spawn; camera overview
  Phase 2 — SINGLE TARGET    Robot acquires and follows person_1 alone
  Phase 3 — DISTRACTORS      Three decoy agents walk into scene; robot must ignore them
  Phase 4 — OCCLUSION        Target walks behind wall; robot coasts on prediction
  Phase 5 — REACQUISITION    Target re-emerges; robot reacquires and continues
  Phase 6 — SPEED CHALLENGE  Target accelerates; robot must keep up
  Phase 7 — ARRIVAL          Target stops; robot approaches and halts at offset

Execution:
  # In a separate terminal after Gazebo + Nav2 are running:
  ros2 run go2_yolo_bringup demo_scenario.py

  # Or with faster pacing:
  ros2 run go2_yolo_bringup demo_scenario.py --ros-args -p phase_duration_s:=15.0

ROS2 parameters:
  phase_duration_s   float  Default duration for each phase [20.0]
  auto_advance       bool   Automatically advance phases on timer [true]
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ── Demo phase definitions ────────────────────────────────────────────────────

PHASES = [
    {
        "id":   1,
        "name": "INITIALISATION",
        "desc": "All agents spawned. Robot acquiring map. Waiting for Nav2 ready.",
        "duration": 15.0,
    },
    {
        "id":   2,
        "name": "SINGLE_TARGET_FOLLOW",
        "desc": "person_1 walks NE path. Robot locks on and follows.",
        "duration": 25.0,
        "commands": [
            ("agent",     "freeze:person_2"),
            ("agent",     "freeze:person_3"),
            ("agent",     "freeze:person_4"),
            ("narrative", "Phase 2: Single target following initiated"),
        ],
    },
    {
        "id":   3,
        "name": "DISTRACTORS",
        "desc": "Three decoy agents unfreeze and walk through scene. Robot must ignore them.",
        "duration": 25.0,
        "commands": [
            ("agent",     "unfreeze"),
            ("narrative", "Phase 3: Decoy agents activated — robot must maintain target lock"),
        ],
    },
    {
        "id":   4,
        "name": "OCCLUSION",
        "desc": "Target walks behind box_1 and central wall. Robot coasts on Kalman prediction.",
        "duration": 20.0,
        "commands": [
            # Teleport target to a position that will route it behind box_1
            ("agent",     "teleport:person_1:2.5:5.5"),
            ("narrative", "Phase 4: Target entering occlusion zone — Kalman filter active"),
        ],
    },
    {
        "id":   5,
        "name": "REACQUISITION",
        "desc": "Target re-emerges on east side. Robot reacquires lock.",
        "duration": 20.0,
        "commands": [
            ("agent",     "teleport:person_1:7.0:3.0"),
            ("narrative", "Phase 5: Target reacquired — lock restored"),
        ],
    },
    {
        "id":   6,
        "name": "SPEED_CHALLENGE",
        "desc": "Target accelerates to 1.4 m/s. Robot uses velocity-lead goal placement.",
        "duration": 25.0,
        "commands": [
            ("agent",     "set_speed:1.4"),
            ("navigator", "set_lead:1.5"),
            ("narrative", "Phase 6: Target acceleration — velocity-lead navigation engaged"),
        ],
    },
    {
        "id":   7,
        "name": "ARRIVAL",
        "desc": "Target stops. Robot approaches and halts at 0.8m standoff.",
        "duration": 15.0,
        "commands": [
            ("agent",     "freeze:person_1"),
            ("agent",     "set_speed:0.0"),
            ("narrative", "Phase 7: Target stationary — robot approaching final position"),
        ],
    },
]


# ── Orchestrator node ─────────────────────────────────────────────────────────

class DemoScenario(Node):

    def __init__(self):
        super().__init__("demo_scenario")

        self.declare_parameter("phase_duration_s", 20.0)
        self.declare_parameter("auto_advance",      True)

        self.default_duration = self.get_parameter("phase_duration_s").value
        self.auto_advance     = self.get_parameter("auto_advance").value

        # ── Publishers ────────────────────────────────────────────────────────
        self.agent_cmd_pub  = self.create_publisher(String, "/agent_manager/command",   10)
        self.nav_cmd_pub    = self.create_publisher(String, "/navigator/target_class",  10)
        self.narrative_pub  = self.create_publisher(String, "/agent_manager/narrative", 10)
        self.phase_pub      = self.create_publisher(String, "/demo/phase",              10)

        # State
        self.current_phase_idx = -1
        self.phase_start_time  = 0.0

        # Advance timer — checks for phase timeout
        self.create_timer(1.0, self._tick)

        # Brief startup delay so subscribers can connect
        self.create_timer(3.0, self._start_demo)

        self.get_logger().info(
            f"DemoScenario ready. {len(PHASES)} phases, "
            f"default duration {self.default_duration}s"
        )

    # ── Sequence control ──────────────────────────────────────────────────────

    def _start_demo(self):
        self.get_logger().info("=== DEMO START ===")
        self._advance_phase()

    def _tick(self):
        if self.current_phase_idx < 0:
            return

        phase    = PHASES[self.current_phase_idx]
        duration = phase.get("duration", self.default_duration)
        elapsed  = time.monotonic() - self.phase_start_time

        if self.auto_advance and elapsed >= duration:
            self._advance_phase()

    def _advance_phase(self):
        self.current_phase_idx += 1

        if self.current_phase_idx >= len(PHASES):
            self.get_logger().info("=== DEMO COMPLETE ===")
            self._publish_narrative("Demo complete. All phases executed.")
            self._publish_phase("COMPLETE")
            return

        phase = PHASES[self.current_phase_idx]
        self.phase_start_time = time.monotonic()

        self.get_logger().info(
            f"\n{'='*60}\n"
            f"PHASE {phase['id']}: {phase['name']}\n"
            f"{phase['desc']}\n"
            f"Duration: {phase.get('duration', self.default_duration):.0f}s\n"
            f"{'='*60}"
        )

        self._publish_phase(f"{phase['id']}:{phase['name']}")

        # Execute commands
        for cmd_type, payload in phase.get("commands", []):
            self._execute_command(cmd_type, payload)

    def _execute_command(self, cmd_type: str, payload: str):
        if cmd_type == "agent":
            msg = String()
            msg.data = payload
            self.agent_cmd_pub.publish(msg)
            self.get_logger().info(f"  [agent_cmd] {payload}")

        elif cmd_type == "narrative":
            self._publish_narrative(payload)

        elif cmd_type == "navigator":
            # e.g. "set_lead:1.5" — future navigator runtime commands
            msg = String()
            msg.data = payload
            self.nav_cmd_pub.publish(msg)
            self.get_logger().info(f"  [nav_cmd] {payload}")

    def _publish_narrative(self, text: str):
        msg = String()
        msg.data = text
        self.narrative_pub.publish(msg)
        self.get_logger().info(f"[Narrative] {text}")

    def _publish_phase(self, phase_str: str):
        msg = String()
        msg.data = phase_str
        self.phase_pub.publish(msg)

    # ── Manual phase advance via topic ────────────────────────────────────────

    def _manual_advance_cb(self, msg: String):
        if msg.data.strip().lower() == "next":
            self.get_logger().info("Manual advance triggered")
            self._advance_phase()
        elif msg.data.strip().lower() == "reset":
            self.current_phase_idx = -1
            self._publish_narrative("Demo reset")
            self._advance_phase()


def main(args=None):
    rclpy.init(args=args)
    node = DemoScenario()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
