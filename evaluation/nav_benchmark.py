#!/usr/bin/env python3
"""
Navigation Benchmark — ros2-go2-nav2-yolo
==========================================
Logs per-run navigation metrics to CSV for evaluation of Nav2 + detection pipeline.

Subscribes to:
  /go2/odom or /odometry/filtered   — robot pose for path tracking
  /detected_objects                  — detection pipeline output
  /navigate_to_pose/_action/status   — Nav2 goal status

Records per-trial:
  - success/failure
  - time-to-goal (TTG) from goal send to goal reached
  - path deviation: mean lateral distance from straight-line path (m)
  - first detection latency: time from node start to first DetectedObject
  - goal distance: straight-line distance from start to goal (m)

Usage:
  source ~/go2_sim_env.sh
  # Terminal 1: launch Gazebo + Nav2 + YOLO pipeline
  # Terminal 2:
  python3 evaluation/nav_benchmark.py --trials 10 --goal-x 2.0 --goal-y 0.0

Output:
  evaluation/results/nav_<timestamp>.csv
  evaluation/results/nav_<timestamp>_summary.json
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

try:
    from go2_yolo_msgs.msg import DetectedObjectArray
    HAS_DETECTION_MSGS = True
except ImportError:
    HAS_DETECTION_MSGS = False


# ── Data structures ──────────────────────────────────────────────────────────

@dataclass
class NavTrialResult:
    trial_id: int
    goal_x: float
    goal_y: float
    start_x: float
    start_y: float
    goal_distance_m: float
    success: bool
    time_to_goal_sec: Optional[float]
    mean_path_deviation_m: Optional[float]
    first_detection_latency_sec: Optional[float]
    nav_status_code: int
    timestamp: str


# ── Benchmark node ───────────────────────────────────────────────────────────

class NavBenchmarkNode(Node):
    """Measures navigation performance for one trial."""

    ODOM_TOPICS = ["/odometry/filtered", "/go2/odom", "/odom"]

    def __init__(self):
        super().__init__("nav_benchmark_node")

        # Odometry subscriber — try multiple topic names
        self._pose: Optional[Tuple[float, float]] = None
        self._path_poses: List[Tuple[float, float]] = []
        self._odom_sub = self.create_subscription(
            Odometry, self.ODOM_TOPICS[0], self._odom_cb, 10
        )

        # Detection subscriber
        self._first_detection_time: Optional[float] = None
        if HAS_DETECTION_MSGS:
            self.create_subscription(
                DetectedObjectArray, "/detected_objects", self._detection_cb, 10
            )

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._pose = (x, y)
        self._path_poses.append((x, y))

    def _detection_cb(self, msg):
        if self._first_detection_time is None and msg.objects:
            self._first_detection_time = time.time()

    def send_goal(self, x: float, y: float) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        return goal

    def compute_path_deviation(
        self,
        path: List[Tuple[float, float]],
        start: Tuple[float, float],
        goal: Tuple[float, float],
    ) -> Optional[float]:
        """Mean perpendicular distance from poses to straight line start→goal."""
        if len(path) < 2:
            return None
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        line_len = math.sqrt(dx**2 + dy**2)
        if line_len < 1e-6:
            return 0.0
        # Perpendicular distance from each pose to the line
        deviations = []
        for px, py in path:
            # Cross product magnitude / line_len
            cross = abs((px - start[0]) * dy - (py - start[1]) * dx)
            deviations.append(cross / line_len)
        return sum(deviations) / len(deviations)


def run_trial(
    node: NavBenchmarkNode,
    trial_id: int,
    goal_x: float,
    goal_y: float,
    timeout_sec: float = 60.0,
) -> NavTrialResult:
    """Execute one navigation trial and return metrics."""
    node._path_poses.clear()
    node._first_detection_time = None

    # Wait for current pose
    deadline = time.time() + 5.0
    while node._pose is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    start_pose = node._pose or (0.0, 0.0)
    start_time = time.time()

    goal_distance = math.sqrt(
        (goal_x - start_pose[0])**2 + (goal_y - start_pose[1])**2
    )

    print(f"  [trial {trial_id}] start={start_pose} → goal=({goal_x},{goal_y}) "
          f"dist={goal_distance:.2f}m", end=" ", flush=True)

    if not node._nav_client.wait_for_server(timeout_sec=5.0):
        print("FAILED (nav2 not available)")
        return NavTrialResult(
            trial_id=trial_id, goal_x=goal_x, goal_y=goal_y,
            start_x=start_pose[0], start_y=start_pose[1],
            goal_distance_m=goal_distance, success=False,
            time_to_goal_sec=None, mean_path_deviation_m=None,
            first_detection_latency_sec=None,
            nav_status_code=-1,
            timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"),
        )

    goal_msg = node.send_goal(goal_x, goal_y)
    send_future = node._nav_client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=5.0)

    goal_handle = send_future.result()
    if not goal_handle or not goal_handle.accepted:
        print("FAILED (goal rejected)")
        return NavTrialResult(
            trial_id=trial_id, goal_x=goal_x, goal_y=goal_y,
            start_x=start_pose[0], start_y=start_pose[1],
            goal_distance_m=goal_distance, success=False,
            time_to_goal_sec=None, mean_path_deviation_m=None,
            first_detection_latency_sec=None,
            nav_status_code=-2,
            timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"),
        )

    result_future = goal_handle.get_result_async()
    deadline = time.time() + timeout_sec
    while not result_future.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    elapsed = time.time() - start_time
    success = False
    status_code = -3

    if result_future.done():
        result = result_future.result()
        status_code = result.status
        success = (status_code == GoalStatus.STATUS_SUCCEEDED)

    det_latency = None
    if node._first_detection_time is not None:
        det_latency = node._first_detection_time - start_time

    path_dev = node.compute_path_deviation(
        node._path_poses, start_pose, (goal_x, goal_y)
    )

    status_str = "SUCCESS" if success else f"FAILED (status={status_code})"
    print(f"{status_str} in {elapsed:.1f}s  dev={path_dev:.3f}m" if path_dev else status_str)

    return NavTrialResult(
        trial_id=trial_id, goal_x=goal_x, goal_y=goal_y,
        start_x=start_pose[0], start_y=start_pose[1],
        goal_distance_m=goal_distance,
        success=success,
        time_to_goal_sec=elapsed if success else None,
        mean_path_deviation_m=path_dev,
        first_detection_latency_sec=det_latency,
        nav_status_code=status_code,
        timestamp=time.strftime("%Y-%m-%dT%H:%M:%S"),
    )


def save_results(results: List[NavTrialResult], output_base: str):
    out = Path(output_base)
    out.parent.mkdir(parents=True, exist_ok=True)

    csv_path = out.with_suffix(".csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(asdict(results[0]).keys()))
        writer.writeheader()
        for r in results:
            writer.writerow(asdict(r))

    successes = [r for r in results if r.success]
    ttgs = [r.time_to_goal_sec for r in successes if r.time_to_goal_sec]
    devs = [r.mean_path_deviation_m for r in successes if r.mean_path_deviation_m]
    dets = [r.first_detection_latency_sec for r in results if r.first_detection_latency_sec]

    summary = {
        "n_trials": len(results),
        "success_rate": len(successes) / len(results),
        "mean_ttg_sec": sum(ttgs) / len(ttgs) if ttgs else None,
        "mean_path_deviation_m": sum(devs) / len(devs) if devs else None,
        "mean_first_detection_latency_sec": sum(dets) / len(dets) if dets else None,
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }

    json_path = out.with_suffix(".json")
    with open(json_path, "w") as f:
        json.dump(summary, f, indent=2)

    print(f"\n  Results: {csv_path}")
    print(f"  Summary: {json_path}")
    print(f"\n  Success rate: {summary['success_rate']*100:.0f}% ({len(successes)}/{len(results)})")
    if summary["mean_ttg_sec"]:
        print(f"  Mean TTG: {summary['mean_ttg_sec']:.1f}s")
    if summary["mean_path_deviation_m"]:
        print(f"  Mean path deviation: {summary['mean_path_deviation_m']:.3f}m")


def main():
    parser = argparse.ArgumentParser(description="Nav2 navigation benchmark")
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--goal-x", type=float, default=2.0)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=60.0,
                        help="Max seconds per trial")
    parser.add_argument("--output", type=str,
                        default=f"evaluation/results/nav_{time.strftime('%Y%m%d_%H%M')}")
    args = parser.parse_args()

    rclpy.init()
    node = NavBenchmarkNode()

    print(f"Nav2 Benchmark: {args.trials} trials → goal ({args.goal_x}, {args.goal_y})")

    results = []
    for i in range(args.trials):
        result = run_trial(node, i + 1, args.goal_x, args.goal_y, args.timeout)
        results.append(result)
        time.sleep(2.0)  # cooldown between trials

    save_results(results, args.output)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
