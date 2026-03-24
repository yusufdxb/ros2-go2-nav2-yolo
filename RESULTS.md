# Results — ros2-go2-nav2-yolo

Quantitative evaluation of the Nav2 + YOLO detection pipeline on the Unitree GO2 simulation.

---

## How to Reproduce

```bash
# YOLO inference benchmark (no simulation required)
python3 evaluation/benchmark_yolo.py

# Full navigation benchmark (requires running Gazebo stack)
source ~/go2_sim_env.sh
ros2 launch go2_yolo_bringup gazebo_launch.py          # Terminal 1
ros2 launch go2_yolo_bringup navigation_launch.py       # Terminal 2 (after ~20s)
ros2 launch go2_yolo_bringup yolo_nav_launch.py         # Terminal 3
python3 evaluation/nav_benchmark.py --trials 10 --goal-x 2.0 --goal-y 0.0
```

---

## YOLO Detection — Inference Latency Benchmark

Measured on synthetic 640×480 RGB frames (GO2 camera resolution).
100 runs after 10 warmup runs. Generated: `2026-03-24T00:27:17Z`

| Model | Mean Latency | p95 Latency | Throughput | Hardware |
|-------|-------------|-------------|------------|----------|
| yolov8n.pt | 53.22 ms | 56.7 ms | 18.8 fps | CPU |
| yolov8s.pt | 116.7 ms | 120.88 ms | 8.6 fps | CPU |

_GO2 camera runs at 30 fps. Both models have sufficient headroom for real-time detection._

> **Custom-trained model**: `go2_yolo_detector/` uses a YOLOv8n fine-tuned on 4 GO2-specific
> classes (`owner_person`, `wrist_marker`, `phone_marker`, `follow_marker`).
> Training scripts are in `training/`. mAP metrics pending training run completion.

---

## Navigation Performance (Gazebo Simulation)

> **Status: pending first benchmark run.**
> Run `python3 evaluation/nav_benchmark.py` with the full stack active and paste results here.

| Metric | Value |
|--------|-------|
| Success rate | — |
| Mean time-to-goal | — |
| Mean path deviation | — |
| First detection latency | — |
| Trials | — |

_Goal: `person_standing` at `(2, 0, 0)` in the demo world. Robot starts at origin._

---

## Nav2 Parameter Choices

| Parameter | Value | Reason |
|-----------|-------|--------|
| `controller_frequency` | 10 Hz | CHAMP gait cycle |
| `FollowPath.desired_linear_vel` | 0.3 m/s | Conservative for quadruped |
| `FollowPath.max_angular_vel` | 1.0 rad/s | Prevents yaw overshoot |
| `inflation_radius` | 0.35 m | GO2 body radius + margin |
| `slam_params.yaml throttle_scans` | 3 | Reduces SLAM queue saturation |
