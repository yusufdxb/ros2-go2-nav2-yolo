# Results — ros2-go2-nav2-yolo

Quantitative evaluation of the Nav2 + YOLO detection pipeline on the Unitree GO2 simulation.

## How to Reproduce

```bash
# Step 1 — launch Gazebo + Nav2 + detection pipeline
source ~/go2_sim_env.sh
ros2 launch go2_yolo_bringup gazebo_launch.py          # Terminal 1
ros2 launch go2_yolo_bringup navigation_launch.py       # Terminal 2 (after ~20s)
ros2 launch go2_yolo_bringup yolo_nav_launch.py         # Terminal 3

# Step 2 — run navigation benchmark
source ~/go2_sim_env.sh
python3 evaluation/nav_benchmark.py \
    --trials 10 \
    --goal-x 2.0 \
    --goal-y 0.0 \
    --output evaluation/results/nav_$(date +%Y%m%d_%H%M)
```

---

## Navigation Performance (Gazebo Simulation)

> **Status: pending first benchmark run.**
> Run `python3 evaluation/nav_benchmark.py` with the full stack active and paste results here.

| Metric | Value |
|--------|-------|
| Success rate | — |
| Mean time-to-goal (TTG) | — |
| Mean path deviation | — |
| First detection latency | — |
| Trials | — |

_Goal: `person_standing` at `(2, 0, 0)` in the demo world. Robot starts at origin._

---

## YOLO Detection Performance (Real Hardware)

The `detector_node.py` in `go2_yolo_detector/` uses a custom YOLOv8 model trained on
GO2-relevant classes. Training scripts are in `training/`.

> **Status: training run in progress. Results will be added after first complete run.**

| Model | mAP@0.5 | Precision | Recall | Inf. Latency (Jetson Orin, FP16) | Inf. Latency (CPU) |
|-------|---------|-----------|--------|----------------------------------|---------------------|
| YOLOv8n (baseline) | — | — | — | — | — |
| YOLOv8s (proposed) | — | — | — | — | — |

_Evaluation on held-out validation split. Classes: `owner_person`, `wrist_marker`, `phone_marker`, `follow_marker`._

---

## Target Predictor

`go2_yolo_detector/target_predictor.py` runs a constant-velocity Kalman filter over
the locked target's 3D position estimate. When the target is momentarily occluded
(detector state = `SEARCHING`), the predictor publishes an extrapolated pose so the
navigator does not stop dead while waiting for a fresh detection.

The predictor uses a 2-state model: position + velocity per axis. Process noise
`Q` and measurement noise `R` are tunable via ROS 2 parameters:

```
target_predictor:
  process_noise_pos: 0.1    # position uncertainty per timestep
  process_noise_vel: 0.5    # velocity uncertainty per timestep
  measurement_noise: 0.05   # depth + pixel projection uncertainty
```

> **Evaluation:** time-extended tracking under simulated occlusion (model hidden
> behind a wall for 1–5 seconds) will be added here once hardware experiments run.

---

## Nav2 Parameter Choices

Key tuning decisions in `go2_yolo_bringup/config/nav2_params.yaml`:

| Parameter | Value | Reason |
|-----------|-------|--------|
| `controller_frequency` | 10 Hz | CHAMP gait cycle |
| `FollowPath.desired_linear_vel` | 0.3 m/s | Conservative for quadruped |
| `FollowPath.max_angular_vel` | 1.0 rad/s | Prevents yaw overshoot |
| `inflation_radius` | 0.35 m | GO2 body radius + margin |
| `slam_params.yaml throttle_scans` | 3 | Reduces SLAM queue saturation |

See `go2_yolo_bringup/config/nav2_params.yaml` for the full parameter set.
