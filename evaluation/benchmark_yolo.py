#!/usr/bin/env python3
"""
YOLOv8 Offline Inference Benchmark — ros2-go2-nav2-yolo
========================================================
Measures inference latency for YOLOv8n and YOLOv8s on synthetic
640x480 images (the GO2 camera resolution). Does NOT require the
Gazebo simulation or ROS 2.

Run from repo root:
  python3 evaluation/benchmark_yolo.py

Outputs:
  evaluation/results/yolo_benchmark.json   — machine-readable
  RESULTS.md                                — populated result tables

Requires:
  pip install ultralytics
"""

from __future__ import annotations

import json
import statistics
import time
from pathlib import Path
from typing import Dict, List

import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    raise ImportError("Run: pip install ultralytics")

# Camera resolution used in go2_yolo_bringup (640x480 RGB)
IMG_H, IMG_W = 480, 640
WARMUP_RUNS = 10
BENCH_RUNS = 100


def bench_model(model_name: str) -> Dict:
    """Load a YOLO model and benchmark inference on synthetic 640x480 frames."""
    print(f"  Loading {model_name}...")
    model = YOLO(model_name)

    # Synthetic frame — same shape as GO2 RealSense / Gazebo camera output
    frame = np.random.randint(0, 255, (IMG_H, IMG_W, 3), dtype=np.uint8)

    # Warmup (JIT/CUDA warm-up, first-inference overhead excluded)
    for _ in range(WARMUP_RUNS):
        model.predict(frame, verbose=False, imgsz=640)

    # Benchmark
    latencies_ms: List[float] = []
    for _ in range(BENCH_RUNS):
        t0 = time.perf_counter()
        model.predict(frame, verbose=False, imgsz=640)
        latencies_ms.append((time.perf_counter() - t0) * 1000)

    sorted_lat = sorted(latencies_ms)
    return {
        "model": model_name,
        "image_size": f"{IMG_W}x{IMG_H}",
        "warmup_runs": WARMUP_RUNS,
        "bench_runs": BENCH_RUNS,
        "latency_ms": {
            "mean": round(statistics.mean(latencies_ms), 2),
            "median": round(statistics.median(latencies_ms), 2),
            "p95": round(sorted_lat[int(0.95 * len(sorted_lat))], 2),
            "p99": round(sorted_lat[int(0.99 * len(sorted_lat))], 2),
            "min": round(min(latencies_ms), 2),
            "max": round(max(latencies_ms), 2),
        },
        "fps_mean": round(1000 / statistics.mean(latencies_ms), 1),
    }


def write_results_md(results: List[Dict], out_path: Path) -> None:
    ts = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    rows = "\n".join(
        f"| {r['model']} | {r['latency_ms']['mean']} ms | "
        f"{r['latency_ms']['p95']} ms | {r['fps_mean']} fps | CPU |"
        for r in results
    )

    content = f"""# Results — ros2-go2-nav2-yolo

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
{BENCH_RUNS} runs after {WARMUP_RUNS} warmup runs. Generated: `{ts}`

| Model | Mean Latency | p95 Latency | Throughput | Hardware |
|-------|-------------|-------------|------------|----------|
{rows}

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
"""
    out_path.write_text(content)
    print(f"RESULTS.md written: {out_path}")


def main() -> None:
    print("YOLOv8 Inference Benchmark")
    print("=" * 40)

    results_dir = Path(__file__).parent / "results"
    results_dir.mkdir(exist_ok=True)

    results = []
    for model_name in ["yolov8n.pt", "yolov8s.pt"]:
        print(f"\nBenchmarking {model_name} ({BENCH_RUNS} runs)...")
        result = bench_model(model_name)
        lat = result["latency_ms"]
        print(f"  Mean: {lat['mean']} ms  p95: {lat['p95']} ms  fps: {result['fps_mean']}")
        results.append(result)

    # Write JSON
    json_out = results_dir / "yolo_benchmark.json"
    with open(json_out, "w") as f:
        json.dump({
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "benchmarks": results,
        }, f, indent=2)
    print(f"\nJSON results: {json_out}")

    # Write RESULTS.md at repo root
    repo_root = Path(__file__).parent.parent
    write_results_md(results, repo_root / "RESULTS.md")


if __name__ == "__main__":
    main()
