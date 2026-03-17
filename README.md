# GO2 Nav2 + YOLOv8 — Unitree Gazebo Demo 🐕

A Gazebo simulation of the **Unitree GO2 quadruped** using **YOLOv8** to detect objects
and **Nav2** to autonomously navigate toward them — all running on the real GO2 platform
and its legged locomotion stack.

![ROS2](https://img.shields.io/badge/ROS_2-Humble-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Classic_11-orange)
![Python](https://img.shields.io/badge/Python-3.10-blue)
![License](https://img.shields.io/badge/License-MIT-green)

> **Related:** This demo isolates the Nav2 + YOLOv8 pipeline from my
> [GO2 Seeing-Eye Dog](https://github.com/yusufdxb/GO2-seeing-eye-dog) thesis project.

---

## Demo

---

## What It Does

```
Camera feed ──► YOLOv8 detection ──► 3D position estimate
                                            │
                                            ▼
                                     Nav2 goal pose
                                            │
                                            ▼
                               CHAMP quadruped controller
                                            │
                                            ▼
                                GO2 walks to the object
```

1. GO2 spawns in a Gazebo world with objects (person proxy, chair, bottle)
2. Onboard camera stream feeds into YOLOv8 — detects objects in real time
3. Depth image fuses with detections to estimate 3D object positions
4. User selects a target class — navigator sends a Nav2 `NavigateToPose` goal
5. Nav2 plans a path and sends velocity commands to the CHAMP controller
6. CHAMP translates `/cmd_vel` into quadruped gait — the GO2 walks there

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     Gazebo Classic                       │
│                                                          │
│   GO2 URDF (go2_description)                            │
│   + 2D LiDAR (for Nav2 costmaps)                        │
│   + RGB-D Camera (for YOLOv8 + depth)                   │
│   + IMU                                                  │
└─────────────────────────────────────────────────────────┘
         │ /joint_states  │ /scan  │ /camera/*  │ /imu
         ▼                ▼
┌──────────────────┐   ┌──────────────────────────────────┐
│  CHAMP Controller│   │        Your ROS 2 Nodes           │
│                  │   │                                   │
│  - gait planner  │◄──│  detector_node  (YOLOv8 + depth) │
│  - state estim.  │   │  navigator_node (Nav2 goals)      │
│  - odometry/EKF  │   │                                   │
│  - /cmd_vel in   │◄──│        Nav2 Stack                 │
└──────────────────┘   │  - AMCL localization              │
         │              │  - NavFn planner                  │
         ▼              │  - DWB controller                 │
   /joint_trajectory    │  - Costmaps (lidar + inflation)   │
                        └──────────────────────────────────┘
```

---

## Dependencies

### External packages to clone alongside this repo

```bash
# CHAMP quadruped controller (handles GO2 legged locomotion)
git clone https://github.com/chvmp/champ.git
git clone https://github.com/chvmp/champ_teleop.git

# GO2 description (URDF, meshes, ros2_control config)
# Already included as go2_description/ in this repo
# Source: https://github.com/arjun-sadananda/go2_nav2_ros2
```

### APT packages

```bash
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs
```

### Python packages

```bash
pip install ultralytics opencv-python numpy
```

---

## Setup

```bash
# 1. Create workspace
mkdir -p ~/go2_yolo_ws/src && cd ~/go2_yolo_ws/src

# 2. Clone this repo
git clone https://github.com/yusufdxb/ros2-go2-nav2-yolo.git

# 3. Clone CHAMP controller
git clone https://github.com/chvmp/champ.git
git clone https://github.com/chvmp/champ_teleop.git

# 4. Install all ROS dependencies
cd ~/go2_yolo_ws
rosdep install --from-paths src --ignore-src -r -y

# 5. Build
colcon build --symlink-install
source install/setup.bash
```

---

## Run

```bash
# Terminal 1 — Launch Gazebo + GO2 + CHAMP controller
ros2 launch go2_yolo_bringup gazebo_launch.py

# Terminal 2 — Launch Nav2 + SLAM
ros2 launch go2_yolo_bringup navigation_launch.py

# Terminal 3 — Launch YOLO detector + navigator
ros2 launch go2_yolo_bringup yolo_nav_launch.py target_class:=person

# Optional: teleop to explore and build a map first
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Changing the target object

```bash
# Navigate to a person
ros2 launch go2_yolo_bringup yolo_nav_launch.py target_class:=person

# Navigate to a chair
ros2 launch go2_yolo_bringup yolo_nav_launch.py target_class:=chair

# Or publish a target at runtime
ros2 topic pub /navigator/target_class std_msgs/msg/String "data: 'bottle'" --once
```

---

## Package Structure

```
ros2-go2-nav2-yolo/
├── go2_description/              # GO2 URDF, meshes, ros2_control config
│   ├── xacro/
│   │   ├── go2_robot.xacro       # Main robot description
│   │   ├── camera.xacro          # RGB-D camera plugin
│   │   └── laser.xacro           # 2D LiDAR plugin
│   ├── meshes/                   # GO2 visual meshes
│   └── config/
│       └── ros2_control.yaml
├── go2_yolo_bringup/             # Launch files
│   ├── launch/
│   │   ├── gazebo_launch.py      # Gazebo + GO2 + CHAMP
│   │   ├── navigation_launch.py  # Nav2 + SLAM
│   │   └── yolo_nav_launch.py    # Detector + navigator
│   ├── worlds/
│   │   └── demo_world.world      # Objects scattered in room
│   ├── config/
│   │   ├── nav2_params.yaml      # Nav2 tuned for GO2
│   │   └── gait.yaml             # CHAMP gait params (tuned for GO2)
│   └── rviz/
│       └── go2_nav2_yolo.rviz
├── go2_yolo_detector/            # YOLOv8 + depth fusion node
│   └── go2_yolo_detector/
│       └── detector_node.py
├── go2_yolo_navigator/           # Nav2 goal publisher node
│   └── go2_yolo_navigator/
│       └── navigator_node.py
└── go2_yolo_msgs/                # Custom messages
    └── msg/
        ├── DetectedObject.msg
        └── DetectedObjectArray.msg
```

---

## Key Differences from TurtleBot3 Version

| | TurtleBot3 (wheeled) | GO2 (this repo) |
|---|---|---|
| Locomotion | Differential drive | CHAMP quadruped controller |
| Motion interface | `/cmd_vel` direct | `/cmd_vel` → CHAMP → joint trajectories |
| Odometry | Wheel encoders | CHAMP state estimation + EKF |
| Visual | Boring | The actual robot from your thesis |
| Nav2 frame | `base_footprint` | `base_footprint` (CHAMP publishes this) |

---

## Known Issues & Notes

- **Odometry drift:** CHAMP's state estimation uses robot dimensions from a generic quadruped config. The odometry may drift over long distances — this is a known issue in the upstream CHAMP/GO2 integration. For demo purposes it's fine over short runs.
- **Gazebo physics:** The GO2 in Gazebo Classic uses simplified contact physics. The gait looks somewhat stiff compared to the real robot — this is normal.
- **Camera topic remapping:** The YOLO detector expects `/camera/image_raw` and `/camera/depth/image_raw`. If your URDF uses different topic names, remap them in `yolo_nav_launch.py`.

---

## Credits

This repo builds on:
- [arjun-sadananda/go2_nav2_ros2](https://github.com/arjun-sadananda/go2_nav2_ros2) — GO2 + Nav2 Gazebo integration
- [chvmp/champ](https://github.com/chvmp/champ) — CHAMP quadruped controller
- [unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) — GO2 URDF/meshes

---

## Author

**Yusuf Guenena** | M.S. Robotics Engineering, Wayne State University
[LinkedIn](https://www.linkedin.com/in/yusuf-guenena) · [GitHub](https://github.com/yusufdxb)
