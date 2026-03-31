# YOLOv8 Custom Training Plan — Gazebo Simulation

This file serves as a quick-reference guide for the custom training pipeline to enable the Unitree Go2 to detect simulation-specific models.

## 1. The Strategy: Auto-Labeling
Since Gazebo's `person_standing` model is synthetic and often fails with pretrained YOLOv8n, we will:
1.  **Record data:** Use `ros2 bag record` on `/go2/camera/image_raw`.
2.  **Auto-Label:** Use a script to project Gazebo's 3D model positions into 2D camera coordinates to generate ground-truth YOLO labels (`.txt`) automatically.
3.  **Train:** Use the existing `training/train.sh` to fine-tune `yolov8n.pt`.

## 2. Preparation Checklist
- [ ] **Define Items:** List models besides `person_standing` (e.g., `construction_cone`, `coke_can`).
- [ ] **World Setup:** Ensure these models are in your Gazebo `.world` file.
- [ ] **Record Bag:** Move the robot around the models and record a 2–3 minute bag.

## 3. Training Workflow
```bash
# 1. Extract frames from bag
python3 training/collect_frames.py extract --bag <path_to_bag> --out <raw_frames_dir>

# 2. Run Auto-Labeler (to be developed)
python3 training/auto_label.py --frames <raw_frames_dir> --world <world_file>

# 3. Split dataset
python3 training/collect_frames.py split --frames <labeled_dir> --dataset <dataset_root>

# 4. Train
bash training/train.sh --data <dataset_root>/data.yaml --epochs 50
```

## 4. Integration
Once trained, update `detector_node.py` to point to your new `best.pt` weights:
- **Location:** `go2_yolo_detector/go2_yolo_detector/detector_node.py`
- **Class Map:** Update the `CLASS_NAMES` dictionary if you added new items.

---
*Created on 2026-03-28. Full history available in Obsidian Vault: Projects/ROS-YOLO-Nav-Go2/*
