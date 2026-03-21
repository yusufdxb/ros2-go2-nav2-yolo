#!/usr/bin/env bash
# train.sh — YOLOv8 custom training pipeline for go2_perception
#
# Prerequisites:
#   pip install ultralytics>=8.2 albumentations rosbags supervision tqdm
#   GPU with CUDA (RTX-class — verified on RTX 3070+)
#
# Usage:
#   bash train.sh                    # full run with defaults
#   bash train.sh --epochs 200       # override epochs
#   bash train.sh --model yolov8m.pt # use medium model
#
# Outputs:
#   ~/models/go2_perception/run_XX/weights/best.pt   ← deploy this
#   ~/models/go2_perception/run_XX/weights/last.pt

set -euo pipefail

# ── Configurable defaults ────────────────────────────────────────────────────
DATASET_YAML="${DATASET_YAML:-$HOME/datasets/go2_perception/dataset.yaml}"
BASE_MODEL="${BASE_MODEL:-yolov8s.pt}"       # s = good balance for RTX GPU
EPOCHS="${EPOCHS:-150}"
IMGSZ="${IMGSZ:-640}"
BATCH="${BATCH:-16}"                          # 16 safe for 8 GB VRAM; 32 for 12+ GB
DEVICE="${DEVICE:-0}"                         # GPU index (0 = first GPU)
PROJECT="${PROJECT:-$HOME/models/go2_perception}"
RUN_NAME="${RUN_NAME:-run_$(date +%Y%m%d_%H%M)}"
PATIENCE="${PATIENCE:-30}"                    # early stopping patience
WORKERS="${WORKERS:-8}"

# ── Parse CLI overrides ───────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case $1 in
        --epochs)   EPOCHS="$2";    shift 2 ;;
        --model)    BASE_MODEL="$2"; shift 2 ;;
        --batch)    BATCH="$2";     shift 2 ;;
        --device)   DEVICE="$2";    shift 2 ;;
        --name)     RUN_NAME="$2";  shift 2 ;;
        *)          echo "Unknown arg: $1"; exit 1 ;;
    esac
done

echo "================================================"
echo " GO2 Perception — YOLOv8 Training"
echo "  dataset : $DATASET_YAML"
echo "  model   : $BASE_MODEL"
echo "  epochs  : $EPOCHS  batch: $BATCH  device: $DEVICE"
echo "  output  : $PROJECT/$RUN_NAME"
echo "================================================"

# ── Step 1: Validate dataset ──────────────────────────────────────────────────
echo "[1/4] Validating dataset YAML..."
python3 - <<'PYEOF'
import yaml, sys
from pathlib import Path

cfg_path = "$DATASET_YAML"
cfg = yaml.safe_load(Path(cfg_path).read_text())
root = Path(cfg["path"])
for split in ("train", "val"):
    img_dir = root / cfg[split]
    if not img_dir.exists():
        print(f"  MISSING: {img_dir}")
        sys.exit(1)
    n = len(list(img_dir.glob("*.jpg")) + list(img_dir.glob("*.png")))
    print(f"  {split}: {n} images in {img_dir}")
print(f"  classes: {cfg['names']}")
PYEOF

# ── Step 2: Optional offline augmentation ────────────────────────────────────
echo "[2/4] Running offline augmentation (wrist_marker, phone_marker, follow_marker)..."
python3 "$(dirname "$0")/augment_dataset.py" \
    --dataset "$(dirname "$DATASET_YAML")" \
    --target-count 800 \
    --classes 1 2 3

# ── Step 3: Train ─────────────────────────────────────────────────────────────
echo "[3/4] Starting training..."
yolo detect train \
    data="$DATASET_YAML" \
    model="$BASE_MODEL" \
    epochs="$EPOCHS" \
    imgsz="$IMGSZ" \
    batch="$BATCH" \
    patience="$PATIENCE" \
    optimizer=AdamW \
    lr0=0.001 \
    lrf=0.01 \
    weight_decay=0.0005 \
    warmup_epochs=5 \
    warmup_momentum=0.8 \
    momentum=0.937 \
    hsv_h=0.015 \
    hsv_s=0.7 \
    hsv_v=0.4 \
    degrees=15.0 \
    translate=0.1 \
    scale=0.5 \
    fliplr=0.5 \
    mosaic=1.0 \
    mixup=0.1 \
    copy_paste=0.15 \
    project="$PROJECT" \
    name="$RUN_NAME" \
    device="$DEVICE" \
    workers="$WORKERS" \
    save_period=10 \
    plots=True \
    val=True \
    verbose=True

# ── Step 4: Export ────────────────────────────────────────────────────────────
WEIGHTS="$PROJECT/$RUN_NAME/weights/best.pt"
echo "[4/4] Exporting: $WEIGHTS"
bash "$(dirname "$0")/export_model.sh" "$WEIGHTS"

echo ""
echo "Training complete."
echo "  Best weights : $WEIGHTS"
echo "  Deploy path  : $HOME/models/go2_perception/best.pt (symlink below)"
ln -sf "$WEIGHTS" "$HOME/models/go2_perception/best.pt" 2>/dev/null || true
echo "  Symlink      : $HOME/models/go2_perception/best.pt -> $WEIGHTS"
