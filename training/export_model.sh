#!/usr/bin/env bash
# export_model.sh — Export a trained YOLOv8 .pt model to deployment formats.
#
# Usage:
#   bash export_model.sh /path/to/best.pt
#   bash export_model.sh  (defaults to ~/models/go2_perception/best.pt)
#
# Outputs (in same directory as best.pt):
#   best.torchscript  — fastest for pure PyTorch on RTX; used by detector_node.py
#   best.onnx         — for TensorRT conversion or cross-platform inference
#
# TensorRT conversion (optional, RTX only, much faster inference):
#   trtexec --onnx=best.onnx --saveEngine=best.engine --fp16 --workspace=4096

set -euo pipefail

WEIGHTS="${1:-$HOME/models/go2_perception/best.pt}"

if [[ ! -f "$WEIGHTS" ]]; then
    echo "ERROR: weights not found: $WEIGHTS"
    exit 1
fi

WEIGHTS_DIR="$(dirname "$WEIGHTS")"
echo "Exporting: $WEIGHTS"

# ── TorchScript (primary, used by detector_node) ──────────────────────────────
echo "  → TorchScript..."
yolo export model="$WEIGHTS" format=torchscript imgsz=640 optimize=False
echo "    saved: $WEIGHTS_DIR/best.torchscript"

# ── ONNX (for TensorRT path or cross-platform) ────────────────────────────────
echo "  → ONNX (opset 17)..."
yolo export model="$WEIGHTS" format=onnx imgsz=640 opset=17 simplify=True
echo "    saved: $WEIGHTS_DIR/best.onnx"

echo ""
echo "Export complete."
echo "  Detector node model_path parameter options:"
echo "    .pt           : $WEIGHTS"
echo "    .torchscript  : $WEIGHTS_DIR/best.torchscript"
echo "    .onnx         : $WEIGHTS_DIR/best.onnx"
echo ""
echo "Optional TensorRT conversion (RTX, FP16 — fastest inference):"
echo "  trtexec --onnx=$WEIGHTS_DIR/best.onnx \\"
echo "          --saveEngine=$WEIGHTS_DIR/best.engine \\"
echo "          --fp16 --workspace=4096"
