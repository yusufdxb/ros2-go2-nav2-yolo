#!/usr/bin/env python3
"""
augment_dataset.py — Offline augmentation for the go2_perception dataset.

Run this BEFORE training to expand small classes (wrist_marker, phone_marker)
to recommended counts. It reads from the train split and writes new augmented
images + labels back into train/images and train/labels.

Usage:
  pip install albumentations opencv-python tqdm
  python3 augment_dataset.py \
      --dataset ~/datasets/go2_perception \
      --target-count 800 \
      --classes 1 2 3   # augment only these class IDs (default: all)

Strategy:
  - For each image that contains a target class, apply N random augmentations
    until the class reaches target_count images containing it.
  - Label files are transformed geometrically alongside images (albumentations
    BboxParams with YOLO format handle this automatically).
  - Heavy augmentation for small/rare markers; lighter for owner_person.
"""

import argparse
import random
import cv2
import numpy as np
from pathlib import Path
from tqdm import tqdm

try:
    import albumentations as A
except ImportError:
    raise SystemExit("Install albumentations: pip install albumentations")


# ── Augmentation pipelines ────────────────────────────────────────────────────

def _bbox_params():
    return A.BboxParams(
        format="yolo",
        label_fields=["class_labels"],
        min_visibility=0.3,    # drop boxes that become <30% visible after crop
        clip=True,
    )


# Light pipeline — used for owner_person (large bbox, common class)
LIGHT_AUG = A.Compose([
    A.HorizontalFlip(p=0.5),
    A.RandomBrightnessContrast(brightness_limit=0.3, contrast_limit=0.3, p=0.7),
    A.HueSaturationValue(hue_shift_limit=10, sat_shift_limit=40, val_shift_limit=30, p=0.6),
    A.GaussianBlur(blur_limit=(3, 5), p=0.3),
    A.RandomGamma(gamma_limit=(80, 120), p=0.4),
    A.ShiftScaleRotate(shift_limit=0.05, scale_limit=0.15, rotate_limit=10,
                       border_mode=cv2.BORDER_REFLECT_101, p=0.7),
], bbox_params=_bbox_params())


# Heavy pipeline — used for small markers (wrist_marker, phone_marker, follow_marker)
HEAVY_AUG = A.Compose([
    A.HorizontalFlip(p=0.5),
    A.VerticalFlip(p=0.1),
    A.RandomBrightnessContrast(brightness_limit=0.5, contrast_limit=0.4, p=0.8),
    A.HueSaturationValue(hue_shift_limit=20, sat_shift_limit=60, val_shift_limit=50, p=0.7),
    A.GaussianBlur(blur_limit=(3, 7), p=0.4),       # simulate motion blur
    A.MotionBlur(blur_limit=9, p=0.3),
    A.RandomGamma(gamma_limit=(60, 140), p=0.5),
    A.CLAHE(clip_limit=4.0, p=0.3),
    A.ShiftScaleRotate(shift_limit=0.1, scale_limit=0.3, rotate_limit=20,
                       border_mode=cv2.BORDER_REFLECT_101, p=0.8),
    A.RandomShadow(num_shadows_lower=1, num_shadows_upper=3,
                   shadow_dimension=5, p=0.3),
    A.ISONoise(color_shift=(0.01, 0.05), intensity=(0.1, 0.5), p=0.3),
    A.CoarseDropout(max_holes=4, max_height=32, max_width=32, p=0.2),  # partial occlusion
], bbox_params=_bbox_params())


# ── I/O helpers ───────────────────────────────────────────────────────────────

def load_labels(label_path: Path):
    """Return list of (class_id, cx, cy, w, h) from a YOLO .txt file."""
    boxes, classes = [], []
    if not label_path.exists():
        return boxes, classes
    for line in label_path.read_text().strip().splitlines():
        parts = line.split()
        if len(parts) != 5:
            continue
        cid, cx, cy, w, h = int(parts[0]), *map(float, parts[1:])
        boxes.append((cx, cy, w, h))
        classes.append(cid)
    return boxes, classes


def save_labels(label_path: Path, bboxes, class_labels):
    lines = [
        f"{c} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}"
        for (cx, cy, w, h), c in zip(bboxes, class_labels)
    ]
    label_path.write_text("\n".join(lines))


def count_class_images(images_dir: Path, labels_dir: Path, class_id: int) -> list[Path]:
    """Return list of image paths that contain at least one instance of class_id."""
    result = []
    for img_path in sorted(images_dir.glob("*.jpg")):
        lbl_path = labels_dir / img_path.with_suffix(".txt").name
        _, classes = load_labels(lbl_path)
        if class_id in classes:
            result.append(img_path)
    return result


# ── Main augmentation logic ───────────────────────────────────────────────────

def augment_class(images_dir: Path, labels_dir: Path,
                  class_id: int, target_count: int, pipeline):
    source_images = count_class_images(images_dir, labels_dir, class_id)
    current_count = len(source_images)

    if current_count >= target_count:
        print(f"  class {class_id}: {current_count} images >= target {target_count} — skipping")
        return

    needed = target_count - current_count
    print(f"  class {class_id}: {current_count} → {target_count} (generating {needed} augmented)")

    aug_idx = 0
    with tqdm(total=needed, desc=f"  class_{class_id}") as pbar:
        while aug_idx < needed:
            src_img_path = random.choice(source_images)
            src_lbl_path = labels_dir / src_img_path.with_suffix(".txt").name

            img = cv2.imread(str(src_img_path))
            if img is None:
                continue
            bboxes, class_labels = load_labels(src_lbl_path)
            if not bboxes:
                continue

            try:
                result = pipeline(image=img, bboxes=bboxes, class_labels=class_labels)
            except Exception:
                continue

            if not result["bboxes"]:
                continue  # all boxes were cropped out

            stem = f"aug_{class_id}_{aug_idx:06d}"
            out_img  = images_dir / f"{stem}.jpg"
            out_lbl  = labels_dir / f"{stem}.txt"

            cv2.imwrite(str(out_img), result["image"], [cv2.IMWRITE_JPEG_QUALITY, 92])
            save_labels(out_lbl, result["bboxes"], result["class_labels"])

            aug_idx += 1
            pbar.update(1)


def main():
    parser = argparse.ArgumentParser(description="Offline augmentation for go2_perception dataset")
    parser.add_argument("--dataset", default="/home/carebear/datasets/go2_perception",
                        help="Dataset root directory")
    parser.add_argument("--target-count", type=int, default=800,
                        help="Target number of images per class")
    parser.add_argument("--classes", type=int, nargs="+", default=[0, 1, 2, 3],
                        help="Class IDs to augment (default: all 4)")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)

    root = Path(args.dataset)
    images_dir = root / "train" / "images"
    labels_dir = root / "train" / "labels"

    if not images_dir.exists():
        raise SystemExit(f"Train images directory not found: {images_dir}")

    # owner_person (class 0) gets light aug; small markers get heavy aug
    pipelines = {
        0: LIGHT_AUG,
        1: HEAVY_AUG,
        2: HEAVY_AUG,
        3: HEAVY_AUG,
    }

    print(f"Augmenting dataset at: {root}")
    print(f"Target count per class: {args.target_count}")
    for class_id in args.classes:
        pipeline = pipelines.get(class_id, HEAVY_AUG)
        augment_class(images_dir, labels_dir, class_id, args.target_count, pipeline)

    print("Augmentation complete.")


if __name__ == "__main__":
    main()
