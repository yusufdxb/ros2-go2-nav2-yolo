#!/usr/bin/env python3
"""
auto_label.py — Generate YOLO labels from Gazebo model states and camera projections.

This script processes extracted frames and a Gazebo world state (or ROS2 bag with model states)
to automatically create ground-truth bounding boxes for custom training.
"""

import os
import argparse
import numpy as np
import cv2
from pathlib import Path

class GazeboAutoLabeler:
    def __init__(self, camera_matrix, dist_coeffs=None):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs if dist_coeffs is not None else np.zeros(5)

    def project_3d_to_2d(self, point_3d, rvec, tvec):
        """Project a 3D point in world coordinates to 2D image coordinates."""
        points_2d, _ = cv2.projectPoints(
            np.array([point_3d], dtype=np.float32), 
            rvec, tvec, 
            self.camera_matrix, 
            self.dist_coeffs
        )
        return points_2d[0][0]

    def get_yolo_bbox(self, points_2d, img_width, img_height):
        """Calculate YOLO format bounding box from a set of 2D points."""
        x_min, y_min = np.min(points_2d, axis=0)
        x_max, y_max = np.max(points_2d, axis=0)

        # Clamp to image boundaries
        x_min = max(0, x_min)
        y_min = max(0, y_min)
        x_max = min(img_width - 1, x_max)
        y_max = min(img_height - 1, y_max)

        # YOLO format: class x_center y_center width height (normalized 0-1)
        w = (x_max - x_min) / img_width
        h = (y_max - y_min) / img_height
        x_center = (x_min + x_max) / 2 / img_width
        y_center = (y_min + y_max) / 2 / img_height

        return x_center, y_center, w, h

def main():
    parser = argparse.ArgumentParser(description="Auto-labeler for Gazebo models")
    parser.add_argument("--frames", required=True, help="Directory containing extracted JPEG frames")
    parser.add_argument("--output", required=True, help="Output directory for YOLO labels")
    # In a real implementation, we would also need the camera calibration and model states
    # This is a skeleton that we will flesh out with ROS2 integration
    args = parser.parse_args()

    print("Auto-labeler initialized. This script requires integration with Gazebo model states.")
    print(f"Targeting frames in: {args.frames}")

if __name__ == "__main__":
    main()
