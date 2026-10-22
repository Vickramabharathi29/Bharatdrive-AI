"""Visualization module for Indian Road Object & Pothole Detection.

Renders color-coded 2D bounding boxes, semantic badges, confidence percentages,
and pinhole camera distance estimates.
"""

from __future__ import annotations
import sys
from pathlib import Path
from typing import List, Dict, Any

# Ensure project root is on sys.path
_ROOT = Path(__file__).resolve().parent.parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import cv2
import numpy as np
from dataset.scripts.generate_synthetic_samples import CLASS_COLORS


def draw_detection_overlay(
    image: np.ndarray,
    detections: List[Dict[str, Any]],
    highlight_collision_corridor: bool = True,
) -> np.ndarray:
    """Draws rich bounding box annotations and distance indicators onto image."""
    annotated = image.copy()
    h, w = annotated.shape[:2]

    # Draw forward safety corridor lines (trapezoid ahead of ego vehicle)
    if highlight_collision_corridor:
        corridor_overlay = annotated.copy()
        corridor_pts = np.array([
            [int(w * 0.42), int(h * 0.50)],
            [int(w * 0.58), int(h * 0.50)],
            [int(w * 0.75), h],
            [int(w * 0.25), h],
        ], dtype=np.int32)
        cv2.fillPoly(corridor_overlay, [corridor_pts], (0, 255, 255))
        cv2.addWeighted(corridor_overlay, 0.12, annotated, 0.88, 0, annotated)

    for det in detections:
        box = det["box"]
        cls_id = int(det["class_id"])
        cls_name = str(det["class_name"])
        conf = float(det["confidence"])
        dist_m = float(det.get("estimated_distance_m", 0.0))

        x1, y1, x2, y2 = map(int, box)
        color = CLASS_COLORS.get(cls_id, (0, 255, 0))

        # Warning border if distance is critically close (< 12 meters)
        if dist_m > 0.0 and dist_m < 12.0:
            box_color = (0, 0, 255) # Red emergency warning
            thickness = 3
        else:
            box_color = color
            thickness = 2

        # Draw main box or ellipse for potholes
        if cls_name == "pothole":
            center = ((x1 + x2) // 2, (y1 + y2) // 2)
            axes = ((x2 - x1) // 2, (y2 - y1) // 2)
            cv2.ellipse(annotated, center, axes, 0, 0, 360, (0, 0, 255), 2)
            cv2.circle(annotated, center, 4, (0, 0, 255), -1)
        else:
            cv2.rectangle(annotated, (x1, y1), (x2, y2), box_color, thickness)

        # Draw text badge
        label = f"{cls_name.replace('_', ' ').title()}: {conf * 100:.0f}%"
        if dist_m > 0:
            label += f" | {dist_m:.1f}m"

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.45
        (text_w, text_h), baseline = cv2.getTextSize(label, font, font_scale, 1)

        badge_y1 = max(0, y1 - text_h - 6)
        badge_y2 = y1
        badge_x2 = min(w, x1 + text_w + 8)

        # Semi-transparent badge background
        badge_bg = annotated[badge_y1:badge_y2, x1:badge_x2].copy()
        if badge_bg.size > 0:
            cv2.rectangle(annotated, (x1, badge_y1), (badge_x2, badge_y2), box_color, -1)
            cv2.putText(
                annotated,
                label,
                (x1 + 4, y1 - 4),
                font,
                font_scale,
                (0, 0, 0) if (color[0] + color[1] + color[2]) > 400 else (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

    # Telemetry HUD in top left
    hud_text = f"BharatDrive-AI YOLOv11 | Targets Detected: {len(detections)}"
    cv2.putText(annotated, hud_text, (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(annotated, hud_text, (15, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 1, cv2.LINE_AA)

    return annotated


def render_and_save_sample(
    input_image_path: str | Path,
    detector: Any,
    output_path: str | Path,
) -> np.ndarray:
    """Runs detector on image and saves visualization artifact."""
    img = cv2.imread(str(input_image_path))
    if img is None:
        raise ValueError(f"Could not load image from {input_image_path}")

    detections = detector.detect(img)
    vis = draw_detection_overlay(img, detections)

    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), vis)
    return vis


if __name__ == "__main__":
    from yolo_detection.scripts.inference import YOLOIndianRoadDetector
    root = Path(__file__).resolve().parent.parent.parent
    test_img = root / "dataset" / "images" / "val" / "indian_road_val_0000.jpg"
    out_img = root / "dataset" / "processed" / "detection_demo.jpg"

    if test_img.exists():
        detector = YOLOIndianRoadDetector()
        render_and_save_sample(test_img, detector, out_img)
        print(f"Sample detection visual saved to: {out_img}")
