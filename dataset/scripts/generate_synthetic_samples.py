"""Generates calibrated synthetic Indian road scene images and YOLO format label files
for training, validation, and automated testing of the 11-class detector.
"""

from pathlib import Path
import random
import cv2
import numpy as np
import yaml

CLASS_NAMES = [
    "car", "bus", "truck", "bike", "auto_rickshaw",
    "cycle", "cow", "dog", "pedestrian", "push_cart", "pothole"
]

CLASS_COLORS = {
    0: (0, 200, 0),     # Car (Green)
    1: (200, 0, 0),     # Bus (Blue)
    2: (150, 0, 150),   # Truck (Purple)
    3: (255, 200, 0),   # Bike (Cyan)
    4: (0, 165, 255),   # Auto-Rickshaw (Orange)
    5: (255, 255, 0),   # Cycle (Yellow-Cyan)
    6: (0, 0, 220),     # Cow (Red-Brown)
    7: (128, 128, 0),   # Dog (Dark Cyan)
    8: (0, 255, 255),   # Pedestrian (Yellow)
    9: (180, 105, 255), # Push Cart (Pink)
    10: (30, 30, 30),   # Pothole (Dark Gray/Black)
}


def draw_synthetic_road_scene(width: int = 640, height: int = 640) -> tuple[np.ndarray, list[tuple[int, float, float, float, float]]]:
    """Renders a synthetic road image with perspective horizon, asphalt texture, and bounding boxes."""
    img = np.zeros((height, width, 3), dtype=np.uint8)

    # 1. Sky and Horizon
    horizon_y = int(height * 0.42)
    img[:horizon_y, :] = (210, 180, 140) # Dusty light sky

    # 2. Road surface with perspective trapezoid
    road_pts = np.array([
        [int(width * 0.35), horizon_y],
        [int(width * 0.65), horizon_y],
        [width, height],
        [0, height],
    ], dtype=np.int32)
    cv2.fillPoly(img, [road_pts], (75, 75, 75)) # Asphalt gray

    # 3. Earthen shoulders (unpaved kaccha edges)
    left_shoulder = np.array([[0, horizon_y], [int(width * 0.35), horizon_y], [0, height]], dtype=np.int32)
    right_shoulder = np.array([[width, horizon_y], [int(width * 0.65), horizon_y], [width, height]], dtype=np.int32)
    cv2.fillPoly(img, [left_shoulder], (50, 100, 140)) # Brownish earth
    cv2.fillPoly(img, [right_shoulder], (50, 100, 140))

    # Add road surface texture noise
    noise = np.random.normal(0, 8, img.shape).astype(np.int16)
    img = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    # 4. Insert 3 to 6 heterogeneous obstacles
    labels: list[tuple[int, float, float, float, float]] = []
    num_objects = random.randint(3, 6)

    # Sample random classes
    chosen_classes = random.sample(range(len(CLASS_NAMES)), num_objects)

    for cls_id in chosen_classes:
        # Distance scaling: closer objects appear lower in image and larger
        y_center_norm = random.uniform(0.50, 0.88)
        scale = (y_center_norm - 0.42) / 0.46 # 0.17 to 1.0

        if cls_id == 10: # Pothole: wide and flat on road surface
            w_norm = random.uniform(0.08, 0.18) * scale
            h_norm = random.uniform(0.04, 0.08) * scale
            x_center_norm = random.uniform(0.30, 0.70)
        elif cls_id in [1, 2]: # Bus, Truck: tall and wide
            w_norm = random.uniform(0.20, 0.32) * scale
            h_norm = random.uniform(0.22, 0.35) * scale
            x_center_norm = random.uniform(0.25, 0.75)
        elif cls_id in [3, 4, 5]: # Bike, Auto-Rickshaw, Cycle
            w_norm = random.uniform(0.08, 0.16) * scale
            h_norm = random.uniform(0.12, 0.22) * scale
            x_center_norm = random.uniform(0.20, 0.80)
        elif cls_id in [6, 7]: # Cow, Dog
            w_norm = random.uniform(0.12, 0.20) * scale
            h_norm = random.uniform(0.10, 0.16) * scale
            x_center_norm = random.uniform(0.25, 0.75)
        else: # Pedestrian, Push Cart, Car
            w_norm = random.uniform(0.06, 0.18) * scale
            h_norm = random.uniform(0.12, 0.25) * scale
            x_center_norm = random.uniform(0.15, 0.85)

        # Draw simulated bounding box on image
        x1 = int((x_center_norm - w_norm / 2.0) * width)
        y1 = int((y_center_norm - h_norm / 2.0) * height)
        x2 = int((x_center_norm + w_norm / 2.0) * width)
        y2 = int((y_center_norm + h_norm / 2.0) * height)

        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(width - 1, x2), min(height - 1, y2)

        color = CLASS_COLORS[cls_id]
        if cls_id == 10:
            # Elliptical depression for pothole
            center_pt = ((x1 + x2) // 2, (y1 + y2) // 2)
            axes = ((x2 - x1) // 2, (y2 - y1) // 2)
            cv2.ellipse(img, center_pt, axes, 0, 0, 360, (20, 20, 20), -1)
            cv2.ellipse(img, center_pt, axes, 0, 0, 360, (50, 50, 50), 2)
        else:
            # Rectangular solid box with border
            cv2.rectangle(img, (x1, y1), (x2, y2), color, -1)
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 255, 255), 2)

        labels.append((cls_id, x_center_norm, y_center_norm, w_norm, h_norm))

    return img, labels


def generate_dataset_split(output_dir: Path, split_name: str, num_samples: int = 15):
    """Generates image and label pairs for a dataset split (train, val, test)."""
    img_dir = output_dir / "images" / split_name
    lbl_dir = output_dir / "labels" / split_name
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)

    for i in range(num_samples):
        base_name = f"indian_road_{split_name}_{i:04d}"
        img, labels = draw_synthetic_road_scene(640, 640)

        img_path = img_dir / f"{base_name}.jpg"
        lbl_path = lbl_dir / f"{base_name}.txt"

        cv2.imwrite(str(img_path), img)

        with open(lbl_path, "w", encoding="utf-8") as f:
            for cls_id, xc, yc, w, h in labels:
                f.write(f"{cls_id} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}\n")


if __name__ == "__main__":
    dataset_root = Path(__file__).resolve().parent.parent
    random.seed(42)
    np.random.seed(42)

    print(f"Generating synthetic Indian road dataset at {dataset_root}...")
    generate_dataset_split(dataset_root, "train", num_samples=20)
    generate_dataset_split(dataset_root, "val", num_samples=10)
    generate_dataset_split(dataset_root, "test", num_samples=5)
    print("Dataset generation completed successfully.")
