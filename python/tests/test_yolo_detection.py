"""Unit tests for YOLO Indian Road Object & Pothole Detection module."""

from pathlib import Path
import numpy as np
import pytest
import yaml

from yolo_detection.scripts.inference import (
    YOLOIndianRoadDetector,
    compute_iou,
    non_maximum_suppression,
    CLASS_NAMES,
)
from yolo_detection.scripts.evaluate import (
    match_predictions_to_ground_truth,
    compute_metrics_for_class,
    evaluate_dataset,
)
from yolo_detection.scripts.visualizer import draw_detection_overlay


def test_yolo_dataset_yaml_classes():
    root = Path(__file__).resolve().parent.parent.parent
    data_yaml = root / "dataset" / "data.yaml"
    assert data_yaml.exists(), "dataset/data.yaml not found"

    with open(data_yaml, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    assert data["nc"] == 11
    expected_classes = [
        "car", "bus", "truck", "bike", "auto_rickshaw",
        "cycle", "cow", "dog", "pedestrian", "push_cart", "pothole"
    ]
    for cid, name in enumerate(expected_classes):
        assert data["names"][cid] == name


def test_iou_calculation():
    # Exactly identical boxes: IoU should be 1.0
    box_a = np.array([10.0, 10.0, 50.0, 50.0])
    assert compute_iou(box_a, box_a) == pytest.approx(1.0)

    # Disjoint boxes: IoU should be 0.0
    box_b = np.array([60.0, 60.0, 100.0, 100.0])
    assert compute_iou(box_a, box_b) == pytest.approx(0.0)

    # Partial 50% area overlap
    box_c = np.array([30.0, 10.0, 50.0, 50.0]) # 20x40 = 800 overlap out of 40x40 + 20x40 - 800 = 1600
    assert compute_iou(box_a, box_c) == pytest.approx(0.50)


def test_non_maximum_suppression():
    boxes = np.array([
        [10.0, 10.0, 50.0, 50.0],
        [12.0, 11.0, 49.0, 51.0], # High overlap duplicate of box 0
        [100.0, 100.0, 140.0, 140.0], # Disjoint box
    ])
    scores = np.array([0.92, 0.75, 0.88])
    class_ids = np.array([4, 4, 0]) # Two Auto-Rickshaws, One Car

    keep = non_maximum_suppression(boxes, scores, class_ids, iou_threshold=0.45)
    # Box 1 should be suppressed because Box 0 has higher score for class 4
    assert 0 in keep
    assert 2 in keep
    assert 1 not in keep


def test_detector_inference_on_synthetic_image():
    detector = YOLOIndianRoadDetector(conf_threshold=0.30)
    test_img = np.zeros((640, 640, 3), dtype=np.uint8)
    test_img[300:500, 200:400] = (0, 165, 255) # Orange rectangular block (simulating Auto-Rickshaw)

    detections = detector.detect(test_img)
    assert isinstance(detections, list)
    if detections:
        det = detections[0]
        assert "box" in det
        assert "class_id" in det
        assert "class_name" in det
        assert "confidence" in det
        assert "estimated_distance_m" in det
        assert det["estimated_distance_m"] > 0.0


def test_evaluation_metrics_computation():
    scores = [0.95, 0.88, 0.72, 0.65]
    tp = [True, True, False, True]
    fp = [False, False, True, False]
    total_gt = 4

    p, r, f1, ap = compute_metrics_for_class(scores, tp, fp, total_gt)
    assert 0.0 <= p <= 1.0
    assert 0.0 <= r <= 1.0
    assert 0.0 <= f1 <= 1.0
    assert 0.0 <= ap <= 1.0
    assert r == pytest.approx(0.75) # 3 TP out of 4 GT


def test_visualizer_rendering():
    img = np.ones((640, 640, 3), dtype=np.uint8) * 128
    dummy_detections = [
        {
            "box": [150.0, 200.0, 260.0, 340.0],
            "class_id": 4,
            "class_name": "auto_rickshaw",
            "confidence": 0.89,
            "estimated_distance_m": 14.5,
        },
        {
            "box": [300.0, 450.0, 420.0, 520.0],
            "class_id": 10,
            "class_name": "pothole",
            "confidence": 0.94,
            "estimated_distance_m": 8.2,
        },
    ]

    annotated = draw_detection_overlay(img, dummy_detections)
    assert annotated.shape == img.shape
    assert annotated.dtype == np.uint8
    # Image should have been modified
    assert not np.array_equal(annotated, img)
