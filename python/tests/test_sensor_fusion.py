"""Unit tests for sensor fusion and perception engine."""

import math
import numpy as np
import pytest
from bharatdrive.perception.tracked_object import TrackedObject
from bharatdrive.perception.sensor_suite import SensorSuite, SensorDetection
from bharatdrive.perception.fusion_engine import SensorFusionEngine


def test_tracked_object_metrics():
    obj = TrackedObject(
        object_id=1,
        object_class="Cattle",
        position=np.array([30.0, 40.0, 0.0]),
        velocity=np.array([-5.0, 0.0, 0.0]),
        dimensions=np.array([2.2, 0.9, 1.5]),
        confidence_score=0.85,
        sensor_contributors=["Camera", "Radar"],
    )

    # Euclidean distance hypot(30, 40) = 50.0 m
    assert obj.distance == pytest.approx(50.0, abs=1e-3)
    # Closing speed -(-5.0) = +5.0 m/s
    assert obj.relative_speed == pytest.approx(5.0, abs=1e-3)

    d = obj.to_dict()
    assert d["object_id"] == 1
    assert d["object_class"] == "Cattle"
    assert d["distance"] == 50.0
    assert "Camera" in d["sensor_contributors"]


def test_sensor_suite_detections_and_fov_filtering():
    suite = SensorSuite(weather_factor=0.1)
    ego_state = np.array([0.0, 0.0, 10.0, 0.0]) # at origin, 10 m/s forward

    actors = [
        # Actor 1: 30m directly in front (inside Camera, LiDAR, Radar)
        {
            "id": 101,
            "class": "Auto-Rickshaw",
            "position": [30.0, 0.0, 0.0],
            "velocity": [8.0, 0.0, 0.0],
            "dimensions": [2.7, 1.4, 1.8],
        },
        # Actor 2: 120m ahead (outside Camera 80m and LiDAR 90m, but inside Radar 150m)
        {
            "id": 102,
            "class": "Truck-Bus",
            "position": [120.0, 0.0, 0.0],
            "velocity": [15.0, 0.0, 0.0],
            "dimensions": [11.0, 2.6, 3.2],
        },
        # Actor 3: Behind ego vehicle (x = -20m, outside all forward sensors)
        {
            "id": 103,
            "class": "Two-Wheeler",
            "position": [-20.0, 0.0, 0.0],
            "velocity": [12.0, 0.0, 0.0],
            "dimensions": [2.0, 0.8, 1.3],
        },
    ]

    detections = suite.generate_measurements(ego_state, actors, t=0.0)

    # Actor 1 should be detected by Camera, LiDAR, and Radar
    cam_ids = [d.target_id for d in detections["Camera"]]
    lidar_ids = [d.target_id for d in detections["LiDAR"]]
    radar_ids = [d.target_id for d in detections["Radar"]]

    assert 101 in cam_ids
    assert 101 in lidar_ids
    assert 101 in radar_ids

    # Actor 2 (120m) should ONLY be detected by Radar
    assert 102 not in cam_ids
    assert 102 not in lidar_ids
    assert 102 in radar_ids

    # Actor 3 (behind) should NOT be detected by any forward sensor
    assert 103 not in cam_ids
    assert 103 not in lidar_ids
    assert 103 not in radar_ids


def test_fusion_engine_tracking_and_confirmation():
    engine = SensorFusionEngine(dt=0.05)
    suite = SensorSuite()

    ego_state = np.array([0.0, 0.0, 10.0, 0.0])
    actors = [{
        "id": 201,
        "class": "Pedestrian",
        "position": [25.0, 1.5, 0.0],
        "velocity": [0.0, -1.0, 0.0],
        "dimensions": [0.5, 0.5, 1.7],
    }]

    # Step 1: Initial measurement -> Track initiated as Tentative
    dets1 = suite.generate_measurements(ego_state, actors, t=0.0)
    tracks1 = engine.step(dets1, current_time=0.0)
    assert len(tracks1) >= 1
    tr1 = tracks1[0]
    assert tr1.object_class == "Pedestrian"
    assert tr1.distance == pytest.approx(math.hypot(25.0, 1.5), abs=1.5)

    # Step 2: Second measurement update -> Promoted to Confirmed
    dets2 = suite.generate_measurements(ego_state, actors, t=0.05)
    tracks2 = engine.step(dets2, current_time=0.05)
    assert len(tracks2) >= 1
    tr2 = tracks2[0]
    assert tr2.track_status == "Confirmed"
    assert tr2.confidence_score >= 0.70
    assert len(tr2.sensor_contributors) >= 2


def test_fusion_engine_output_fields():
    engine = SensorFusionEngine(dt=0.05)
    suite = SensorSuite()

    ego_state = np.array([0.0, 0.0, 12.0, 0.0])
    actors = [{
        "id": 301,
        "class": "Cattle",
        "position": [45.0, -0.5, 0.0],
        "velocity": [0.0, 0.0, 0.0],
        "dimensions": [2.2, 0.9, 1.5],
    }]

    # Run for 5 steps
    for step in range(5):
        t = step * 0.05
        dets = suite.generate_measurements(ego_state, actors, t)
        tracks = engine.step(dets, t)

    assert len(tracks) >= 1
    target = tracks[0]

    # Verify all 5 required fields
    assert hasattr(target, "object_class") and target.object_class == "Cattle"
    assert hasattr(target, "position") and len(target.position) == 3
    assert hasattr(target, "velocity") and len(target.velocity) == 3
    assert hasattr(target, "distance") and target.distance > 0.0
    assert hasattr(target, "confidence_score") and 0.0 <= target.confidence_score <= 1.0
