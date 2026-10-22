"""
Unit Tests for End-to-End Autonomous Driving Pipeline.
BharatDrive-AI - MathWorks Autonomous Driving Hackathon.
"""

import math
import numpy as np
import pytest

from bharatdrive.integration.system_pipeline import BharatDrivePipeline


class TestEndToEndIntegration:
    """Tests for the complete closed-loop 8-module pipeline."""

    def test_pipeline_initialization(self):
        pipe = BharatDrivePipeline(cruise_speed=13.89, road_width=7.00, friction_mu=0.85)
        assert pipe.ego_state.v == 8.0
        assert pipe.road_width == 7.00
        assert pipe.planner is not None
        assert pipe.decision_supervisor is not None
        assert pipe.controller is not None
        assert pipe.plant is not None

    def test_single_step_feedforward_and_feedback(self):
        pipe = BharatDrivePipeline()
        obstacles = [
            {"id": 1, "class": "Cattle", "position": [30.0, 0.5], "velocity": [0.0, 0.0]},
            {"id": 2, "class": "Auto-Rickshaw", "position": [45.0, -1.5], "velocity": [5.0, 0.0]},
        ]

        record = pipe.step(sim_time=0.05, obstacles=obstacles)

        assert "ego_x" in record
        assert "ego_y" in record
        assert "ego_v" in record
        assert "steer_cmd" in record
        assert "accel_cmd" in record
        assert "state_name" in record

        # Actuator physical limits
        assert -0.55 <= record["steer_cmd"] <= 0.55
        assert -6.50 <= record["accel_cmd"] <= 3.50

    def test_closed_loop_cow_avoidance(self):
        pipe = BharatDrivePipeline(cruise_speed=12.0)
        # Stationary cow at x=25m, y=+0.6m (partially blocking right side)
        obstacles = [
            {"id": 1, "class": "Cattle", "position": [25.0, 0.6], "velocity": [0.0, 0.0]},
        ]

        states_observed = []
        nudges_observed = []

        # Run 35 cycles (1.75 seconds of driving)
        for step in range(35):
            t = step * pipe.dt
            rec = pipe.step(t, obstacles)
            states_observed.append(rec["state_name"])
            nudges_observed.append(rec["nudge"])

        # Pipeline triggers Move Left and vehicle laterally displaces left to avoid cow on right
        assert "Move Left" in states_observed, "Pipeline should trigger Move Left for obstacle on right"
        assert pipe.ego_state.y < -0.5, "Vehicle should laterally displace left away from cow"
        # Vehicle must progress forward while maneuvering
        assert pipe.ego_state.x > 8.0

    def test_emergency_stop_intervention(self):
        pipe = BharatDrivePipeline(cruise_speed=12.0)
        # Sudden critical pedestrian sprint directly in vehicle path (4.0m ahead)
        critical_obstacles = [
            {"id": 1, "class": "Pedestrian", "position": [4.0, 0.0], "velocity": [0.0, 0.0]},
        ]

        rec = pipe.step(0.05, critical_obstacles)

        # High-risk pedestrian intrusion triggers Emergency Stop
        assert rec["state_name"] == "Emergency Stop"
        assert math.isclose(rec["accel_cmd"], -6.50, abs_tol=1e-3)
        assert rec["hazards"] is True

    def test_road_boundary_integrity(self):
        pipe = BharatDrivePipeline(cruise_speed=12.0, road_width=7.00)
        obstacles = [
            {"id": 1, "class": "Cattle", "position": [20.0, 0.5], "velocity": [0.0, 0.0]},
            {"id": 2, "class": "Pothole", "position": [40.0, -1.0], "velocity": [0.0, 0.0]},
        ]

        half_w = pipe.road_width / 2.0
        y_positions = []

        for step in range(40):
            t = step * pipe.dt
            rec = pipe.step(t, obstacles)
            y_positions.append(rec["ego_y"])

        # Vehicle must remain strictly within road envelope throughout the drive
        assert np.all(np.abs(y_positions) <= half_w), f"Vehicle veered off road! Max Y = {max(np.abs(y_positions))}"
