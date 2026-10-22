"""
Unit Tests for Stateflow Tactical Decision Logic.
BharatDrive-AI - MathWorks Autonomous Driving Hackathon.
"""

import pytest
import numpy as np

from bharatdrive.decision.maneuver_state import ManeuverState
from bharatdrive.decision.stateflow_supervisor import StateflowDecisionSupervisor


class TestStateflowDecision:
    """Tests for the 8 tactical driving states in StateflowDecisionSupervisor."""

    def test_accelerate_state(self):
        supervisor = StateflowDecisionSupervisor(cruise_speed=13.89)
        ego = [0.0, 0.0, 8.0]
        threat = {"Distance": 80.0, "RiskScore": 0.10, "Class": "None", "TTC": float("inf")}

        dec = supervisor.step(ego, threat)
        assert dec["state"] == ManeuverState.ACCELERATE
        assert dec["target_speed"] == 13.89
        assert dec["target_accel"] > 0.0
        assert dec["target_lateral_offset"] == 0.0

    def test_brake_state(self):
        supervisor = StateflowDecisionSupervisor(cruise_speed=13.89)
        ego = [10.0, 0.0, 12.0]
        # Lead vehicle 18m ahead with 5 m/s speed delta
        threat = {
            "Distance": 18.0,
            "VRel": 5.0,
            "Position": [28.0, 0.0],
            "RiskScore": 0.55,
            "TTC": 3.6,
            "Class": "Auto-Rickshaw",
        }

        dec = supervisor.step(ego, threat)
        assert dec["state"] == ManeuverState.BRAKE
        assert dec["target_accel"] < 0.0
        assert dec["target_speed"] < ego[2]

    def test_move_left_state(self):
        supervisor = StateflowDecisionSupervisor(cruise_speed=13.89)
        supervisor.state_time = 0.50  # Clear hysteresis
        ego = [20.0, 0.0, 10.0]
        # Stationary cow at x=38m, y=+0.8m (right of centerline)
        threat = {
            "Distance": 18.0,
            "VRel": 10.0,
            "Position": [38.0, 0.8],
            "RiskScore": 0.60,
            "TTC": 1.8,
            "Class": "Cattle",
        }
        env = {"RoadWidth": 7.5, "LeftClearance": 2.5, "RightClearance": 1.0, "OncomingDist": 100.0}

        dec = supervisor.step(ego, threat, env)
        assert dec["state"] == ManeuverState.MOVE_LEFT
        assert dec["target_lateral_offset"] < 0.0  # Shift left

    def test_move_right_state(self):
        supervisor = StateflowDecisionSupervisor(cruise_speed=13.89)
        supervisor.state_time = 0.50
        ego = [30.0, 0.0, 10.0]
        # Severe pothole at x=45m, y=-0.9m (left wheel track)
        threat = {
            "Distance": 15.0,
            "VRel": 10.0,
            "Position": [45.0, -0.9],
            "RiskScore": 0.55,
            "TTC": 1.5,
            "Class": "Pothole",
        }
        env = {"RoadWidth": 7.0, "LeftClearance": 1.0, "RightClearance": 2.5, "OncomingDist": 80.0}

        dec = supervisor.step(ego, threat, env)
        assert dec["state"] == ManeuverState.MOVE_RIGHT
        assert dec["target_lateral_offset"] > 0.0  # Shift right

    def test_overtake_state_progression(self):
        supervisor = StateflowDecisionSupervisor(cruise_speed=13.89)
        supervisor.state_time = 0.50
        ego = [50.0, 0.0, 10.0]
        # Tractor moving at 4.0 m/s (v_rel = 6.0 m/s), 15m ahead
        lead_x = 65.0
        threat = {
            "Distance": 15.0,
            "VRel": 6.0,
            "Position": [lead_x, 0.0],
            "RiskScore": 0.35,
            "TTC": 2.5,
            "Class": "Truck-Bus",
        }
        env = {"RoadWidth": 8.0, "RightClearance": 3.0, "OncomingDist": 120.0}

        # Step 1: Initiate Overtake (Phase 1)
        dec1 = supervisor.step(ego, threat, env, dt=0.1)
        assert dec1["state"] == ManeuverState.OVERTAKE
        assert dec1["overtake_phase"] == 1
        assert dec1["target_lateral_offset"] == 1.80

        # Step 2: Transition to Passing (Phase 2)
        supervisor.state_time = 1.30
        dec2 = supervisor.step(ego, threat, env, dt=0.1)
        assert dec2["overtake_phase"] == 2
        assert dec2["target_accel"] > 0.0

        # Step 3: Transition to Cut-back (Phase 3) when ego passes lead
        ego_advanced = [lead_x + 8.0, 1.8, 14.0]
        dec3 = supervisor.step(ego_advanced, threat, env, dt=0.1)
        assert dec3["overtake_phase"] == 3

        # Step 4: Completion back to Accelerate
        supervisor.state_time = 1.60
        dec4 = supervisor.step([lead_x + 16.0, 0.0, 13.89], threat, env, dt=0.1)
        assert dec4["state"] == ManeuverState.ACCELERATE
        assert dec4["overtake_phase"] == 0

    def test_overtake_abort_on_oncoming_threat(self):
        supervisor = StateflowDecisionSupervisor(cruise_speed=13.89)
        supervisor.state_time = 0.50
        ego = [50.0, 0.0, 10.0]
        threat = {"Distance": 15.0, "VRel": 6.0, "Position": [65.0, 0.0], "Class": "Auto-Rickshaw"}
        env = {"RoadWidth": 7.5, "RightClearance": 2.5, "OncomingDist": 100.0}

        # Start overtake
        dec = supervisor.step(ego, threat, env, dt=0.1)
        assert dec["state"] == ManeuverState.OVERTAKE

        # Sudden oncoming truck appears at 35m
        env_threat = {"RoadWidth": 7.5, "RightClearance": 2.5, "OncomingDist": 35.0}
        dec_abort = supervisor.step(ego, threat, env_threat, dt=0.1)
        assert dec_abort["state"] == ManeuverState.BRAKE
        assert dec_abort["overtake_phase"] == 0

    def test_wait_state_and_horn_activation(self):
        supervisor = StateflowDecisionSupervisor(cruise_speed=13.89)
        ego = [80.0, 0.0, 1.5]
        # Cow crossing in front (7m away)
        threat = {
            "Distance": 7.0,
            "VRel": 1.5,
            "Position": [87.0, 0.2],
            "RiskScore": 0.70,
            "TTC": 4.6,
            "Class": "Cattle",
        }

        dec = supervisor.step(ego, threat, dt=0.1)
        assert dec["state"] == ManeuverState.WAIT
        assert dec["target_speed"] == 0.0
        assert dec["hazard_lights"] is True
        assert dec["horn"] is False

        # After waiting for > 1.5s, horn activates to clear animal
        supervisor.state_time = 2.0
        dec_horn = supervisor.step(ego, threat, dt=0.1)
        assert dec_horn["horn"] is True

    def test_stop_state_red_signal(self):
        supervisor = StateflowDecisionSupervisor(cruise_speed=13.89)
        ego = [100.0, 0.0, 8.0]
        threat = {"Distance": 30.0, "Class": "Roadblock"}
        env = {"SignalState": "Red"}

        dec = supervisor.step(ego, threat, env)
        assert dec["state"] == ManeuverState.STOP
        assert dec["target_speed"] == 0.0
        assert dec["target_accel"] == -2.50

        # When signal turns green, resume acceleration
        env_green = {"SignalState": "Green"}
        dec_green = supervisor.step(ego, {"Distance": 50.0, "Class": "None"}, env_green)
        assert dec_green["state"] == ManeuverState.ACCELERATE

    def test_emergency_stop_preemption(self):
        supervisor = StateflowDecisionSupervisor(cruise_speed=13.89)
        ego = [120.0, 0.0, 11.0]
        # Critical pedestrian sprint (TTC = 0.5s, Risk = 0.95)
        threat = {
            "Distance": 5.0,
            "VRel": 10.0,
            "Position": [125.0, 0.1],
            "RiskScore": 0.95,
            "TTC": 0.50,
            "Class": "Pedestrian",
        }

        dec = supervisor.step(ego, threat)
        assert dec["state"] == ManeuverState.EMERGENCY_STOP
        assert dec["target_speed"] == 0.0
        assert dec["target_accel"] == -6.50
        assert dec["hazard_lights"] is True
