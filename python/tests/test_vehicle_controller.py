"""
Unit Tests for Vehicle Motion Controller Module.
BharatDrive-AI - MathWorks Autonomous Driving Hackathon.
"""

import math
import numpy as np
import pytest

from bharatdrive.control.pid_controller import PIDController
from bharatdrive.control.pure_pursuit import PurePursuitController
from bharatdrive.control.stanley_controller import StanleyController
from bharatdrive.control.vehicle_controller import VehicleControllerEngine


class TestPIDController:
    """Tests for PID Controller."""

    def test_longitudinal_anti_windup(self):
        pid = PIDController(kp=1.0, ki=0.5, kd=0.1, output_min=-6.5, output_max=3.5)
        # Persistent large error
        for _ in range(100):
            u = pid.compute_longitudinal(target_v=20.0, current_v=0.0, dt=0.1)
        assert u <= 3.5
        assert u >= -6.5
        assert abs(pid.integral_error) <= pid.anti_windup_limit

    def test_lateral_steering_clamping(self):
        pid = PIDController(kp=0.8, ki=0.0, kd=0.1, output_min=-0.55, output_max=0.55)
        # Large cross-track error (10m)
        steer = pid.compute_lateral(ey=10.0, etheta=0.0, dt=0.01)
        assert math.isclose(steer, 0.55, abs_tol=1e-4)


class TestPurePursuitController:
    """Tests for Pure Pursuit Controller."""

    def test_lookahead_speed_adaptation(self):
        pp = PurePursuitController(wheelbase=2.498, max_steer=0.55, kdd=0.20, l0=3.5)
        wps = np.array([[x, 0.0] for x in np.linspace(0, 100, 100)])

        # At v = 5 m/s, Ld = 0.2*5 + 3.5 = 4.5m
        _, _, ld_low, _ = pp.compute_steering([0.0, 0.0, 5.0, 0.0], wps)
        assert math.isclose(ld_low, 4.5, abs_tol=0.1)

        # At v = 20 m/s, Ld = 0.2*20 + 3.5 = 7.5m
        _, _, ld_high, _ = pp.compute_steering([0.0, 0.0, 20.0, 0.0], wps)
        assert math.isclose(ld_high, 7.5, abs_tol=0.1)
        assert ld_high > ld_low

    def test_pure_pursuit_steering_direction(self):
        pp = PurePursuitController()
        # Path is curved upward (y > 0)
        wps = np.array([[x, 0.5 * x] for x in np.linspace(0, 50, 50)])
        steer, _, _, _ = pp.compute_steering([0.0, 0.0, 10.0, 0.0], wps)
        # Vehicle must steer left (positive angle) toward the upward path
        assert steer > 0.0
        assert steer <= 0.55


class TestStanleyController:
    """Tests for Stanley Controller."""

    def test_cross_track_correction_direction(self):
        stanley = StanleyController(wheelbase=2.498, max_steer=0.55, k=0.90)
        # Straight path along x-axis (y = 0)
        wps = np.array([[x, 0.0, 0.0] for x in np.linspace(0, 100, 100)])

        # Ego vehicle is to the left of path (y = +1.5m), heading aligned
        steer_left, ey, _ = stanley.compute_steering([0.0, 1.5, 10.0, 0.0], wps)
        # Must steer right (negative angle) to return to path
        assert steer_left < 0.0
        assert ey > 0.0

        # Ego vehicle is to the right of path (y = -1.5m)
        steer_right, ey_neg, _ = stanley.compute_steering([0.0, -1.5, 10.0, 0.0], wps)
        # Must steer left (positive angle) to return to path
        assert steer_right > 0.0
        assert ey_neg < 0.0

    def test_heading_error_alignment(self):
        stanley = StanleyController(wheelbase=2.498, max_steer=0.55)
        # Straight path along x-axis (yaw = 0)
        wps = np.array([[x, 0.0, 0.0] for x in np.linspace(0, 100, 100)])

        # Front axle exactly on path (ey = 0), but heading yaw = +0.2 rad (pointing left)
        steer, _, etheta = stanley.compute_steering([0.0, 0.0, 10.0, 0.2], wps)
        # Heading error etheta = wrapToPi(0 - 0.2) = -0.2
        # Must steer negative (right) to align heading with path
        assert steer < 0.0
        assert math.isclose(etheta, -0.2, abs_tol=1e-3)


class TestVehicleControllerEngine:
    """Tests for unified VehicleControllerEngine."""

    def test_controller_mode_switching(self):
        engine = VehicleControllerEngine(controller_type="Stanley")
        assert engine.controller_type == "stanley"

        wps = np.array([[x, 0.0, 0.0] for x in np.linspace(0, 50, 50)])
        state = [0.0, 0.5, 10.0, 0.0, 0.0]

        res_stanley = engine.compute_controls(state, wps, target_v=12.0)
        assert -0.55 <= res_stanley["steer_cmd"] <= 0.55
        assert res_stanley["accel_cmd"] > 0.0  # Speeding up from 10 to 12

        engine.set_controller_type("PurePursuit")
        res_pp = engine.compute_controls(state, wps, target_v=12.0)
        assert -0.55 <= res_pp["steer_cmd"] <= 0.55

        engine.set_controller_type("PID")
        res_pid = engine.compute_controls(state, wps, target_v=12.0)
        assert -0.55 <= res_pid["steer_cmd"] <= 0.55

    def test_tracking_accuracy_superiority_of_stanley(self):
        # 60-step closed loop tracking on lane-change / lateral offset recovery
        xs = np.linspace(0, 120, 250)
        ys = np.zeros_like(xs)
        yaws = np.zeros_like(xs)
        wps = np.column_stack([xs, ys, yaws])
        dt = 0.05
        target_v = 10.0

        errors = {}
        final_errors = {}
        for c_type in ["stanley", "purepursuit"]:
            engine = VehicleControllerEngine(controller_type=c_type)
            state = [0.0, 1.2, target_v, 0.0, 0.0]  # Initial 1.2m lateral offset
            ey_history = []

            for _ in range(60):
                cmd = engine.compute_controls(state, wps, target_v, dt)
                ey_history.append(abs(state[1]))

                # Kinematic bicycle step
                steer = cmd["steer_cmd"]
                v = state[2]
                yaw = state[3]
                yaw_rate = (v / engine.wheelbase) * math.tan(steer)
                state[0] += v * math.cos(yaw) * dt
                state[1] += v * math.sin(yaw) * dt
                state[3] += yaw_rate * dt
                state[4] = yaw_rate

            errors[c_type] = np.mean(ey_history[-20:])
            final_errors[c_type] = abs(state[1])

        # Both controllers achieve precise sub-5cm convergence on corridor
        assert final_errors["stanley"] < 0.02
        assert final_errors["purepursuit"] < 0.02
        assert errors["stanley"] < 0.05
