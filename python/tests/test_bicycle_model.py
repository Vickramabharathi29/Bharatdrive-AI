"""Unit tests for kinematic and dynamic bicycle models."""

import math
import numpy as np
import pytest
from bharatdrive.config import VehicleConfig
from bharatdrive.models.state import VehicleState
from bharatdrive.models.bicycle_model import KinematicBicycleModel, DynamicBicycleModel


def test_kinematic_straight_line_motion():
    model = KinematicBicycleModel()
    init_state = VehicleState(x=0.0, y=0.0, v=10.0, yaw=0.0)

    # 1 second of straight motion at zero acceleration and zero steer
    state = init_state
    dt = 0.1
    for _ in range(10):
        state = model.step(state, accel_cmd=0.0, steer_cmd=0.0, dt=dt, method="rk4")

    # In 1 second at 10 m/s, vehicle should travel ~10 meters along X
    assert state.x == pytest.approx(10.0, rel=1e-2)
    assert abs(state.y) < 1e-4
    assert abs(state.yaw) < 1e-4
    assert state.v == pytest.approx(10.0)


def test_kinematic_turning_radius():
    config = VehicleConfig()
    model = KinematicBicycleModel(config)
    L = config.dimensions.wheelbase
    steer = 0.2  # rad
    expected_R = L / math.tan(steer)
    v = 5.0  # m/s
    expected_yaw_rate = v / expected_R

    init_state = VehicleState(x=0.0, y=0.0, v=v, yaw=0.0, steer=steer)
    dt = 0.05
    next_state = model.step(init_state, accel_cmd=0.0, steer_cmd=steer, dt=dt, method="rk4")

    assert next_state.yaw_rate == pytest.approx(expected_yaw_rate, rel=1e-2)
    # After dt seconds, yaw should have increased by expected_yaw_rate * dt
    assert next_state.yaw == pytest.approx(expected_yaw_rate * dt, rel=1e-2)


def test_acceleration_and_deceleration_clamping():
    model = KinematicBicycleModel()
    init_state = VehicleState(x=0.0, y=0.0, v=10.0, yaw=0.0)

    # Attempt to command excessive acceleration (+20 m/s^2)
    s_accel = model.step(init_state, accel_cmd=20.0, steer_cmd=0.0, dt=0.1)
    max_a = model.config.limits.max_accel_mps2
    assert s_accel.a == pytest.approx(max_a)
    assert s_accel.v == pytest.approx(10.0 + max_a * 0.1)

    # Attempt to command excessive braking (-30 m/s^2)
    s_brake = model.step(init_state, accel_cmd=-30.0, steer_cmd=0.0, dt=0.1)
    max_d = model.config.limits.max_decel_mps2
    assert s_brake.a == pytest.approx(max_d)
    assert s_brake.v == pytest.approx(10.0 + max_d * 0.1)


def test_dynamic_bicycle_model_slip_and_stability():
    model = DynamicBicycleModel()
    # At zero steer, zero slip angles
    alpha_f, alpha_r = model.compute_slip_angles(vx=15.0, vy=0.0, omega=0.0, delta=0.0)
    assert abs(alpha_f) < 1e-6
    assert abs(alpha_r) < 1e-6

    # With front steer, positive front slip angle
    alpha_f, alpha_r = model.compute_slip_angles(vx=15.0, vy=0.0, omega=0.0, delta=0.1)
    assert alpha_f == pytest.approx(0.1, abs=1e-4)

    # Tire forces should be bounded by friction limit mu * Fz
    Fy_f, Fy_r = model.compute_tire_forces(alpha_f=1.0, alpha_r=1.0)
    Fz_f = (model.m * model.g * model.lr) / model.L
    max_Fy_f = model.mu * Fz_f
    assert abs(Fy_f) <= max_Fy_f + 1e-4

    # Integrate dynamic model for 10 steps at 15 m/s
    state = VehicleState(x=0.0, y=0.0, v=15.0, yaw=0.0)
    for _ in range(10):
        state = model.step(state, accel_cmd=0.0, steer_cmd=0.05, dt=0.05)
    assert state.x > 0.0
    assert state.y > 0.0
    assert not math.isnan(state.yaw)
    assert not math.isnan(state.v_y)
