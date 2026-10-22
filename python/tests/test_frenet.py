"""Unit tests for 2D Cubic Spline reference paths and Frenet frame transformations."""

import math
import numpy as np
import pytest
from bharatdrive.models.state import VehicleState, FrenetState
from bharatdrive.geometry.frenet import (
    CubicSpline2D,
    cartesian_to_frenet,
    frenet_to_cartesian,
)


def test_spline_straight_line():
    wx = [0.0, 10.0, 20.0, 50.0]
    wy = [0.0, 0.0, 0.0, 0.0]
    spline = CubicSpline2D(wx, wy)

    assert spline.total_length == pytest.approx(50.0)
    x, y = spline.calc_position(25.0)
    assert x == pytest.approx(25.0)
    assert y == pytest.approx(0.0)

    # Tangent angle should be zero along X axis
    theta = spline.calc_tangent_angle(25.0)
    assert theta == pytest.approx(0.0, abs=1e-5)

    # Curvature on straight line is zero
    kappa = spline.calc_curvature(25.0)
    assert abs(kappa) < 1e-4


def test_spline_curved_path():
    # 90-degree circular curve approximated by waypoints
    angles = np.linspace(0, math.pi / 2.0, 10)
    radius = 50.0
    wx = radius * np.cos(angles)
    wy = radius * np.sin(angles)
    # Order from start (radius, 0) to end (0, radius)
    spline = CubicSpline2D(wx, wy)

    # Projection of point (radius, 0) should be near s=0
    s_proj = spline.find_projection(radius, 0.0)
    assert s_proj == pytest.approx(0.0, abs=0.5)

    # Projection of endpoint (0, radius) should be near total_length
    s_end = spline.find_projection(0.0, radius)
    assert s_end == pytest.approx(spline.total_length, abs=0.5)


def test_cartesian_to_frenet_and_back_straight_road():
    wx = [0.0, 20.0, 40.0, 80.0, 120.0]
    wy = [0.0, 0.0, 0.0, 0.0, 0.0]
    ref_path = CubicSpline2D(wx, wy)

    # Vehicle at x=30, y=1.2 (offset left by 1.2m), moving at v=10 m/s with small heading angle
    orig_state = VehicleState(x=30.0, y=1.2, v=10.0, yaw=0.05, a=0.5)

    frenet_state = cartesian_to_frenet(orig_state, ref_path)
    assert frenet_state.s == pytest.approx(30.0, abs=1e-2)
    assert frenet_state.d == pytest.approx(1.2, abs=1e-2)
    assert frenet_state.s_d > 0.0

    # Convert back to Cartesian
    recon_state = frenet_to_cartesian(frenet_state, ref_path)
    assert recon_state.x == pytest.approx(orig_state.x, abs=1e-2)
    assert recon_state.y == pytest.approx(orig_state.y, abs=1e-2)
    assert recon_state.v == pytest.approx(orig_state.v, abs=1e-2)
    assert recon_state.yaw == pytest.approx(orig_state.yaw, abs=1e-2)


def test_cartesian_to_frenet_curved_road():
    # S-curve road
    wx = [0.0, 30.0, 60.0, 90.0, 120.0]
    wy = [0.0, 5.0, 0.0, -5.0, 0.0]
    ref_path = CubicSpline2D(wx, wy)

    s_target = 45.0
    rx, ry = ref_path.calc_position(s_target)
    theta_r = ref_path.calc_tangent_angle(s_target)

    # Position vehicle 1.0 m to the left of the road normal
    norm_x = -math.sin(theta_r)
    norm_y = math.cos(theta_r)
    veh_x = rx + 1.0 * norm_x
    veh_y = ry + 1.0 * norm_y

    cart_state = VehicleState(x=veh_x, y=veh_y, v=12.0, yaw=theta_r, a=0.0)
    frenet_state = cartesian_to_frenet(cart_state, ref_path, s_guess=s_target)

    assert frenet_state.s == pytest.approx(s_target, abs=0.1)
    assert frenet_state.d == pytest.approx(1.0, abs=0.05)

    recon = frenet_to_cartesian(frenet_state, ref_path)
    assert recon.x == pytest.approx(veh_x, abs=0.05)
    assert recon.y == pytest.approx(veh_y, abs=0.05)
    assert recon.v == pytest.approx(12.0, abs=0.05)
