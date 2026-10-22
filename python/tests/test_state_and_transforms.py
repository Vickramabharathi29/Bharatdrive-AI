"""Unit tests for vehicle state representation, coordinate transformations, and SAT collision checks."""

import math
import numpy as np
import pytest
from bharatdrive.config import VehicleDimensions
from bharatdrive.models.state import VehicleState, FrenetState
from bharatdrive.geometry.transforms import (
    cartesian_to_body,
    body_to_cartesian,
    compute_vehicle_corners,
    check_polygon_collision_sat,
)


def test_vehicle_state_initialization_and_normalization():
    state = VehicleState(x=10.0, y=5.0, v=15.0, yaw=3 * math.pi)
    # Yaw should normalize into [-pi, pi]
    assert state.yaw == pytest.approx(math.pi, rel=1e-4) or state.yaw == pytest.approx(-math.pi, rel=1e-4)
    vec = state.as_vector()
    assert len(vec) == 7
    assert vec[0] == 10.0
    assert vec[1] == 5.0
    assert vec[2] == 15.0


def test_state_reconstruction_from_vector():
    vec = np.array([12.5, -4.2, 8.0, 0.45, 0.05, 1.2, 0.1])
    state = VehicleState.from_vector(vec, t=1.5)
    assert state.x == pytest.approx(12.5)
    assert state.y == pytest.approx(-4.2)
    assert state.v == pytest.approx(8.0)
    assert state.yaw == pytest.approx(0.45)
    assert state.t == pytest.approx(1.5)


def test_bounding_box_corners():
    dims = VehicleDimensions(length=4.0, width=2.0, wheelbase=2.5, front_overhang=0.8, rear_overhang=0.7)
    state = VehicleState(x=0.0, y=0.0, yaw=0.0)
    corners = state.get_corners(dims)
    assert corners.shape == (4, 2)
    # Front-left corner x should be wheelbase + front_overhang = 3.3, y = 1.0
    assert corners[0, 0] == pytest.approx(3.3, abs=1e-3)
    assert corners[0, 1] == pytest.approx(1.0, abs=1e-3)
    # Rear-right corner x should be -rear_overhang = -0.7, y = -1.0
    assert corners[2, 0] == pytest.approx(-0.7, abs=1e-3)
    assert corners[2, 1] == pytest.approx(-1.0, abs=1e-3)


def test_coordinate_transforms_roundtrip():
    ego_x, ego_y, ego_yaw = 20.0, -15.0, math.pi / 4.0
    global_pts = np.array([
        [25.0, -10.0],
        [18.0, -20.0],
        [30.0, 0.0],
    ])

    body_pts = cartesian_to_body(global_pts, ego_x, ego_y, ego_yaw)
    recovered_global = body_to_cartesian(body_pts, ego_x, ego_y, ego_yaw)

    np.testing.assert_allclose(global_pts, recovered_global, atol=1e-6)


def test_sat_collision_detection():
    # Box 1 at origin, width=2, height=2
    box_a = np.array([
        [1.0, 1.0],
        [-1.0, 1.0],
        [-1.0, -1.0],
        [1.0, -1.0],
    ])

    # Box 2 overlapping Box 1
    box_overlap = np.array([
        [1.5, 0.5],
        [0.0, 0.5],
        [0.0, -0.5],
        [1.5, -0.5],
    ])
    assert check_polygon_collision_sat(box_a, box_overlap) is True

    # Box 3 strictly disjoint (separated along X)
    box_disjoint = np.array([
        [5.0, 1.0],
        [3.0, 1.0],
        [3.0, -1.0],
        [5.0, -1.0],
    ])
    assert check_polygon_collision_sat(box_a, box_disjoint) is False

    # Box 4 rotated diagonally without overlap
    rot_angle = math.pi / 4.0
    c, s = math.cos(rot_angle), math.sin(rot_angle)
    r_mat = np.array([[c, -s], [s, c]])
    box_rot_separated = (r_mat @ box_a.T).T + np.array([4.0, 4.0])
    assert check_polygon_collision_sat(box_a, box_rot_separated) is False
