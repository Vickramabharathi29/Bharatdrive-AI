"""Coordinate transformations and polygon collision detection (Separating Axis Theorem).

Supports:
- Cartesian Global to Local Body Frame (ISO 8855 Forward-Left-Up)
- Local Body to Cartesian Global Frame
- Exact Oriented Bounding Box (OBB) calculations for heterogeneous vehicles
- Separating Axis Theorem (SAT) for convex 2D polygon collision verification
"""

from __future__ import annotations
import math
import numpy as np
from typing import Sequence


def cartesian_to_body(
    points_global: np.ndarray,
    ego_x: float,
    ego_y: float,
    ego_yaw: float,
) -> np.ndarray:
    """Transforms 2D global Cartesian points to the vehicle body frame (Forward-Left).
    
    Args:
        points_global: Array of shape (N, 2) or (2,) in global coordinates.
        ego_x: Ego vehicle X position in global frame.
        ego_y: Ego vehicle Y position in global frame.
        ego_yaw: Ego vehicle heading angle in radians.
        
    Returns:
        np.ndarray: Array of shape (N, 2) in vehicle body frame.
    """
    pts = np.atleast_2d(points_global)
    dx = pts[:, 0] - ego_x
    dy = pts[:, 1] - ego_y

    cos_yaw = math.cos(ego_yaw)
    sin_yaw = math.sin(ego_yaw)

    x_body = dx * cos_yaw + dy * sin_yaw
    y_body = -dx * sin_yaw + dy * cos_yaw

    res = np.column_stack((x_body, y_body))
    return res[0] if points_global.ndim == 1 else res


def body_to_cartesian(
    points_body: np.ndarray,
    ego_x: float,
    ego_y: float,
    ego_yaw: float,
) -> np.ndarray:
    """Transforms 2D local vehicle body points back to global Cartesian frame.
    
    Args:
        points_body: Array of shape (N, 2) or (2,) in body coordinates.
        ego_x: Ego vehicle X position in global frame.
        ego_y: Ego vehicle Y position in global frame.
        ego_yaw: Ego vehicle heading angle in radians.
        
    Returns:
        np.ndarray: Array of shape (N, 2) in global Cartesian frame.
    """
    pts = np.atleast_2d(points_body)
    cos_yaw = math.cos(ego_yaw)
    sin_yaw = math.sin(ego_yaw)

    x_glob = pts[:, 0] * cos_yaw - pts[:, 1] * sin_yaw + ego_x
    y_glob = pts[:, 0] * sin_yaw + pts[:, 1] * cos_yaw + ego_y

    res = np.column_stack((x_glob, y_glob))
    return res[0] if points_body.ndim == 1 else res


def compute_vehicle_corners(
    center_x: float,
    center_y: float,
    yaw: float,
    length: float,
    width: float,
    center_to_front: float | None = None,
) -> np.ndarray:
    """Computes the 4 corners of an oriented bounding box (OBB).
    
    Args:
        center_x: Vehicle reference point X [m].
        center_y: Vehicle reference point Y [m].
        yaw: Orientation angle [rad].
        length: Total bounding length [m].
        width: Total bounding width [m].
        center_to_front: Distance from reference point to front edge (default length/2).
        
    Returns:
        np.ndarray: (4, 2) corner points [FL, FR, RR, RL] in global frame.
    """
    if center_to_front is None:
        front_d = length / 2.0
        rear_d = -length / 2.0
    else:
        front_d = center_to_front
        rear_d = -(length - center_to_front)

    half_w = width / 2.0
    local_box = np.array([
        [front_d, half_w],   # FL
        [front_d, -half_w],  # FR
        [rear_d, -half_w],   # RR
        [rear_d, half_w],    # RL
    ])

    return body_to_cartesian(local_box, center_x, center_y, yaw)


def check_polygon_collision_sat(poly_a: np.ndarray, poly_b: np.ndarray) -> bool:
    """Determines if two convex 2D polygons collide using the Separating Axis Theorem (SAT).
    
    Args:
        poly_a: (N, 2) vertices of convex polygon A in counter-clockwise or clockwise order.
        poly_b: (M, 2) vertices of convex polygon B.
        
    Returns:
        bool: True if polygons intersect or overlap, False if a separating axis exists (no collision).
    """
    for polygon in [poly_a, poly_b]:
        num_vertices = len(polygon)
        for i in range(num_vertices):
            p1 = polygon[i]
            p2 = polygon[(i + 1) % num_vertices]
            edge = p2 - p1

            # Normal vector perpendicular to the edge
            normal = np.array([-edge[1], edge[0]], dtype=np.float64)
            norm = np.linalg.norm(normal)
            if norm < 1e-9:
                continue
            axis = normal / norm

            # Project poly_a onto axis
            proj_a = np.dot(poly_a, axis)
            min_a, max_a = np.min(proj_a), np.max(proj_a)

            # Project poly_b onto axis
            proj_b = np.dot(poly_b, axis)
            min_b, max_b = np.min(proj_b), np.max(proj_b)

            # Check for separation along this axis
            if max_a < min_b or max_b < min_a:
                return False  # Separating axis found, cannot collide

    return True  # Overlaps on all axes, collision confirmed
