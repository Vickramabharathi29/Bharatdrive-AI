"""Curvilinear Frenet-Serret Frame transformations and 2D Cubic Spline reference paths.

Provides mathematically rigorous transformations between Cartesian Global Frame (x, y, v, yaw, a, kappa)
and Frenet-Serret Frame (s, s_dot, s_ddot, d, d_dot, d_ddot).

Based on optimal trajectory generation formulation (Werling et al., Chu et al.).
"""

from __future__ import annotations
import math
import bisect
import numpy as np
from typing import Tuple, Sequence, List
from scipy.interpolate import CubicSpline
from bharatdrive.models.state import VehicleState, FrenetState


class CubicSpline1D:
    """Natural 1D Cubic Spline parameterized by monotonically increasing values."""

    def __init__(self, x: Sequence[float], y: Sequence[float]):
        self.x = np.asarray(x, dtype=np.float64)
        self.y = np.asarray(y, dtype=np.float64)
        self.spline = CubicSpline(self.x, self.y, bc_type="natural")
        self.dspline = self.spline.derivative(nu=1)
        self.ddspline = self.spline.derivative(nu=2)
        self.dddspline = self.spline.derivative(nu=3)

    def calc_position(self, x_val: float) -> float:
        return float(self.spline(x_val))

    def calc_first_derivative(self, x_val: float) -> float:
        return float(self.dspline(x_val))

    def calc_second_derivative(self, x_val: float) -> float:
        return float(self.ddspline(x_val))

    def calc_third_derivative(self, x_val: float) -> float:
        return float(self.dddspline(x_val))


class CubicSpline2D:
    """2D Natural Parametric Cubic Spline curve parameterized by cumulative Euclidean arc length s."""

    def __init__(self, x_waypoints: Sequence[float], y_waypoints: Sequence[float]):
        x_pts = np.asarray(x_waypoints, dtype=np.float64)
        y_pts = np.asarray(y_waypoints, dtype=np.float64)

        dx = np.diff(x_pts)
        dy = np.diff(y_pts)
        ds = np.hypot(dx, dy)

        self.s = np.concatenate(([0.0], np.cumsum(ds)))
        self.total_length = float(self.s[-1])

        self.sx = CubicSpline1D(self.s, x_pts)
        self.sy = CubicSpline1D(self.s, y_pts)

    def calc_position(self, s: float) -> Tuple[float, float]:
        """Evaluates Cartesian position (x, y) along spline at arc length s."""
        s_clamped = np.clip(s, 0.0, self.total_length)
        return self.sx.calc_position(s_clamped), self.sy.calc_position(s_clamped)

    def calc_tangent_angle(self, s: float) -> float:
        """Evaluates tangent heading angle theta_r = atan2(y', x') [rad]."""
        s_clamped = np.clip(s, 0.0, self.total_length)
        dx = self.sx.calc_first_derivative(s_clamped)
        dy = self.sy.calc_first_derivative(s_clamped)
        return float(math.atan2(dy, dx))

    def calc_curvature(self, s: float) -> float:
        """Evaluates signed curvature kappa_r(s) = (x' y'' - y' x'') / (x'^2 + y'^2)^(3/2) [1/m]."""
        s_clamped = np.clip(s, 0.0, self.total_length)
        dx = self.sx.calc_first_derivative(s_clamped)
        ddx = self.sx.calc_second_derivative(s_clamped)
        dy = self.sy.calc_first_derivative(s_clamped)
        ddy = self.sy.calc_second_derivative(s_clamped)

        denom = (dx**2 + dy**2)**1.5
        if denom < 1e-9:
            return 0.0
        return float((dx * ddy - dy * ddx) / denom)

    def calc_curvature_derivative(self, s: float) -> float:
        """Evaluates d(kappa)/ds via finite differences around s."""
        ds = 0.01
        k_plus = self.calc_curvature(s + ds)
        k_minus = self.calc_curvature(s - ds)
        return (k_plus - k_minus) / (2.0 * ds)

    def find_projection(self, x: float, y: float, s_guess: float | None = None) -> float:
        """Finds closest point s on spline to given (x, y) coordinates using coarse-to-fine optimization."""
        if s_guess is not None:
            s_min = max(0.0, s_guess - 10.0)
            s_max = min(self.total_length, s_guess + 10.0)
            s_samples = np.linspace(s_min, s_max, 50)
        else:
            num_samples = max(100, int(self.total_length / 0.5))
            s_samples = np.linspace(0.0, self.total_length, num_samples)

        px = np.array([self.sx.calc_position(si) for si in s_samples])
        py = np.array([self.sy.calc_position(si) for si in s_samples])
        dists_sq = (px - x)**2 + (py - y)**2
        best_idx = int(np.argmin(dists_sq))
        best_s = s_samples[best_idx]

        # Fine gradient descent refinement
        s_opt = best_s
        for _ in range(10):
            rx, ry = self.calc_position(s_opt)
            drx = self.sx.calc_first_derivative(s_opt)
            dry = self.sy.calc_first_derivative(s_opt)
            ddrx = self.sx.calc_second_derivative(s_opt)
            ddry = self.sy.calc_second_derivative(s_opt)

            diff_x = rx - x
            diff_y = ry - y

            # Objective: f(s) = (r(s) - p) . r'(s) = 0
            f = diff_x * drx + diff_y * dry
            df = drx**2 + dry**2 + diff_x * ddrx + diff_y * ddry

            if abs(df) < 1e-9:
                break
            step = f / df
            s_opt = np.clip(s_opt - step, 0.0, self.total_length)
            if abs(step) < 1e-4:
                break

        return float(s_opt)


def cartesian_to_frenet(
    state: VehicleState,
    reference_path: CubicSpline2D,
    s_guess: float | None = None,
) -> FrenetState:
    """Converts a Cartesian vehicle state into Frenet curvilinear coordinates.
    
    Args:
        state: Instantaneous Cartesian vehicle state (x, y, v, yaw, a).
        reference_path: Parametric reference spline.
        s_guess: Optional prior arc-length hint for computational acceleration.
        
    Returns:
        FrenetState: Decomposed (s, s_d, s_dd, d, d_d, d_dd).
    """
    s = reference_path.find_projection(state.x, state.y, s_guess)
    rx, ry = reference_path.calc_position(s)
    theta_r = reference_path.calc_tangent_angle(s)
    kappa_r = reference_path.calc_curvature(s)

    dx = state.x - rx
    dy = state.y - ry

    # Lateral distance d (cross product of tangent unit vector and delta position)
    # n_r = [-sin(theta_r), cos(theta_r)]
    d = -dx * math.sin(theta_r) + dy * math.cos(theta_r)

    # Relative heading angle
    delta_theta = math.atan2(math.sin(state.yaw - theta_r), math.cos(state.yaw - theta_r))

    denom = 1.0 - kappa_r * d
    if abs(denom) < 1e-4:
        denom = 1e-4 if denom >= 0 else -1e-4

    # Longitudinal and lateral velocities in Frenet frame
    s_d = (state.v * math.cos(delta_theta)) / denom
    d_d = state.v * math.sin(delta_theta)

    # Accelerations
    # d_dot_dot = a * sin(delta_theta) + v * (yaw_rate - theta_r_dot) * cos(delta_theta)
    theta_r_d = kappa_r * s_d
    yaw_rate = state.yaw_rate if abs(state.yaw_rate) > 1e-6 else (state.v / reference_path.total_length)
    d_dd = state.a * math.sin(delta_theta) + state.v * (yaw_rate - theta_r_d) * math.cos(delta_theta)
    s_dd = (state.a * math.cos(delta_theta) - state.v * (yaw_rate - theta_r_d) * math.sin(delta_theta)) / denom

    return FrenetState(
        s=float(s),
        s_d=float(s_d),
        s_dd=float(s_dd),
        d=float(d),
        d_d=float(d_d),
        d_dd=float(d_dd),
        t=state.t,
    )


def frenet_to_cartesian(
    frenet_state: FrenetState,
    reference_path: CubicSpline2D,
) -> VehicleState:
    """Converts a Frenet curvilinear state back into the Cartesian global frame.
    
    Args:
        frenet_state: Frenet state (s, s_d, s_dd, d, d_d, d_dd).
        reference_path: Parametric reference spline.
        
    Returns:
        VehicleState: Corresponding Cartesian vehicle state.
    """
    s = np.clip(frenet_state.s, 0.0, reference_path.total_length)
    rx, ry = reference_path.calc_position(s)
    theta_r = reference_path.calc_tangent_angle(s)
    kappa_r = reference_path.calc_curvature(s)

    # Cartesian position
    x = rx - frenet_state.d * math.sin(theta_r)
    y = ry + frenet_state.d * math.cos(theta_r)

    # Relative heading
    denom = frenet_state.s_d * (1.0 - kappa_r * frenet_state.d)
    if abs(denom) < 1e-4:
        delta_theta = 0.0
    else:
        delta_theta = math.atan2(frenet_state.d_d, denom)

    yaw = math.atan2(math.sin(theta_r + delta_theta), math.cos(theta_r + delta_theta))
    v = math.hypot(denom, frenet_state.d_d)

    # Longitudinal acceleration
    a = frenet_state.s_dd * math.cos(delta_theta) + frenet_state.d_dd * math.sin(delta_theta)
    yaw_rate = (frenet_state.s_d * kappa_r) + (frenet_state.d_dd / max(v, 0.1))

    return VehicleState(
        x=float(x),
        y=float(y),
        v=float(v),
        yaw=float(yaw),
        yaw_rate=float(yaw_rate),
        a=float(a),
        t=frenet_state.t,
    )
