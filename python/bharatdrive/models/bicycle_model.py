"""Vehicle kinematic and dynamic bicycle models.

Implements:
1. KinematicBicycleModel: Rear-axle parameterized non-holonomic kinematic model with RK4 integration.
2. DynamicBicycleModel: 2-DOF vehicle dynamics model with front/rear lateral tire slip and Pacejka/linear tire force models.
"""

from __future__ import annotations
import math
import numpy as np
from typing import Tuple
from bharatdrive.config import VehicleConfig
from bharatdrive.models.state import VehicleState


class KinematicBicycleModel:
    """Kinematic Bicycle Model parameterized at the center of the rear axle.
    
    Differential Equations:
        dx/dt = v * cos(yaw)
        dy/dt = v * sin(yaw)
        dyaw/dt = (v / L) * tan(steer)
        dv/dt = a
    """

    def __init__(self, config: VehicleConfig | None = None):
        self.config = config or VehicleConfig()
        self.L = self.config.dimensions.wheelbase
        self.max_steer = self.config.limits.max_steer_angle_rad
        self.max_steer_rate = self.config.limits.max_steer_rate_rad_s
        self.max_v = self.config.limits.max_velocity_mps
        self.min_v = self.config.limits.min_velocity_mps
        self.max_accel = self.config.limits.max_accel_mps2
        self.max_decel = self.config.limits.max_decel_mps2

    def _derivatives(self, state_vec: np.ndarray, a: float, steer: float) -> np.ndarray:
        """Computes [dx, dy, dv, dyaw] state derivatives."""
        _, _, v, yaw = state_vec[:4]
        dx = v * math.cos(yaw)
        dy = v * math.sin(yaw)
        dv = a
        dyaw = (v / self.L) * math.tan(steer)
        return np.array([dx, dy, dv, dyaw], dtype=np.float64)

    def step(
        self,
        current_state: VehicleState,
        accel_cmd: float,
        steer_cmd: float,
        dt: float,
        method: str = "rk4",
    ) -> VehicleState:
        """Propagates vehicle state forward in time by dt seconds.
        
        Args:
            current_state: Current vehicle state.
            accel_cmd: Commanded longitudinal acceleration [m/s^2].
            steer_cmd: Commanded front steering angle [rad].
            dt: Time step duration [s].
            method: 'euler' or 'rk4' (Runge-Kutta 4th Order).
            
        Returns:
            VehicleState: Propagated vehicle state.
        """
        # Clamp inputs within vehicle hardware limits
        a_applied = np.clip(accel_cmd, self.max_decel, self.max_accel)
        steer_target = np.clip(steer_cmd, -self.max_steer, self.max_steer)

        # Rate-limit steering
        steer_diff = steer_target - current_state.steer
        steer_rate_limit = self.max_steer_rate * dt
        actual_steer = current_state.steer + np.clip(steer_diff, -steer_rate_limit, steer_rate_limit)

        s_vec = np.array([current_state.x, current_state.y, current_state.v, current_state.yaw], dtype=np.float64)

        if method.lower() == "rk4":
            k1 = self._derivatives(s_vec, a_applied, actual_steer)
            k2 = self._derivatives(s_vec + 0.5 * dt * k1, a_applied, actual_steer)
            k3 = self._derivatives(s_vec + 0.5 * dt * k2, a_applied, actual_steer)
            k4 = self._derivatives(s_vec + dt * k3, a_applied, actual_steer)
            s_next = s_vec + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        else:  # Euler integration
            s_next = s_vec + dt * self._derivatives(s_vec, a_applied, actual_steer)

        next_x, next_y, next_v, next_yaw = s_next
        next_v = float(np.clip(next_v, self.min_v, self.max_v))
        yaw_rate = float((next_v / self.L) * math.tan(actual_steer))

        return VehicleState(
            x=float(next_x),
            y=float(next_y),
            v=next_v,
            yaw=float(next_yaw),
            yaw_rate=yaw_rate,
            a=float(a_applied),
            steer=float(actual_steer),
            v_y=0.0,
            t=current_state.t + dt,
        )


class DynamicBicycleModel:
    """2-DOF Dynamic Bicycle Model with tire slip dynamics parameterized at vehicle CG.
    
    Includes low-speed kinematic blending to prevent division-by-zero singularities.
    """

    def __init__(self, config: VehicleConfig | None = None):
        self.config = config or VehicleConfig()
        self.m = self.config.mass.mass
        self.Iz = self.config.mass.inertia_z
        self.lf = self.config.mass.cg_to_front_axle
        self.lr = self.config.mass.cg_to_rear_axle
        self.L = self.lf + self.lr
        self.Cf = self.config.tires.cornering_stiffness_front
        self.Cr = self.config.tires.cornering_stiffness_rear
        self.mu = self.config.tires.friction_coefficient
        self.g = 9.81
        self.low_speed_threshold = 1.0  # m/s

        # Understeer Gradient: Kus = (m / L) * (lr / (2*Cf) - lf / (2*Cr))
        self.Kus = (self.m / self.L) * ((self.lr / (2.0 * self.Cf)) - (self.lf / (2.0 * self.Cr)))

    def compute_slip_angles(self, vx: float, vy: float, omega: float, delta: float) -> Tuple[float, float]:
        """Calculates front and rear tire lateral slip angles [rad]."""
        if abs(vx) < 0.1:
            return 0.0, 0.0
        alpha_f = delta - math.atan2(vy + self.lf * omega, vx)
        alpha_r = -math.atan2(vy - self.lr * omega, vx)
        return float(alpha_f), float(alpha_r)

    def compute_tire_forces(self, alpha_f: float, alpha_r: float) -> Tuple[float, float]:
        """Computes lateral tire forces with hyperbolic tangent saturation approximation."""
        # Normal loads on front and rear axles
        Fz_f = (self.m * self.g * self.lr) / self.L
        Fz_r = (self.m * self.g * self.lf) / self.L

        # Max lateral friction capacity
        Fy_max_f = self.mu * Fz_f
        Fy_max_r = self.mu * Fz_r

        # Linear tire force with saturation capping
        Fy_f = float(np.clip(2.0 * self.Cf * alpha_f, -Fy_max_f, Fy_max_f))
        Fy_r = float(np.clip(2.0 * self.Cr * alpha_r, -Fy_max_r, Fy_max_r))
        return Fy_f, Fy_r

    def step(
        self,
        current_state: VehicleState,
        accel_cmd: float,
        steer_cmd: float,
        dt: float,
    ) -> VehicleState:
        """Integrates dynamic equations of motion."""
        vx = max(current_state.v, 0.01)
        vy = current_state.v_y
        omega = current_state.yaw_rate
        steer = np.clip(steer_cmd, -self.config.limits.max_steer_angle_rad, self.config.limits.max_steer_angle_rad)
        ax = np.clip(accel_cmd, self.config.limits.max_decel_mps2, self.config.limits.max_accel_mps2)

        # Low-speed fallback to kinematic model
        if vx < self.low_speed_threshold:
            kinematic_fallback = KinematicBicycleModel(self.config)
            return kinematic_fallback.step(current_state, ax, steer, dt, method="rk4")

        alpha_f, alpha_r = self.compute_slip_angles(vx, vy, omega, steer)
        Fy_f, Fy_r = self.compute_tire_forces(alpha_f, alpha_r)

        # Equations of motion in vehicle body frame
        d_vx = ax + vy * omega
        d_vy = (Fy_f * math.cos(steer) + Fy_r) / self.m - vx * omega
        d_omega = (self.lf * Fy_f * math.cos(steer) - self.lr * Fy_r) / self.Iz

        # Inertial frame position derivatives
        cos_yaw = math.cos(current_state.yaw)
        sin_yaw = math.sin(current_state.yaw)
        dx = vx * cos_yaw - vy * sin_yaw
        dy = vx * sin_yaw + vy * cos_yaw

        # Forward Euler update
        next_vx = float(np.clip(vx + d_vx * dt, self.config.limits.min_velocity_mps, self.config.limits.max_velocity_mps))
        next_vy = float(vy + d_vy * dt)
        next_omega = float(omega + d_omega * dt)
        next_yaw = float(current_state.yaw + omega * dt)
        next_x = float(current_state.x + dx * dt)
        next_y = float(current_state.y + dy * dt)

        return VehicleState(
            x=next_x,
            y=next_y,
            v=next_vx,
            yaw=next_yaw,
            yaw_rate=next_omega,
            a=float(ax),
            steer=float(steer),
            v_y=next_vy,
            t=current_state.t + dt,
        )
