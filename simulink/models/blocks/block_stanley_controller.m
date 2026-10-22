function [steer_cmd, accel_cmd, brake_pressure, ey, etheta] = ...
    block_stanley_controller(ego_bus, planner_bus, path_points)
%#codegen
% BLOCK_STANLEY_CONTROLLER Embedded Coder-compliant Stanley Motion Controller.
% Evaluates Front-Axle Stanley steering and Anti-Windup Longitudinal Acceleration.
% BharatDrive-AI - MathWorks Autonomous Driving Hackathon.

L = 2.498;           % Tata Nexon EV wheelbase (m)
max_steer = 0.55;    % ~31.5 deg max steer angle
k_gain = 2.20;       % Cross-track gain (fast convergence without overshoot)
k_soft = 1.0;        % Softening factor (m/s)
k_damp = 0.15;       % High-speed yaw-rate damping
dt = 0.01;           % 100 Hz sampling step

persistent integral_v prev_err_v
if isempty(integral_v)
    integral_v = 0.0;
    prev_err_v = 0.0;
end

xr = ego_bus.x;
yr = ego_bus.y;
v  = max(0.1, ego_bus.v);
yaw = ego_bus.yaw;
yaw_rate = ego_bus.yaw_rate;

target_v = planner_bus.TargetVelocity;

% 1. Front Axle Coordinates
xf = xr + L * cos(yaw);
yf = yr + L * sin(yaw);

% 2. Find Nearest Waypoint to Front Axle
min_d = 1e6;
nearest_idx = 1;
N_pts = size(path_points, 1);
for i = 1:min(20, N_pts)
    d = hypot(path_points(i, 1) - xf, path_points(i, 2) - yf);
    if d < min_d
        min_d = d;
        nearest_idx = i;
    end
end

target_pt_x = path_points(nearest_idx, 1);
target_pt_y = path_points(nearest_idx, 2);
path_yaw    = path_points(nearest_idx, 3);

% 3. Heading Error: wrap to [-pi, pi]
etheta = atan2(sin(path_yaw - yaw), cos(path_yaw - yaw));

% 4. Front-Axle Signed Cross-Track Error
dx_f = xf - target_pt_x;
dy_f = yf - target_pt_y;
ey = -dx_f * sin(path_yaw) + dy_f * cos(path_yaw);

% 5. Non-linear Cross-Track Term with Soft Velocity Denominator (steers towards path)
delta_e = atan2(-k_gain * ey, v + k_soft);

% 6. Yaw Damping
delta_damp = -k_damp * yaw_rate;

% 7. Commanded Steering Angle
raw_steer = etheta + delta_e + delta_damp;
steer_cmd = max(-max_steer, min(max_steer, raw_steer));

% 8. Longitudinal Speed Tracking PID with Anti-Windup
kp_v = 1.20;
ki_v = 0.15;
kd_v = 0.08;

err_v = target_v - v;
integral_v = integral_v + err_v * dt;
integral_v = max(-10.0, min(10.0, integral_v));
deriv_v = (err_v - prev_err_v) / dt;
prev_err_v = err_v;

raw_accel = kp_v * err_v + ki_v * integral_v + kd_v * deriv_v;
accel_cmd = max(-6.50, min(3.50, raw_accel));

% Brake Pressure Calculation (0 to 100 bar)
if accel_cmd < 0.0
    brake_pressure = min(100.0, abs(accel_cmd) * 15.38); % 6.5 m/s^2 -> 100 bar
else
    brake_pressure = 0.0;
end

end
