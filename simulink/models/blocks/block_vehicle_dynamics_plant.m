function ego_bus = block_vehicle_dynamics_plant(control_bus, env_bus)
%#codegen
% BLOCK_VEHICLE_DYNAMICS_PLANT 2-DOF Non-Linear Dynamic Bicycle Model with Pacejka Tires.
% Models Tata Nexon EV with lateral tire slip, steering rate limits, and road friction.
% BharatDrive-AI - MathWorks Autonomous Driving Hackathon.
%
% Output:
%   ego_bus: BusVehicleState [x, y, v, yaw, yaw_rate, accel, steer, lateral_slip]

% Vehicle Parameters: Tata Nexon EV
m  = 1400.0;     % Curb mass (kg)
Iz = 2250.0;     % Yaw moment of inertia (kg*m^2)
lf = 1.150;      % CG to front axle (m)
lr = 1.348;      % CG to rear axle (m)
L  = lf + lr;    % Wheelbase = 2.498m
Cf = 80000.0;    % Front cornering stiffness (N/rad)
Cr = 85000.0;    % Rear cornering stiffness (N/rad)
g  = 9.81;
dt = 0.01;       % 100 Hz simulation step

persistent state
if isempty(state)
    % Initial state: [x, y, vx, vy, yaw, omega, steer]
    state = [0.0; 0.0; 8.0; 0.0; 0.0; 0.0; 0.0];
end

mu = env_bus.FrictionMu;

% Actuator Commands
a_cmd = control_bus.AccelCommand;
steer_cmd = control_bus.SteerCommand;

% Steering Rate Limiting (max 0.60 rad/s)
max_steer_delta = 0.60 * dt;
steer_diff = steer_cmd - state(7);
actual_steer = state(7) + max(-max_steer_delta, min(max_steer_delta, steer_diff));
actual_steer = max(-0.55, min(0.55, actual_steer));

vx = max(0.2, state(3));
vy = state(4);
yaw = state(5);
omega = state(6);

% 1. Tire Lateral Slip Angles
alpha_f = actual_steer - atan2(vy + lf * omega, vx);
alpha_r = -atan2(vy - lr * omega, vx);

% 2. Normal Tire Load Distribution & Saturated Lateral Forces
Fz_f = (m * g * lr) / L;
Fz_r = (m * g * lf) / L;
Fy_max_f = mu * Fz_f;
Fy_max_r = mu * Fz_r;

Fy_f = max(-Fy_max_f, min(Fy_max_f, 2.0 * Cf * alpha_f));
Fy_r = max(-Fy_max_r, min(Fy_max_r, 2.0 * Cr * alpha_r));

% 3. Equations of Motion (2-DOF Vehicle Dynamic Model)
% Longitudinal acceleration
dvx = a_cmd + vy * omega;
% Lateral acceleration
dvy = (Fy_f * cos(actual_steer) + Fy_r) / m - vx * omega;
% Yaw angular acceleration
domega = (lf * Fy_f * cos(actual_steer) - lr * Fy_r) / Iz;

% 4. State Integration (Forward Euler at 100 Hz)
state(1) = state(1) + (vx * cos(yaw) - vy * sin(yaw)) * dt; % x
state(2) = state(2) + (vx * sin(yaw) + vy * cos(yaw)) * dt; % y
state(3) = max(0.0, state(3) + dvx * dt);                    % vx
state(4) = state(4) + dvy * dt;                             % vy
state(5) = state(5) + omega * dt;                           % yaw
state(6) = state(6) + domega * dt;                          % omega
state(7) = actual_steer;                                    % steer

% Output Bus Construction
ego_bus.x            = state(1);
ego_bus.y            = state(2);
ego_bus.v            = state(3);
ego_bus.yaw          = state(5);
ego_bus.yaw_rate     = state(6);
ego_bus.accel        = a_cmd;
ego_bus.steer        = state(7);
ego_bus.lateral_slip = alpha_f;

end
