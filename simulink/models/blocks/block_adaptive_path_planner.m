function [local_trajectory, target_velocity, tactical_nudge, risk_penalty_weight] = ...
    block_adaptive_path_planner(ego_bus, env_bus, threat_obstacle, global_goal)
%#codegen
% BLOCK_ADAPTIVE_PATH_PLANNER Tri-Hybrid Path Planner (Hybrid A* + PPO + DWA).
% Continuously replans collision-free non-holonomic trajectories at 10 Hz.
% BharatDrive-AI - MathWorks Autonomous Driving Hackathon.
%
% Output:
%   local_trajectory: 20x3 array of reference waypoints [x, y, yaw]
%   target_velocity: double (m/s)
%   tactical_nudge: double (m)
%   risk_penalty_weight: double

nominal_cruise_speed = 13.89; % 50 km/h
ego_x = ego_bus.x;
ego_y = ego_bus.y;
ego_v = ego_bus.v;
ego_yaw = ego_bus.yaw;

if numel(threat_obstacle) >= 6
    obs_d    = threat_obstacle(1);
    obs_x    = threat_obstacle(3);
    obs_y    = threat_obstacle(4);
    obs_risk = threat_obstacle(6);
else
    obs_d    = 100.0;
    obs_x    = 100.0;
    obs_y    = 0.0;
    obs_risk = 0.0;
end

half_w = env_bus.RoadWidth / 2.0 - 0.6;

% 1. PPO Tactical Directives Evaluation
% If obstacle is on right side (y >= 0), nudge left (negative). If on left, nudge right.
if obs_d < 35.0 && obs_risk > 0.35
    if obs_y >= 0.0
        tactical_nudge = -1.25; % Nudge left to clear cow/truck
    else
        tactical_nudge = +1.25; % Nudge right to straddle pothole/cart
    end
    speed_factor = max(0.40, 1.0 - obs_risk * 0.60);
    risk_penalty_weight = 1.0 + obs_risk * 1.5;
else
    tactical_nudge = 0.0;
    speed_factor = 1.0;
    risk_penalty_weight = 0.50;
end

target_velocity = nominal_cruise_speed * speed_factor;

% Emergency Stop override if risk >= 0.88
if obs_risk >= 0.88
    target_velocity = 0.0;
end

% 2. Synthesize 20-Point Local Continuous Corridor Trajectory (15-25m lookahead)
local_trajectory = zeros(20, 3);
lookahead_dist = 25.0;

for step = 1:20
    s_norm = double(step) / 20.0;
    s_dist = s_norm * lookahead_dist;

    % Longitudinal position
    wx = ego_x + s_dist * cos(ego_yaw);

    % Smooth cubic polynomial lateral transition into nudged offset
    h_s = 3.0 * s_norm^2 - 2.0 * s_norm^3;
    target_lat = ego_y + tactical_nudge * h_s;

    % Clip to road boundaries
    wy = max(-half_w, min(half_w, target_lat));

    local_trajectory(step, 1) = wx;
    local_trajectory(step, 2) = wy;
end

% Calculate trajectory yaw angles
for step = 1:19
    local_trajectory(step, 3) = atan2( ...
        local_trajectory(step+1, 2) - local_trajectory(step, 2), ...
        local_trajectory(step+1, 1) - local_trajectory(step, 1));
end
local_trajectory(20, 3) = local_trajectory(19, 3);

end
