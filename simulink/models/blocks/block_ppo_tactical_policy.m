function [tactical_nudge, speed_factor, risk_weight] = block_ppo_tactical_policy(ego_bus, env_bus, obstacles)
%#codegen
% PPO Tactical Negotiation Policy Block
% Inputs:
%   ego_bus: BusVehicleState [x, y, v, yaw, yaw_rate, accel, steer]
%   env_bus: BusRiskContext [RoadWidth, FrictionMu, Visibility, TrafficDensity]
%   obstacles: 5x6 array [x, y, vx, vy, class_id, risk_score]
% Outputs:
%   tactical_nudge: Lateral shift (-1.5m to +1.5m)
%   speed_factor: Velocity scaling factor (0.25 to 1.15)
%   risk_weight: DWA risk aversion penalty (0.5 to 2.5)

persistent W1 W2 W_head
if isempty(W1)
    W1 = single(zeros(64, 12));
    W2 = single(zeros(64, 64));
    W_head = single(zeros(3, 64));
    % Deterministic weights initialization for codegen
    for i = 1:64
        for j = 1:12
            W1(i, j) = single(0.05 * sin(double(i*13 + j*7)));
        end
        for j = 1:64
            W2(i, j) = single(0.04 * cos(double(i*11 + j*5)));
        end
    end
    for i = 1:3
        for j = 1:64
            W_head(i, j) = single(0.06 * sin(double(i*17 + j*3)));
        end
    end
end

% Find maximum risk obstacle
max_risk = 0.0;
obs_dist = 60.0;
obs_vrel = 0.0;
obs_lat = 0.0;
N_obs = size(obstacles, 1);
has_cols = size(obstacles, 2) >= 6;
for k = 1:min(5, N_obs)
    if has_cols && obstacles(k, 6) > max_risk
        max_risk = obstacles(k, 6);
        obs_dist = hypot(obstacles(k, 1) - ego_bus.x, obstacles(k, 2) - ego_bus.y);
        obs_vrel = hypot(obstacles(k, 3), obstacles(k, 4));
        obs_lat = obstacles(k, 2);
    end
end

% Normalize 12-element observation
obs = single(zeros(12, 1));
obs(1) = single(min(1.0, ego_bus.v / 22.22));
obs(2) = single(min(1.0, ego_bus.y / 3.5));
obs(3) = single(min(1.0, env_bus.RoadWidth / 14.0));
obs(4) = single(env_bus.FrictionMu / 0.85);
obs(5) = single(min(1.0, env_bus.TrafficDensity / 5.0));
obs(6) = single(min(1.0, obs_dist / 60.0));
obs(7) = single(min(1.0, obs_vrel / 20.0));
obs(8) = single(max_risk);
obs(9) = single(min(1.0, abs(obs_lat) / 3.5));
obs(10) = single(ego_bus.yaw);
obs(11) = single(ego_bus.yaw_rate);
obs(12) = single(0.0);

% Forward MLP
z1 = max(single(0.0), W1 * obs);
z2 = tanh(W2 * z1);
raw = W_head * z2;

tactical_nudge = double(1.50 * tanh(raw(1)));
speed_factor   = double(0.70 + 0.45 * tanh(raw(2)));
risk_weight    = double(1.40 + 1.10 * tanh(raw(3)));
end
