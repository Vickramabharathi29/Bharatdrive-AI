function raw_cmd = block_dwa_replanner(ego_bus, local_target, speed_factor, risk_weight, obstacles)
%#codegen
% Dynamic Window Approach Local Continuous Replanner Block
% Returns raw_cmd: [target_velocity, target_yaw_rate]

max_v = min(22.22, 13.89 * speed_factor);
max_accel = 3.5;
max_decel = 6.5;
dt = 0.1;

v_min = max(0.0, ego_bus.v - max_decel * dt);
v_max = min(max_v, ego_bus.v + max_accel * dt);

w_min = -0.85;
w_max = 0.85;

best_score = -1e9;
best_v = 0.0;
best_w = 0.0;

% Velocity candidate search (7x9 grid for fast real-time codegen execution)
for v_step = 0:6
    v_cand = v_min + (v_max - v_min) * (v_step / 6.0);
    for w_step = 0:8
        w_cand = w_min + (w_max - w_min) * (w_step / 8.0);
        
        % Rollout 2.0s
        pred_yaw = ego_bus.yaw + w_cand * 2.0;
        pred_x   = ego_bus.x + v_cand * cos(pred_yaw) * 2.0;
        pred_y   = ego_bus.y + v_cand * sin(pred_yaw) * 2.0;
        
        % Heading score
        goal_yaw = atan2(local_target(2) - pred_y, local_target(1) - pred_x);
        heading_diff = abs(atan2(sin(goal_yaw - pred_yaw), cos(goal_yaw - pred_yaw)));
        score_h = (pi - heading_diff) / pi;
        
        % Clearance score
        min_d = 100.0;
        N_obs = size(obstacles, 1);
        for k = 1:min(5, N_obs)
            d = hypot(pred_x - obstacles(k, 1), pred_y - obstacles(k, 2));
            if d < min_d
                min_d = d;
            end
        end
        score_c = min(1.0, min_d / 8.0);
        
        % Admissibility
        stop_dist = (v_cand^2) / (2.0 * max_decel);
        if min_d <= stop_dist && v_cand > 1.0
            continue;
        end
        
        score_v = v_cand / 22.22;
        score_risk = min(1.0, 1.0 / (min_d + 0.1));
        
        score = 0.20 * score_h + 0.40 * score_c + 0.25 * score_v - (risk_weight * 0.3) * score_risk;
        if score > best_score
            best_score = score;
            best_v = v_cand;
            best_w = w_cand;
        end
    end
end

raw_cmd = [best_v; best_w];
end
