function local_target = block_hybrid_astar(ego_bus, target_goal, tactical_nudge, obstacles, env_bus)
%#codegen
% Kinematic Hybrid A* Corridor Planner Block
% Computes local collision-free target waypoint [x_t, y_t] 15m ahead.

lookahead = 15.0;
x_target = ego_bus.x + lookahead * cos(ego_bus.yaw);
y_target = ego_bus.y + lookahead * sin(ego_bus.yaw) + tactical_nudge;

% Road boundary clipping
half_w = env_bus.RoadWidth / 2.0 - 0.7;
if y_target > half_w
    y_target = half_w;
elseif y_target < -half_w
    y_target = -half_w;
end

% Obstacle geometric nudge
N_obs = size(obstacles, 1);
for k = 1:min(5, N_obs)
    ox = obstacles(k, 1);
    oy = obstacles(k, 2);
    d = hypot(x_target - ox, y_target - oy);
    if d < 2.5 && ox > ego_bus.x
        if y_target >= oy
            y_target = oy + 2.5;
        else
            y_target = oy - 2.5;
        end
    end
end

% Ensure avoidance nudge does not breach outer road boundary
y_target = max(-half_w, min(half_w, y_target));

local_target = [x_target; y_target];
end
