function [v_target, w_target, steer_cmd, accel_cmd, risk_idx] = block_emergency_override(raw_cmd, obstacles, ego_bus)
%#codegen
% Emergency Braking & Actuation Command Generator

L = 2.498; % Nexon EV Wheelbase
v_raw = raw_cmd(1);
w_raw = raw_cmd(2);

% Check critical forward proximity (TTC < 1.0s or distance < 4.0m)
emergency_stop = false;
max_risk = 0.0;
N_obs = size(obstacles, 1);
has_risk_col = size(obstacles, 2) >= 6;
for k = 1:min(5, N_obs)
    d = hypot(obstacles(k, 1) - ego_bus.x, obstacles(k, 2) - ego_bus.y);
    if has_risk_col && obstacles(k, 6) > max_risk
        max_risk = obstacles(k, 6);
    end
    if d < 4.0 && obstacles(k, 1) > ego_bus.x && abs(obstacles(k, 2) - ego_bus.y) < 1.5
        emergency_stop = true;
    end
end

if emergency_stop || max_risk >= 0.90
    v_target = 0.0;
    w_target = 0.0;
    accel_cmd = -6.5; % Max deceleration braking
    steer_cmd = 0.0;
else
    v_target = v_raw;
    w_target = w_raw;
    accel_cmd = (v_target - ego_bus.v) / 0.1;
    accel_cmd = max(-6.5, min(3.5, accel_cmd));
    % Kinematic steering angle conversion: delta = atan(w * L / v)
    if abs(v_target) > 0.5
        steer_cmd = atan(w_target * L / v_target);
    else
        steer_cmd = 0.0;
    end
    steer_cmd = max(-0.55, min(0.55, steer_cmd));
end

risk_idx = max_risk;
end
