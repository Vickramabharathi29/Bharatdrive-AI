function predicted_trajectories = block_trajectory_prediction(fused_tracks, env_bus)
%#codegen
% BLOCK_TRAJECTORY_PREDICTION 5-Second Multi-Agent Trajectory Predictor.
% Evaluates non-autoregressive motion forecasting conditioned on road boundaries.
% BharatDrive-AI - MathWorks Autonomous Driving Hackathon.
%
% Output:
%   predicted_trajectories: 5x10 array
%     Each row: [x(1s), y(1s), x(2s), y(2s), x(3s), y(3s), x(4s), y(4s), x(5s), y(5s)]

predicted_trajectories = zeros(5, 10);
half_w = env_bus.RoadWidth / 2.0 - 0.4;

for i = 1:5
    x0 = fused_tracks(i, 1);
    y0 = fused_tracks(i, 2);
    vx = fused_tracks(i, 3);
    vy = fused_tracks(i, 4);
    class_id = int32(fused_tracks(i, 5));

    if fused_tracks(i, 7) < 0.1
        continue; % Inactive track
    end

    % Class-conditioned dynamics behavior
    % Cattle (class 7) and Potholes (class 11) are predominantly stationary or wandering
    if class_id == 7 || class_id == 11
        vx = 0.0;
        vy = 0.0;
    elseif class_id == 5 % Auto-rickshaw cuts laterally
        vy = vy + 0.10;
    end

    for step = 1:5
        t = double(step) * 1.0; % 1s, 2s, 3s, 4s, 5s horizons
        pred_x = x0 + vx * t;
        pred_y = y0 + vy * t;

        % Bound within drivable road envelope
        pred_y = max(-half_w, min(half_w, pred_y));

        col_x = (step - 1) * 2 + 1;
        col_y = (step - 1) * 2 + 2;
        predicted_trajectories(i, col_x) = pred_x;
        predicted_trajectories(i, col_y) = pred_y;
    end
end

end
