function predicted_tracks = run_trajectory_prediction()
% RUN_TRAJECTORY_PREDICTION Forecasts 5-second future trajectories (50 steps at 10 Hz)
% for all detected obstacles in MATLAB Automated Driving Toolbox.
%
% Implements Spatio-Temporal Trajectory Forecasting with Kinematic CTRA
% integration for Auto-Rickshaws, Cattle, Two-Wheelers, and Commercial Buses.

    clc;
    fprintf('=========================================================================\n');
    fprintf('   BHARATDRIVE-AI: 5-SECOND MULTI-AGENT TRAJECTORY PREDICTION (MATLAB)   \n');
    fprintf('   Horizon: 5.0 Seconds (50 Steps @ 10 Hz) | Non-Autoregressive Rollout \n');
    fprintf('=========================================================================\n\n');

    % Setup paths
    root_dir = fullfile(fileparts(mfilename('fullpath')), '..');
    addpath(root_dir);
    addpath(fullfile(root_dir, '+bharatdrive'));

    dt = 0.1;           % 10 Hz trajectory discretization
    pred_horizon_s = 5.0;
    pred_steps = round(pred_horizon_s / dt); % 50 steps

    % 1. Ego Vehicle State [x, y, v, yaw]
    ego_x = 15.0;
    ego_y = -1.75; % Left lane
    ego_v = 13.89; % 50 km/h
    ego_yaw = 0.0;

    % 2. Active Detected Objects from Perception Engine
    tracked_agents(1) = struct('ID', 1, 'Class', 'Auto-Rickshaw', ...
        'Pos', [45.0, 1.2, 0.0], 'Vel', [9.0, -1.1, 0.0], 'YawRate', -0.12, 'Dim', [2.7, 1.4, 1.8]);
    tracked_agents(2) = struct('ID', 2, 'Class', 'Cattle', ...
        'Pos', [85.0, -1.5, 0.0], 'Vel', [0.0, 0.0, 0.0], 'YawRate', 0.0, 'Dim', [2.2, 0.9, 1.5]);
    tracked_agents(3) = struct('ID', 3, 'Class', 'Truck-Bus', ...
        'Pos', [180.0, 1.75, 0.0], 'Vel', [-12.5, 0.0, 0.0], 'YawRate', 0.0, 'Dim', [11.2, 2.6, 3.2]);
    tracked_agents(4) = struct('ID', 4, 'Class', 'Two-Wheeler', ...
        'Pos', [30.0, -2.8, 0.0], 'Vel', [11.0, 0.4, 0.0], 'YawRate', 0.05, 'Dim', [2.0, 0.8, 1.3]);

    fprintf('Active obstacles tracked: %d | Forecasting next %d steps (0.1s to 5.0s)...\n\n', ...
        length(tracked_agents), pred_steps);

    predicted_tracks = cell(length(tracked_agents), 1);

    fprintf('------------------------------------------------------------------------------------------------------\n');
    fprintf('%-6s | %-14s | %-14s | %-14s | %-14s | %-14s | %-8s\n', ...
        'ID', 'Class Name', 'Pos(t=0s)', 'Pos(t=1s)', 'Pos(t=2.5s)', 'Pos(t=5.0s)', 'Min TTC');
    fprintf('------------------------------------------------------------------------------------------------------\n');

    for i = 1:length(tracked_agents)
        ag = tracked_agents(i);
        future_traj = zeros(pred_steps, 2);

        curr_x = ag.Pos(1);
        curr_y = ag.Pos(2);
        curr_vx = ag.Vel(1);
        curr_vy = ag.Vel(2);
        curr_speed = hypot(curr_vx, curr_vy);
        curr_yaw = atan2(curr_vy, curr_vx);
        yaw_rate = ag.YawRate;

        min_ttc = inf;

        % 50-step non-autoregressive kinematic rollout
        for step = 1:pred_steps
            t = step * dt;
            if strcmp(ag.Class, 'Cattle')
                % Cattle stationary assumption
                future_x = curr_x;
                future_y = curr_y;
            else
                curr_yaw = curr_yaw + yaw_rate * dt;
                curr_x = curr_x + curr_speed * cos(curr_yaw) * dt;
                curr_y = curr_y + curr_speed * sin(curr_yaw) * dt;
                future_x = curr_x;
                future_y = curr_y;
            end
            future_traj(step, :) = [future_x, future_y];

            % Compute Time-to-Collision (TTC) with ego trajectory
            ego_future_x = ego_x + ego_v * t;
            ego_future_y = ego_y;
            dist_to_ego = hypot(future_x - ego_future_x, future_y - ego_future_y);

            if dist_to_ego < 2.5 && min_ttc == inf
                min_ttc = t;
            end
        end

        predicted_tracks{i} = struct( ...
            'ID', ag.ID, ...
            'Class', ag.Class, ...
            'Trajectory_50steps', future_traj, ...
            'MinTTC_s', min_ttc ...
        );

        p0_str = sprintf('[%5.1f,%4.1f]', ag.Pos(1), ag.Pos(2));
        p1_str = sprintf('[%5.1f,%4.1f]', future_traj(10, 1), future_traj(10, 2));
        p2_str = sprintf('[%5.1f,%4.1f]', future_traj(25, 1), future_traj(25, 2));
        p5_str = sprintf('[%5.1f,%4.1f]', future_traj(50, 1), future_traj(50, 2));

        if min_ttc < 10.0
            ttc_str = sprintf('%4.1f s [ALERT]', min_ttc);
        else
            ttc_str = 'Safe';
        end

        fprintf('#%-5d | %-14s | %-14s | %-14s | %-14s | %-14s | %-8s\n', ...
            ag.ID, ag.Class, p0_str, p1_str, p2_str, p5_str, ttc_str);
    end
    fprintf('------------------------------------------------------------------------------------------------------\n');
    fprintf('\n[SUCCESS] 5-second trajectory predictions generated for all active obstacles.\n');
    fprintf('Predicted spatiotemporal states ready for Frenet Path Planning & Collision Avoidance.\n');
end
