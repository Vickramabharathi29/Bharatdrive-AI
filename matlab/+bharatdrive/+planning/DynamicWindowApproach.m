classdef DynamicWindowApproach < handle
    % DYNAMICWINDOWAPPROACH Local trajectory sampler operating in velocity space (v, omega).
    % Evaluates kinematically reachable trajectories over dynamic acceleration limits.

    properties
        MaxSpeed        % Maximum forward speed (m/s) [e.g. 22.22 m/s]
        MinSpeed        % Minimum forward speed (m/s) [0.0 m/s]
        MaxYawRate      % Maximum yaw rate (rad/s) [e.g. 0.8 rad/s]
        MaxAccel        % Max longitudinal acceleration (m/s^2) [3.5]
        MaxDecel        % Emergency braking deceleration (m/s^2) [6.5]
        MaxYawAccel     % Maximum angular acceleration (rad/s^2) [1.5]
        PredictTime     % Forward rollout horizon (s) [default 2.0s]
        Dt              % Trajectory step size (s) [default 0.1s]

        % Discretization resolution
        VSamples        % Number of velocity candidates [default 15]
        OmegaSamples    % Number of yaw rate candidates [default 21]

        % Objective weights (dynamically modulated by PPO agent)
        WeightHeading   % Alignment with goal heading
        WeightClearance % Distance to obstacles / road bounds
        WeightVelocity  % Forward progress
        WeightRiskCost  % Penalty from dynamic costmap
    end

    methods
        function obj = DynamicWindowApproach(max_speed, max_accel, max_decel)
            if nargin < 3, max_decel = 6.5; end
            if nargin < 2, max_accel = 3.5; end
            if nargin < 1, max_speed = 22.22; end

            obj.MaxSpeed = max_speed;
            obj.MinSpeed = 0.0;
            obj.MaxYawRate = 0.85;
            obj.MaxAccel = max_accel;
            obj.MaxDecel = max_decel;
            obj.MaxYawAccel = 1.5;
            obj.PredictTime = 2.0;
            obj.Dt = 0.1;

            obj.VSamples = 15;
            obj.OmegaSamples = 21;

            obj.WeightHeading = 0.15;
            obj.WeightClearance = 0.40;
            obj.WeightVelocity = 0.25;
            obj.WeightRiskCost = 0.50;
        end

        function [best_cmd, best_traj] = plan(obj, current_state, target_goal, obstacle_list, costmap_engine)
            % PLAN Selects optimal [v, omega] maximizing DWA objective.
            % current_state: [x, y, v, yaw, yaw_rate]
            % target_goal:   [x_goal, y_goal]

            if nargin < 5, costmap_engine = []; end
            if nargin < 4, obstacle_list = []; end

            curr_x = current_state(1);
            curr_y = current_state(2);
            curr_v = current_state(3);
            curr_yaw = current_state(4);
            curr_omega = current_state(5);

            % 1. Dynamic Window: achievable velocities within dt
            v_min_dyn = max(obj.MinSpeed, curr_v - obj.MaxDecel * obj.Dt);
            v_max_dyn = min(obj.MaxSpeed, curr_v + obj.MaxAccel * obj.Dt);
            w_min_dyn = max(-obj.MaxYawRate, curr_omega - obj.MaxYawAccel * obj.Dt);
            w_max_dyn = min(obj.MaxYawRate, curr_omega + obj.MaxYawAccel * obj.Dt);

            v_candidates = linspace(v_min_dyn, v_max_dyn, obj.VSamples);
            w_candidates = linspace(w_min_dyn, w_max_dyn, obj.OmegaSamples);

            best_score = -inf;
            best_cmd = [0.0, 0.0]; % [v, omega]
            num_steps = round(obj.PredictTime / obj.Dt);
            best_traj = repmat([curr_x, curr_y, curr_yaw], num_steps, 1);

            for v = v_candidates
                for w = w_candidates
                    % Simulate forward trajectory arc
                    traj = zeros(num_steps, 3);
                    sim_x = curr_x;
                    sim_y = curr_y;
                    sim_yaw = curr_yaw;
                    min_dist_to_obs = inf;

                    for step = 1:num_steps
                        sim_yaw = sim_yaw + w * obj.Dt;
                        sim_x = sim_x + v * cos(sim_yaw) * obj.Dt;
                        sim_y = sim_y + v * sin(sim_yaw) * obj.Dt;
                        traj(step, :) = [sim_x, sim_y, sim_yaw];

                        % Check obstacle clearances
                        for ob_idx = 1:length(obstacle_list)
                            ob = obstacle_list(ob_idx);
                            dist = hypot(sim_x - ob.Position(1), sim_y - ob.Position(2));
                            if dist < min_dist_to_obs
                                min_dist_to_obs = dist;
                            end
                        end
                    end

                    % Admissibility check: can stop before collision?
                    stopping_dist = (v^2) / (2.0 * obj.MaxDecel);
                    if min_dist_to_obs <= stopping_dist && v > 1.0
                        continue; % Inadmissible trajectory
                    end

                    % 1. Heading alignment objective
                    final_pt = traj(end, 1:2);
                    goal_heading = atan2(target_goal(2) - final_pt(2), target_goal(1) - final_pt(1));
                    heading_err = abs(atan2(sin(goal_heading - traj(end, 3)), cos(goal_heading - traj(end, 3))));
                    score_heading = (pi - heading_err) / pi;

                    % 2. Clearance objective (normalized up to 10m)
                    score_clearance = min(1.0, min_dist_to_obs / 8.0);

                    % 3. Velocity progress objective
                    score_velocity = v / max(1.0, obj.MaxSpeed);

                    % 4. Risk Cost penalty
                    score_risk = 0.0;
                    if ~isempty(costmap_engine)
                        % Query costmap along trajectory
                        score_risk = min(1.0, 1.0 / (min_dist_to_obs + 0.1));
                    end

                    % Total weighted score
                    total_score = obj.WeightHeading * score_heading + ...
                                  obj.WeightClearance * score_clearance + ...
                                  obj.WeightVelocity * score_velocity - ...
                                  obj.WeightRiskCost * score_risk;

                    if total_score > best_score
                        best_score = total_score;
                        best_cmd = [v, w];
                        best_traj = traj;
                    end
                end
            end
        end
    end
end
