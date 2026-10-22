classdef AdaptivePlannerCoordinator < handle
    % ADAPTIVEPLANNERCOORDINATOR Fuses Hybrid A*, DWA, and PPO Reinforcement Learning
    % into a unified, high-frequency continuous replanning architecture (10-20 Hz).

    properties
        HybridAStar         % Global corridor kinematic planner
        DWA                 % Short-term dynamic window sampler
        PPOAgent            % Reinforcement learning tactical negotiator
        LastTrajectory      % Previously executed trajectory for smooth splicing
        ReplanningRateHz    % Replanning loop frequency [10 Hz]
        TargetSpeed         % Nominal cruise speed (m/s) [13.89 m/s = 50 km/h]
    end

    methods
        function obj = AdaptivePlannerCoordinator(target_speed)
            if nargin < 1, target_speed = 13.89; end

            obj.TargetSpeed = target_speed;
            obj.ReplanningRateHz = 10.0; % 100ms replan cycle

            % Initialize Level 1, Level 2, Level 3 planners
            obj.HybridAStar = bharatdrive.planning.HybridAStarPlanner();
            obj.DWA = bharatdrive.planning.DynamicWindowApproach(target_speed);
            obj.PPOAgent = bharatdrive.planning.PPOAgentNegotiator();
            obj.LastTrajectory = [];
        end

        function plan_output = replan_cycle(obj, ego_state, detected_obstacles, risk_results, road_width, weather, global_goal)
            % REPLAN_CYCLE Executes a single continuous replan iteration (10 Hz).
            % ego_state: [x, y, v, yaw, yaw_rate]

            if nargin < 7, global_goal = [ego_state(1) + 60.0, 0.0, 0.0]; end
            if nargin < 6, weather.friction_mu = 0.85; weather.visibility = 1.0; end
            if nargin < 5, road_width = 7.0; end

            % 1. Identify primary threatening obstacle from risk assessments
            highest_risk_obs = [];
            max_risk = 0.0;
            for i = 1:length(risk_results)
                r = risk_results{i};
                if r.RiskScore > max_risk
                    max_risk = r.RiskScore;
                    highest_risk_obs = r;
                end
            end

            % 2. Level 2: Evaluate PPO Tactical Policy
            traffic_density = length(detected_obstacles);
            obs_vec = obj.PPOAgent.extract_observation(ego_state, highest_risk_obs, road_width, weather.friction_mu, traffic_density);
            [ppo_action, state_value] = obj.PPOAgent.evaluate_policy(obs_vec);

            % 3. Level 1: Modulate Mid-Term Goal using PPO Tactical Nudge
            nudged_goal = global_goal;
            nudged_goal(2) = np_clip(global_goal(2) + ppo_action.NudgeOffset, -(road_width/2 - 0.6), (road_width/2 - 0.6));

            % Generate or update kinematic corridor reference via Hybrid A*
            obstacle_boxes = zeros(length(detected_obstacles), 4);
            for b = 1:length(detected_obstacles)
                obstacle_boxes(b, :) = [detected_obstacles(b).Position(1), detected_obstacles(b).Position(2), ...
                                        detected_obstacles(b).Dimensions(1), detected_obstacles(b).Dimensions(2)];
            end
            hybrid_guide = obj.HybridAStar.plan(ego_state(1:3), nudged_goal, obstacle_boxes, road_width);

            % 4. Level 3: Dynamically Configure DWA from PPO Directives
            target_pt = hybrid_guide(min(15, size(hybrid_guide, 1)), 1:2);
            obj.DWA.MaxSpeed = obj.TargetSpeed * ppo_action.SpeedFactor;
            obj.DWA.WeightRiskCost = ppo_action.RiskWeight;

            % Emergency Override if Risk is Critical
            if max_risk >= 0.85
                obj.DWA.MaxSpeed = 0.0; % Force stopping trajectory
                obj.DWA.WeightClearance = 0.80;
            end

            % Execute DWA velocity search
            [best_cmd, best_traj] = obj.DWA.plan(ego_state, target_pt, detected_obstacles, []);

            % 5. Smooth Trajectory Splicing & Output Formatting
            plan_output.Command = struct('TargetVelocity', best_cmd(1), 'TargetYawRate', best_cmd(2));
            plan_output.LocalTrajectory = best_traj; % [x, y, yaw]
            plan_output.TacticalNudge = ppo_action.NudgeOffset;
            plan_output.SpeedFactor = ppo_action.SpeedFactor;
            plan_output.DWARiskWeight = ppo_action.RiskWeight;
            plan_output.StateValue = state_value;
            plan_output.GuidePath = hybrid_guide;

            obj.LastTrajectory = best_traj;
        end
    end
end

function val = np_clip(val, min_val, max_val)
    val = max(min_val, min(max_val, val));
end
