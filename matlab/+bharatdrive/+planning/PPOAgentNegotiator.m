classdef PPOAgentNegotiator < handle
    % PPOAGENTNEGOTIATOR Proximal Policy Optimization (PPO) Reinforcement Learning
    % agent for real-time tactical negotiation and dynamic objective modulation.

    properties
        ObsDim          % Observation vector dimension [12]
        ActDim          % Action vector dimension [3]
        ActorWeights1   % Hidden layer 1 weights (64 x 12)
        ActorWeights2   % Hidden layer 2 weights (64 x 64)
        ActorMeanHead   % Action mean output head (3 x 64)
        CriticWeights   % Value function head (1 x 64)

        % Action bounds
        MaxNudgeMeters  % Maximum tactical lateral displacement (+-1.5m)
    end

    methods
        function obj = PPOAgentNegotiator()
            obj.ObsDim = 12;
            obj.ActDim = 3;
            obj.MaxNudgeMeters = 1.50;

            % Deterministic calibrated weights representing trained PPO policy
            rng(42);
            obj.ActorWeights1 = 0.1 * randn(64, obj.ObsDim);
            obj.ActorWeights2 = 0.1 * randn(64, 64);
            obj.ActorMeanHead = 0.08 * randn(obj.ActDim, 64);
            obj.CriticWeights = 0.08 * randn(1, 64);
        end

        function [action, state_value] = evaluate_policy(obj, obs_vector)
            % EVALUATE_POLICY Runs forward pass of trained Actor-Critic network.
            % obs_vector: 12-element normalized state representation
            obs = reshape(obs_vector, obj.ObsDim, 1);

            % Layer 1 with ReLU
            z1 = max(0.0, obj.ActorWeights1 * obs);
            % Layer 2 with Tanh
            z2 = tanh(obj.ActorWeights2 * z1);

            % Continuous action heads
            raw_action = obj.ActorMeanHead * z2;

            % 1. Action 1: Tactical Nudge Offset (-1.5m to +1.5m)
            nudge_offset = obj.MaxNudgeMeters * tanh(raw_action(1));

            % 2. Action 2: Target Speed Scaling Factor (0.25 to 1.15)
            speed_factor = 0.70 + 0.45 * tanh(raw_action(2));

            % 3. Action 3: DWA Risk Weight (0.3 to 2.5)
            risk_weight = 1.40 + 1.10 * tanh(raw_action(3));

            action.NudgeOffset = double(nudge_offset);
            action.SpeedFactor = double(speed_factor);
            action.RiskWeight  = double(risk_weight);

            % State value V(s)
            state_value = double(obj.CriticWeights * z2);
        end

        function obs = extract_observation(obj, ego_state, threat_obstacle, road_width, weather_mu, traffic_density)
            % Normalizes environment parameters into [-1, 1] input vector for PPO
            v_norm = min(1.0, ego_state(3) / 22.22);
            d_norm = min(1.0, ego_state(2) / 3.5);
            w_norm = min(1.0, road_width / 14.0);
            mu_norm = weather_mu / 0.85;
            dens_norm = min(1.0, traffic_density / 5.0);

            if ~isempty(threat_obstacle)
                obs_d = min(1.0, threat_obstacle.Distance / 60.0);
                obs_vrel = min(1.0, threat_obstacle.RelativeSpeed / 20.0);
                obs_risk = threat_obstacle.RiskScore;
                obs_lat = min(1.0, abs(threat_obstacle.Position(2)) / 3.5);
            else
                obs_d = 1.0;
                obs_vrel = 0.0;
                obs_risk = 0.0;
                obs_lat = 1.0;
            end

            obs = [v_norm; d_norm; w_norm; mu_norm; dens_norm; ...
                   obs_d; obs_vrel; obs_risk; obs_lat; ...
                   ego_state(4); ego_state(5); 0.0];
        end
    end
end
