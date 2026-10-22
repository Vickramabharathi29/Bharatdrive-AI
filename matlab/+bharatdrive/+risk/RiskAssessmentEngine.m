classdef RiskAssessmentEngine < handle
    % RISKASSESSMENTENGINE Dynamic Risk Assessment Engine for Unstructured Indian Roads.
    % Evaluates the Indian Road Risk Metric (IRRM) across 7 physical and contextual dimensions:
    % 1. Distance (d)
    % 2. Relative Speed (v_rel)
    % 3. Direction / Heading Contention (theta)
    % 4. Object Type & Vulnerability (M_type)
    % 5. Road Width Lateral Escape Bottleneck (C_width)
    % 6. Weather, Road Friction & Visibility (Omega_weather)
    % 7. Local Traffic Density / Multi-Agent Pinch (Psi_density)

    properties
        Tau0            % Critical Time-to-Collision horizon (s) [default 3.5s]
        D0              % Minimum buffer distance (m) [default 3.0m]
        TReact          % Driver / Perception reaction time (s) [default 1.0s]
        AComfort        % Passenger comfort braking deceleration (m/s^2) [default 2.5]
        NominalRoadWidth % Baseline road width (m) [default 7.0m for standard 2-lane road]
        MuDry           % Baseline dry clean asphalt friction coefficient [default 0.85]
        Gravity         % Gravitational acceleration (m/s^2) [9.81]

        % Vulnerability and severity lookup table
        TypeVulnerabilityMap
    end

    methods
        function obj = RiskAssessmentEngine()
            obj.Tau0 = 3.5;
            obj.D0 = 3.0;
            obj.TReact = 1.0;
            obj.AComfort = 2.5;
            obj.NominalRoadWidth = 7.0;
            obj.MuDry = 0.85;
            obj.Gravity = 9.81;

            % Calibrated vulnerability and structural severity weights
            obj.TypeVulnerabilityMap = containers.Map();
            obj.TypeVulnerabilityMap('Pedestrian')    = 2.50;
            obj.TypeVulnerabilityMap('Cattle')        = 2.20;
            obj.TypeVulnerabilityMap('Cow')           = 2.20;
            obj.TypeVulnerabilityMap('Two-Wheeler')   = 2.00;
            obj.TypeVulnerabilityMap('Bike')          = 2.00;
            obj.TypeVulnerabilityMap('Cycle')         = 2.00;
            obj.TypeVulnerabilityMap('Auto-Rickshaw') = 1.80;
            obj.TypeVulnerabilityMap('Truck-Bus')     = 2.40;
            obj.TypeVulnerabilityMap('Bus')           = 2.40;
            obj.TypeVulnerabilityMap('Truck')         = 2.40;
            obj.TypeVulnerabilityMap('Pothole')       = 1.60;
            obj.TypeVulnerabilityMap('Push-Cart')     = 1.70;
            obj.TypeVulnerabilityMap('Car')           = 1.00;
        end

        function assessment = evaluate_risk(obj, d, v_rel, approach_angle_deg, ...
                                            object_type, road_width, weather, ...
                                            traffic_density, ego_speed)
            % EVALUATE_RISK Computes the complete IRRM score and actionable decision.
            %
            % Parameters:
            %   d                   - Distance to obstacle (m)
            %   v_rel               - Relative closing speed (m/s) (>0 = approaching)
            %   approach_angle_deg  - Angle between obstacle velocity and line-of-sight to ego (deg)
            %                         (180 = oncoming head-on, 90 = cut-in, 0 = preceding)
            %   object_type         - String class name ('Cattle', 'Auto-Rickshaw', etc.)
            %   road_width          - Total carriageway width (m) (e.g. 3.75, 5.5, 7.0, 14.0)
            %   weather             - Struct with fields: .friction_mu (0.35-0.85), .visibility (0.0-1.0)
            %   traffic_density     - Number of nearby agents within 20m radius (>= 0)
            %   ego_speed           - Ego vehicle forward speed (m/s)

            if nargin < 9 || isempty(ego_speed)
                ego_speed = 13.89; % 50 km/h default
            end
            if nargin < 8 || isempty(traffic_density)
                traffic_density = 0;
            end
            if nargin < 7 || isempty(weather)
                weather.friction_mu = 0.85;
                weather.visibility = 1.0;
            end

            % --- 1. Dimension 1 & 2: Distance & Relative Speed (Kinetic Risk) ---
            if v_rel > 0.05
                ttc = d / v_rel;
                r_ttc = exp(-ttc / obj.Tau0);
            else
                ttc = inf;
                r_ttc = 0.0;
            end

            % Safe stopping distance under comfort deceleration
            d_safe = obj.D0 + ego_speed * obj.TReact + (ego_speed^2) / (2 * obj.AComfort);
            r_prox = exp(-d / max(5.0, d_safe));

            % Kinetic baseline
            r_kin = 0.65 * r_ttc + 0.35 * r_prox;

            % --- 2. Dimension 3: Direction (Approach Vector Contention) ---
            rad = deg2rad(approach_angle_deg);
            % Oncoming traffic (cos ~ -1) gets high penalty; orthogonal cut-in (sin ~ 1) gets penalty
            phi_dir = 1.0 + 0.60 * max(0.0, -cos(rad)) + 0.45 * abs(sin(rad));

            % --- 3. Dimension 4: Object Type Vulnerability & Severity ---
            if isKey(obj.TypeVulnerabilityMap, object_type)
                m_type = obj.TypeVulnerabilityMap(object_type);
            else
                m_type = 1.00;
            end

            % --- 4. Dimension 5: Road Width (Lateral Escape Bottleneck) ---
            % Narrow roads heavily restrict evasion corridor
            eff_w = max(2.5, road_width);
            c_width = (obj.NominalRoadWidth / eff_w)^1.4;

            % --- 5. Dimension 6: Weather & Road Surface Friction ---
            mu = max(0.20, min(1.0, weather.friction_mu));
            vis = max(0.20, min(1.0, weather.visibility));
            omega_weather = ((obj.MuDry / mu)^0.85) * (1.0 + 0.35 * (1.0 - vis));

            % Required emergency physical stopping distance
            d_stop_emergency = (ego_speed^2) / (2.0 * mu * obj.Gravity);

            % --- 6. Dimension 7: Traffic Density (Multi-Agent Evasion Pinch) ---
            psi_density = 1.0 + 0.12 * min(5.0, double(traffic_density));

            % --- 7. Total Unified Indian Road Risk Metric (IRRM) ---
            raw_risk = r_kin * (0.35 * phi_dir + 0.25 * m_type + 0.20 * c_width + 0.15 * omega_weather + 0.05 * psi_density);
            risk_score = min(1.0, max(0.0, raw_risk));

            % Hard safety override: if distance is less than emergency stopping distance
            if d <= d_stop_emergency && v_rel > 0
                risk_score = max(risk_score, 0.90);
            end

            % Determine Criticality Zone & Action Recommendation
            if risk_score < 0.35
                criticality = 'LOW';
                action = 'MAINTAIN_CRUISE';
            elseif risk_score < 0.65
                criticality = 'MEDIUM';
                action = 'PREPARE_NUDGE';
            elseif risk_score < 0.85
                criticality = 'HIGH';
                action = 'ACTIVE_DECELERATION';
            else
                criticality = 'CRITICAL';
                action = 'EMERGENCY_BRAKE';
            end

            % Package outputs
            assessment.RiskScore = double(risk_score);
            assessment.Criticality = criticality;
            assessment.Time_to_Collision = double(ttc);
            assessment.StoppingDistance = double(d_stop_emergency);
            assessment.ActionRecommendation = action;
            assessment.Factors = struct( ...
                'R_kin', r_kin, ...
                'Phi_dir', phi_dir, ...
                'M_type', m_type, ...
                'C_width', c_width, ...
                'Omega_weather', omega_weather, ...
                'Psi_density', psi_density ...
            );
        end

        function scene_risks = assess_scene_tracks(obj, tracked_objects, road_width, weather, ego_speed)
            % ASSESS_SCENE_TRACKS Evaluates risk for all active TrackedObject instances.
            num_tracks = length(tracked_objects);
            scene_risks = cell(num_tracks, 1);

            for i = 1:num_tracks
                tr = tracked_objects{i};
                d = tr.Distance;
                v_rel = tr.RelativeSpeed;

                % Calculate approach angle from obstacle velocity relative to line-of-sight
                % In ego frame: line of sight is from obstacle [x, y] to ego [0, 0] = [-x, -y]
                los_angle = atan2(-tr.Position(2), -tr.Position(1));
                obs_vel_angle = atan2(tr.Velocity(2), tr.Velocity(1));
                approach_diff = rad2deg(atan2(sin(obs_vel_angle - los_angle), cos(obs_vel_angle - los_angle)));

                % Neighboring obstacles density
                local_density = max(0, num_tracks - 1);

                res = obj.evaluate_risk(d, v_rel, abs(approach_diff), tr.ObjectClass, ...
                                       road_width, weather, local_density, ego_speed);
                res.ObjectID = tr.ObjectID;
                res.ObjectClass = tr.ObjectClass;
                res.Position = tr.Position;
                scene_risks{i} = res;
            end
        end
    end
end
