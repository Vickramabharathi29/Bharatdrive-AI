classdef BharatDriveDashboard < matlab.apps.AppBase
    % BHARATDRIVEDASHBOARD MATLAB App Designer interface for BharatDrive-AI.
    % Real-Time Autonomous Driving Telemetry, Risk Assessment & Perception Dashboard
    % calibrated for Unstructured Indian Road Scenarios.
    %
    % Displays:
    %   1. Vehicle Speed (Gauge & Digital Readout, Target Speed, Accel)
    %   2. Current Risk (7-Factor IRRM Score, Criticality Level, Action, Factor Bar Chart)
    %   3. Detected Objects (Detailed Tracked Objects Table & BEV Markers)
    %   4. Collision Warning (FCW / AEB Alert Lamp, Dynamic TTC Countdown, Threat Details)
    %   5. Current Path (2D Bird's-Eye View Trajectory, Hybrid A* Corridor, DWA Path, Nudge)
    %   6. FPS (Real-time Simulation/Rendering Framerate & Step Execution Latency)
    %   7. Sensor Status (Camera, 3D LiDAR, Radar, IMU/GNSS, Health & Weather Degradation)
    %   8. Scenario Name (Scenario Selector: Village Road, Busy Market, Urban Junction, Highway, Cattle Crossing)
    %
    % BharatDrive-AI - MathWorks Autonomous Driving Hackathon.

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                      matlab.ui.Figure
        MainGrid                      matlab.ui.container.GridLayout

        % Header Panel
        HeaderPanel                   matlab.ui.container.Panel
        HeaderGrid                    matlab.ui.container.GridLayout
        TitleLabel                    matlab.ui.control.Label
        SubtitleLabel                 matlab.ui.control.Label
        ScenarioDropDownLabel         matlab.ui.control.Label
        ScenarioDropDown              matlab.ui.control.DropDown
        FPSBadge                      matlab.ui.control.Label
        SimTimeBadge                  matlab.ui.control.Label

        % Body Layout Grid (3 Columns: Left Telemetry, Center BEV & Controls, Right Risk & Objects)
        BodyGrid                      matlab.ui.container.GridLayout

        % --- LEFT COLUMN: Vehicle Speed, Collision Warning, Sensors ---
        LeftPanel                     matlab.ui.container.Panel
        LeftGrid                      matlab.ui.container.GridLayout

        % Speedometer Card
        SpeedPanel                    matlab.ui.container.Panel
        SpeedGrid                     matlab.ui.container.GridLayout
        SpeedGauge                    matlab.ui.control.Gauge
        SpeedDigitalLabel             matlab.ui.control.Label
        SpeedSubLabel                 matlab.ui.control.Label
        TargetSpeedLabel              matlab.ui.control.Label
        AccelStateLabel               matlab.ui.control.Label

        % Collision Warning Card
        WarningPanel                  matlab.ui.container.Panel
        WarningGrid                   matlab.ui.container.GridLayout
        WarningLamp                   matlab.ui.control.Lamp
        WarningBanner                 matlab.ui.control.Label
        TTCLabel                      matlab.ui.control.Label
        ThreatDescLabel               matlab.ui.control.Label

        % Sensor Status Card
        SensorPanel                   matlab.ui.container.Panel
        SensorGrid                    matlab.ui.container.GridLayout
        CameraLamp                    matlab.ui.control.Lamp
        CameraLabel                   matlab.ui.control.Label
        LidarLamp                     matlab.ui.control.Lamp
        LidarLabel                    matlab.ui.control.Label
        RadarLamp                     matlab.ui.control.Lamp
        RadarLabel                    matlab.ui.control.Label
        IMULamp                       matlab.ui.control.Lamp
        IMULabel                      matlab.ui.control.Label
        WeatherLabel                  matlab.ui.control.Label

        % --- CENTER COLUMN: Current Path (BEV Map) & Playback Controls ---
        CenterPanel                   matlab.ui.container.Panel
        CenterGrid                    matlab.ui.container.GridLayout

        BEVCard                       matlab.ui.container.Panel
        BEVAxes                       matlab.ui.control.UIAxes

        ControlPanel                  matlab.ui.container.Panel
        ControlGrid                   matlab.ui.container.GridLayout
        PlayButton                    matlab.ui.control.Button
        PauseButton                   matlab.ui.control.Button
        StepButton                    matlab.ui.control.Button
        ResetButton                   matlab.ui.control.Button
        SpeedLabel                    matlab.ui.control.Label
        SpeedMultiplierDropDown       matlab.ui.control.DropDown
        EmergencyStopBtn              matlab.ui.control.Button
        StateflowBadge                matlab.ui.control.Label

        % --- RIGHT COLUMN: Risk Assessment & Detected Objects ---
        RightPanel                    matlab.ui.container.Panel
        RightGrid                     matlab.ui.container.GridLayout

        % Risk Assessment Card
        RiskCard                      matlab.ui.container.Panel
        RiskGrid                      matlab.ui.container.GridLayout
        RiskScoreLabel                matlab.ui.control.Label
        RiskLevelBadge                matlab.ui.control.Label
        RiskActionLabel               matlab.ui.control.Label
        RiskGauge                     matlab.ui.control.LinearGauge
        RiskFactorAxes                matlab.ui.control.UIAxes

        % Detected Objects Card
        ObjectsCard                   matlab.ui.container.Panel
        ObjectsGrid                   matlab.ui.container.GridLayout
        ObjectsTitleLabel             matlab.ui.control.Label
        ObjectsTable                  matlab.ui.control.Table
    end

    % Private Properties for Simulation Engine & State Management
    properties (Access = private)
        SimTimer                      timer
        IsRunning                     logical = false
        StepCount                     double = 0
        SimTime                       double = 0.0
        DeltaT                        double = 0.05   % 20 Hz cycle
        SpeedMultiplier               double = 1.0

        % Performance Timing & FPS
        LastFrameTic                  uint64
        FrameTimeHistory              double = []
        CurrentFPS                    double = 20.0

        % Scenario Data
        ActiveScenarioID              double = 5
        ScenarioName                  char = 'Scenario 5: Cattle Crossing'
        RoadWidth                     double = 7.00
        RoadLength                    double = 200.0
        SurfaceMu                     double = 0.82
        Visibility                    double = 1.00

        % Ego Vehicle State [x; y; v; yaw; steer]
        EgoState                      double = [0.0; -1.75; 8.33; 0.0; 0.0]
        EgoNominalSpeed               double = 13.89  % 50 km/h
        EgoLength                     double = 3.993
        EgoWidth                      double = 1.811
        EgoHistory                    double = []     % [x, y] history

        % Scenario Obstacles struct array
        Obstacles                     struct = struct([])

        % Planning & Control outputs
        GuidePath                     double = []     % [x, y]
        LocalTraj                     double = []     % [x, y]
        TacticalNudge                 double = 0.0
        CurrentStateflow              char = 'Accelerate'
        AccelCmd                      double = 0.0
        SteerCmd                      double = 0.0

        % Risk & Warning State
        CurrentRiskScore              double = 0.0
        CurrentCriticality            char = 'LOW'
        CurrentAction                 char = 'MAINTAIN_CRUISE'
        CurrentTTC                    double = inf
        PrimaryThreatDesc             char = 'Clear Roadway'
        RiskFactorValues              double = zeros(1, 6)

        % Engines (if toolbox available, else native embedded logic)
        RiskEngine
        HasNativeEngines              logical = false
    end

    % Methods: Callbacks and Event Handlers
    methods (Access = private)

        % --- Start / Pause / Step / Reset Callbacks ---
        function PlayButtonPushed(app, ~)
            if ~app.IsRunning
                app.IsRunning = true;
                app.PlayButton.BackgroundColor = [0.10, 0.40, 0.20];
                app.PauseButton.BackgroundColor = [0.18, 0.22, 0.29];
                start(app.SimTimer);
            end
        end

        function PauseButtonPushed(app, ~)
            if app.IsRunning
                app.IsRunning = false;
                stop(app.SimTimer);
                app.PlayButton.BackgroundColor = [0.18, 0.22, 0.29];
                app.PauseButton.BackgroundColor = [0.45, 0.35, 0.10];
            end
        end

        function StepButtonPushed(app, ~)
            if app.IsRunning
                app.PauseButtonPushed();
            end
            app.stepSimulation();
        end

        function ResetButtonPushed(app, ~)
            app.PauseButtonPushed();
            app.loadScenario(app.ActiveScenarioID);
            app.updateUI();
        end

        function ScenarioDropDownValueChanged(app, ~)
            val = app.ScenarioDropDown.Value;
            switch val
                case 'Scenario 1: Village Road (Gramin Sadak)'
                    sc_id = 1;
                case 'Scenario 2: Busy Market (Bazaar Street)'
                    sc_id = 2;
                case 'Scenario 3: Urban Junction (Chowk)'
                    sc_id = 3;
                case 'Scenario 4: Highway Merge (National Highway)'
                    sc_id = 4;
                case 'Scenario 5: Cattle Crossing (Bovine Obstruction)'
                    sc_id = 5;
                case 'Scenario 6: Mixed Hazard Master (Cow + Auto + Pothole)'
                    sc_id = 6;
                otherwise
                    sc_id = 5;
            end
            app.loadScenario(sc_id);
            app.updateUI();
        end

        function SpeedMultiplierValueChanged(app, ~)
            val_str = app.SpeedMultiplierDropDown.Value;
            switch val_str
                case '0.5x'
                    app.SpeedMultiplier = 0.5;
                case '1.0x (Real-Time)'
                    app.SpeedMultiplier = 1.0;
                case '2.0x'
                    app.SpeedMultiplier = 2.0;
                case '5.0x'
                    app.SpeedMultiplier = 5.0;
                otherwise
                    app.SpeedMultiplier = 1.0;
            end
            % Adjust timer period accordingly
            if isvalid(app.SimTimer)
                was_running = app.IsRunning;
                if was_running, stop(app.SimTimer); end
                new_period = max(0.01, app.DeltaT / app.SpeedMultiplier);
                app.SimTimer.Period = round(new_period, 3);
                if was_running, start(app.SimTimer); end
            end
        end

        function EmergencyStopBtnPushed(app, ~)
            % Manual AEB Override
            app.CurrentStateflow = 'Emergency Stop';
            app.AccelCmd = -6.5;
            app.EgoState(3) = max(0.0, app.EgoState(3) - 6.5 * app.DeltaT);
            app.CurrentRiskScore = 0.98;
            app.CurrentCriticality = 'CRITICAL';
            app.CurrentAction = 'EMERGENCY_BRAKE';
            app.updateUI();
        end

        % --- Master Simulation Loop Cycle (Triggered by Timer at 20 Hz) ---
        function onTimerTick(app, ~, ~)
            tic_now = tic;
            app.stepSimulation();
            frame_elapsed = toc(tic_now);

            % Update FPS moving average
            if frame_elapsed > 0
                inst_fps = 1.0 / max(frame_elapsed, 0.001);
                app.CurrentFPS = 0.85 * app.CurrentFPS + 0.15 * min(inst_fps, 60.0);
            end
        end

        function stepSimulation(app)
            app.StepCount = app.StepCount + 1;
            dt = app.DeltaT;
            app.SimTime = app.SimTime + dt;

            % 1. Propagate Scenario Dynamic Obstacles
            for k = 1:length(app.Obstacles)
                ob = app.Obstacles(k);
                ob.Position(1) = ob.Position(1) + ob.Velocity(1) * dt;
                ob.Position(2) = ob.Position(2) + ob.Velocity(2) * dt;
                
                % Subtle autonomous wandering for dynamic actors
                if strcmp(ob.Class, 'Cattle') && ob.Velocity(2) > 0.0
                    % Calf crossing road stops after reaching center
                    if ob.Position(2) >= 0.0
                        ob.Velocity(2) = 0.0;
                    end
                elseif strcmp(ob.Class, 'Auto-Rickshaw') && ob.Velocity(2) ~= 0.0
                    % Auto cutting in reaches left-center lane
                    if ob.Position(2) >= -0.3
                        ob.Velocity(2) = 0.0;
                    end
                end
                app.Obstacles(k) = ob;
            end

            % 2. Sensor Perception & Object Tracking relative to Ego
            num_obs = length(app.Obstacles);
            fused_objects = app.Obstacles;
            highest_risk = 0.0;
            min_ttc = inf;
            threat_name = 'Clear Corridor';
            lead_threat_dist = inf;

            for k = 1:num_obs
                dx = fused_objects(k).Position(1) - app.EgoState(1);
                dy = fused_objects(k).Position(2) - app.EgoState(2);
                dist = hypot(dx, dy);
                v_rel = app.EgoState(3) - fused_objects(k).Velocity(1);
                angle_deg = rad2deg(atan2(dy, dx) - app.EgoState(4));

                fused_objects(k).Distance = dist;
                fused_objects(k).RelativeSpeed = v_rel;
                fused_objects(k).AngleDeg = angle_deg;

                % Sensor Contributors logic based on FOV and distance
                sensors = {};
                if dist <= 80.0 && abs(angle_deg) <= 32.5
                    sensors{end+1} = 'CAM';
                end
                if dist <= 90.0 && abs(angle_deg) <= 60.0
                    sensors{end+1} = 'LIDAR';
                end
                if dist <= 150.0 && abs(angle_deg) <= 22.5
                    sensors{end+1} = 'RADAR';
                end
                if isempty(sensors)
                    sensors{end+1} = 'EST';
                end
                fused_objects(k).Sensors = strjoin(sensors, '+');
                fused_objects(k).Confidence = min(0.98, 0.70 + 0.10 * length(sensors));

                % 3. Evaluate 7-Factor Indian Road Risk Metric (IRRM)
                risk_eval = app.computeIRRM(dist, v_rel, angle_deg, fused_objects(k).Class, ...
                    app.RoadWidth, app.SurfaceMu, app.Visibility, num_obs, app.EgoState(3));
                fused_objects(k).RiskScore = risk_eval.Score;
                fused_objects(k).TTC = risk_eval.TTC;

                if risk_eval.Score > highest_risk
                    highest_risk = risk_eval.Score;
                    min_ttc = risk_eval.TTC;
                    threat_name = sprintf('%s at %0.1fm', fused_objects(k).Class, dist);
                    lead_threat_dist = dist;
                    app.RiskFactorValues = risk_eval.Factors;
                end
            end

            app.CurrentRiskScore = highest_risk;
            app.CurrentTTC = min_ttc;
            app.PrimaryThreatDesc = threat_name;

            % Criticality & Action Mapping
            if highest_risk < 0.35
                app.CurrentCriticality = 'LOW';
                app.CurrentAction = 'MAINTAIN_CRUISE';
            elseif highest_risk < 0.65
                app.CurrentCriticality = 'MEDIUM';
                app.CurrentAction = 'PREPARE_NUDGE';
            elseif highest_risk < 0.85
                app.CurrentCriticality = 'HIGH';
                app.CurrentAction = 'ACTIVE_DECELERATION';
            else
                app.CurrentCriticality = 'CRITICAL';
                app.CurrentAction = 'EMERGENCY_BRAKE';
            end

            % 4. Stateflow Decision Supervisor Logic
            target_v = app.EgoNominalSpeed;
            target_nudge = 0.0;

            if strcmp(app.CurrentAction, 'EMERGENCY_BRAKE') || (min_ttc < 1.4 && min_ttc > 0)
                app.CurrentStateflow = 'Emergency Stop';
                app.AccelCmd = -6.5;
                target_v = 0.0;
            elseif strcmp(app.CurrentAction, 'ACTIVE_DECELERATION')
                app.CurrentStateflow = 'Brake';
                app.AccelCmd = -3.2;
                target_v = max(3.0, app.EgoNominalSpeed * 0.45);
            elseif strcmp(app.CurrentAction, 'PREPARE_NUDGE')
                % Determine nudge side: if obstacle is right of ego center, nudge left; else right
                ob_lat = 0.0;
                for k = 1:num_obs
                    if fused_objects(k).RiskScore == highest_risk
                        ob_lat = fused_objects(k).Position(2);
                        break;
                    end
                end
                if ob_lat >= app.EgoState(2)
                    app.CurrentStateflow = 'Move Left';
                    target_nudge = -1.2; % nudge towards left shoulder/lane
                else
                    app.CurrentStateflow = 'Move Right';
                    target_nudge = 1.2;  % nudge towards center/right
                end
                app.AccelCmd = -1.0;
                target_v = app.EgoNominalSpeed * 0.75;
            else
                app.CurrentStateflow = 'Accelerate';
                app.AccelCmd = 1.5 * (target_v - app.EgoState(3));
                target_nudge = 0.0;
            end
            app.TacticalNudge = 0.85 * app.TacticalNudge + 0.15 * target_nudge;

            % 5. Path Planning (Corridor + DWA Rollout)
            app.generatePaths(target_v);

            % 6. Closed-Loop Kinematic Bicycle Vehicle Plant Update
            % Longitudinal speed update with saturation
            app.EgoState(3) = max(0.0, min(25.0, app.EgoState(3) + app.AccelCmd * dt));
            
            % Steering control: Stanley-style tracking towards guide path
            lookahead_x = app.EgoState(1) + max(4.0, app.EgoState(3) * 0.8);
            target_y = -1.75 + app.TacticalNudge;
            heading_err = atan2(target_y - app.EgoState(2), lookahead_x - app.EgoState(1)) - app.EgoState(4);
            heading_err = atan2(sin(heading_err), cos(heading_err));
            app.SteerCmd = max(-0.45, min(0.45, 0.75 * heading_err));

            % 2-DOF Bicycle update
            L = 2.498; % Wheelbase Tata Nexon EV
            app.EgoState(4) = app.EgoState(4) + (app.EgoState(3) / L) * tan(app.SteerCmd) * dt;
            app.EgoState(1) = app.EgoState(1) + app.EgoState(3) * cos(app.EgoState(4)) * dt;
            app.EgoState(2) = app.EgoState(2) + app.EgoState(3) * sin(app.EgoState(4)) * dt;
            app.EgoState(5) = app.SteerCmd;

            % Record history trail
            app.EgoHistory = [app.EgoHistory; app.EgoState(1), app.EgoState(2)];
            if size(app.EgoHistory, 1) > 200
                app.EgoHistory(1, :) = [];
            end

            % Update app table and visualization
            app.Obstacles = fused_objects;
            app.updateUI();

            % Loop back or stop if ego reaches end of track
            if app.EgoState(1) >= (app.RoadLength - 10.0)
                app.PauseButtonPushed();
            end
        end

        % --- Compute 7-Factor Indian Road Risk Metric (IRRM) ---
        function eval = computeIRRM(app, d, v_rel, angle_deg, obj_type, road_w, mu, vis, density, v_ego)
            tau0 = 3.5;
            d0 = 3.0;
            a_comf = 2.5;

            % Dimension 1 & 2: Distance & Closing Speed
            if v_rel > 0.05
                ttc = d / v_rel;
                r_ttc = exp(-ttc / tau0);
            else
                ttc = inf;
                r_ttc = 0.0;
            end
            d_safe = d0 + v_ego * 1.0 + (v_ego^2) / (2 * a_comf);
            r_prox = exp(-d / max(4.0, d_safe));
            r_kin = 0.65 * r_ttc + 0.35 * r_prox;

            % Dimension 3: Approach Angle Contention
            rad = deg2rad(angle_deg);
            phi_dir = 1.0 + 0.60 * max(0.0, -cos(rad)) + 0.45 * abs(sin(rad));

            % Dimension 4: Vulnerability
            switch obj_type
                case 'Pedestrian',   m_type = 2.50;
                case 'Cattle',       m_type = 2.20;
                case 'Two-Wheeler',  m_type = 2.00;
                case 'Auto-Rickshaw',m_type = 1.80;
                case 'Truck-Bus',    m_type = 2.40;
                case 'Pothole',      m_type = 1.60;
                otherwise,           m_type = 1.00;
            end

            % Dimension 5: Road Width Bottleneck
            c_width = (7.0 / max(3.0, road_w))^1.4;

            % Dimension 6: Weather & Friction
            omega_weather = ((0.85 / max(0.25, mu))^0.85) * (1.0 + 0.35 * (1.0 - vis));

            % Dimension 7: Traffic Density
            psi_density = 1.0 + 0.12 * min(5.0, double(density));

            % Combined Unified Risk
            raw_risk = r_kin * (0.35 * phi_dir + 0.25 * m_type + 0.20 * c_width + 0.15 * omega_weather + 0.05 * psi_density);
            score = min(1.0, max(0.0, raw_risk));

            % Emergency stopping distance check
            d_stop_emerg = (v_ego^2) / (2.0 * max(0.2, mu) * 9.81);
            if d <= d_stop_emerg && v_rel > 0
                score = max(score, 0.90);
            end

            eval.Score = score;
            eval.TTC = ttc;
            eval.Factors = [r_kin, phi_dir / 2.0, m_type / 2.5, c_width / 2.0, omega_weather / 2.5, psi_density / 1.6];
        end

        % --- Path Generation (Hybrid A* Corridor + DWA Execution Trajectory) ---
        function generatePaths(app, target_v)
            x_ego = app.EgoState(1);
            y_ego = app.EgoState(2);
            yaw_ego = app.EgoState(4);

            % 1. Global Guide Path (Corridor Spline with Tactical Nudge)
            s_pts = linspace(x_ego, x_ego + 60.0, 30);
            guide_y = zeros(size(s_pts));
            for i = 1:length(s_pts)
                progress = min(1.0, (s_pts(i) - x_ego) / 35.0);
                guide_y(i) = -1.75 + app.TacticalNudge * sin(progress * pi / 2);
            end
            app.GuidePath = [s_pts', guide_y'];

            % 2. DWA Short-Horizon Dynamic Trajectory (2.5s Rollout)
            dwa_dt = 0.1;
            steps_dwa = 25;
            dwa_traj = zeros(steps_dwa, 2);
            sim_x = x_ego;
            sim_y = y_ego;
            sim_yaw = yaw_ego;
            sim_v = app.EgoState(3);

            for s = 1:steps_dwa
                sim_v = 0.95 * sim_v + 0.05 * target_v;
                sim_yaw = sim_yaw + (sim_v / 2.498) * tan(app.SteerCmd) * dwa_dt;
                sim_x = sim_x + sim_v * cos(sim_yaw) * dwa_dt;
                sim_y = sim_y + sim_v * sin(sim_yaw) * dwa_dt;
                dwa_traj(s, :) = [sim_x, sim_y];
            end
            app.LocalTraj = dwa_traj;
        end

        % --- Scenario Loader ---
        function loadScenario(app, sc_id)
            app.ActiveScenarioID = sc_id;
            app.StepCount = 0;
            app.SimTime = 0.0;
            app.EgoHistory = [];
            app.TacticalNudge = 0.0;
            app.SteerCmd = 0.0;
            app.AccelCmd = 0.0;

            switch sc_id
                case 1 % Village Road (Gramin Sadak)
                    app.ScenarioName = 'Scenario 1: Village Road (Gramin Sadak)';
                    app.RoadWidth = 4.20;
                    app.RoadLength = 220.0;
                    app.SurfaceMu = 0.70;
                    app.Visibility = 0.90;
                    app.EgoNominalSpeed = 8.33; % 30 km/h
                    app.EgoState = [0.0; -0.80; 8.33; 0.0; 0.0];

                    app.Obstacles = struct([]);
                    app.Obstacles(1) = struct('ID', 101, 'Class', 'Truck-Bus', 'Position', [70.0, 0.8], ...
                        'Velocity', [-4.5, 0.0], 'Dimensions', [8.2, 2.3], 'Distance', 70.0, 'RelativeSpeed', 12.8, ...
                        'AngleDeg', 180.0, 'Sensors', 'CAM+LIDAR+RADAR', 'Confidence', 0.95, 'RiskScore', 0.40, 'TTC', 5.4);
                    app.Obstacles(2) = struct('ID', 102, 'Class', 'Pedestrian', 'Position', [42.0, -1.9], ...
                        'Velocity', [0.8, 0.1], 'Dimensions', [0.6, 0.6], 'Distance', 42.0, 'RelativeSpeed', 7.5, ...
                        'AngleDeg', -5.0, 'Sensors', 'CAM+LIDAR', 'Confidence', 0.92, 'RiskScore', 0.28, 'TTC', 5.6);

                case 2 % Busy Market (Bazaar Street)
                    app.ScenarioName = 'Scenario 2: Busy Market (Bazaar Street)';
                    app.RoadWidth = 8.50;
                    app.RoadLength = 180.0;
                    app.SurfaceMu = 0.80;
                    app.Visibility = 0.95;
                    app.EgoNominalSpeed = 5.56; % 20 km/h
                    app.EgoState = [0.0; -1.50; 5.56; 0.0; 0.0];

                    app.Obstacles = struct([]);
                    app.Obstacles(1) = struct('ID', 201, 'Class', 'Auto-Rickshaw', 'Position', [22.0, -1.8], ...
                        'Velocity', [4.0, 0.2], 'Dimensions', [2.6, 1.3], 'Distance', 22.0, 'RelativeSpeed', 1.5, ...
                        'AngleDeg', 0.0, 'Sensors', 'CAM+LIDAR+RADAR', 'Confidence', 0.96, 'RiskScore', 0.42, 'TTC', 14.6);
                    app.Obstacles(2) = struct('ID', 202, 'Class', 'Two-Wheeler', 'Position', [35.0, 0.5], ...
                        'Velocity', [6.0, -0.2], 'Dimensions', [1.9, 0.8], 'Distance', 35.0, 'RelativeSpeed', -0.5, ...
                        'AngleDeg', 15.0, 'Sensors', 'CAM+LIDAR', 'Confidence', 0.94, 'RiskScore', 0.32, 'TTC', inf);
                    app.Obstacles(3) = struct('ID', 203, 'Class', 'Pedestrian', 'Position', [15.0, -3.2], ...
                        'Velocity', [0.0, 0.6], 'Dimensions', [0.5, 0.5], 'Distance', 15.0, 'RelativeSpeed', 5.5, ...
                        'AngleDeg', -25.0, 'Sensors', 'CAM+LIDAR', 'Confidence', 0.90, 'RiskScore', 0.55, 'TTC', 2.7);

                case 3 % Urban Junction (Chowk)
                    app.ScenarioName = 'Scenario 3: Urban Junction (Chowk)';
                    app.RoadWidth = 9.00;
                    app.RoadLength = 200.0;
                    app.SurfaceMu = 0.82;
                    app.Visibility = 1.00;
                    app.EgoNominalSpeed = 9.72; % 35 km/h
                    app.EgoState = [0.0; -2.00; 9.72; 0.0; 0.0];

                    app.Obstacles = struct([]);
                    app.Obstacles(1) = struct('ID', 301, 'Class', 'Two-Wheeler', 'Position', [32.0, 3.5], ...
                        'Velocity', [-2.0, -2.5], 'Dimensions', [1.8, 0.7], 'Distance', 32.0, 'RelativeSpeed', 11.7, ...
                        'AngleDeg', 90.0, 'Sensors', 'CAM+LIDAR', 'Confidence', 0.93, 'RiskScore', 0.68, 'TTC', 2.7);
                    app.Obstacles(2) = struct('ID', 302, 'Class', 'Auto-Rickshaw', 'Position', [55.0, 0.0], ...
                        'Velocity', [2.5, 0.0], 'Dimensions', [2.6, 1.3], 'Distance', 55.0, 'RelativeSpeed', 7.2, ...
                        'AngleDeg', 0.0, 'Sensors', 'CAM+LIDAR+RADAR', 'Confidence', 0.97, 'RiskScore', 0.36, 'TTC', 7.6);

                case 4 % Highway Merge (National Highway)
                    app.ScenarioName = 'Scenario 4: Highway Merge (National Highway)';
                    app.RoadWidth = 10.50;
                    app.RoadLength = 300.0;
                    app.SurfaceMu = 0.85;
                    app.Visibility = 1.00;
                    app.EgoNominalSpeed = 16.67; % 60 km/h
                    app.EgoState = [0.0; -3.00; 13.89; 0.0; 0.0];

                    app.Obstacles = struct([]);
                    app.Obstacles(1) = struct('ID', 401, 'Class', 'Truck-Bus', 'Position', [75.0, 0.0], ...
                        'Velocity', [14.0, 0.0], 'Dimensions', [11.5, 2.6], 'Distance', 75.0, 'RelativeSpeed', 2.6, ...
                        'AngleDeg', 0.0, 'Sensors', 'CAM+LIDAR+RADAR', 'Confidence', 0.98, 'RiskScore', 0.30, 'TTC', 28.8);
                    app.Obstacles(2) = struct('ID', 402, 'Class', 'Car', 'Position', [35.0, -1.0], ...
                        'Velocity', [18.0, -0.4], 'Dimensions', [4.2, 1.8], 'Distance', 35.0, 'RelativeSpeed', -1.3, ...
                        'AngleDeg', 20.0, 'Sensors', 'CAM+LIDAR+RADAR', 'Confidence', 0.97, 'RiskScore', 0.25, 'TTC', inf);

                case 5 % Cattle Crossing (Bovine Herd)
                    app.ScenarioName = 'Scenario 5: Cattle Crossing (Bovine Herd)';
                    app.RoadWidth = 7.00;
                    app.RoadLength = 250.0;
                    app.SurfaceMu = 0.82;
                    app.Visibility = 1.00;
                    app.EgoNominalSpeed = 13.89; % 50 km/h
                    app.EgoState = [0.0; -1.75; 11.11; 0.0; 0.0];

                    app.Obstacles = struct([]);
                    app.Obstacles(1) = struct('ID', 501, 'Class', 'Cattle', 'Position', [38.0, -1.5], ...
                        'Velocity', [0.0, 0.0], 'Dimensions', [2.2, 1.0], 'Distance', 38.0, 'RelativeSpeed', 11.1, ...
                        'AngleDeg', 0.0, 'Sensors', 'CAM+LIDAR+RADAR', 'Confidence', 0.98, 'RiskScore', 0.76, 'TTC', 3.4);
                    app.Obstacles(2) = struct('ID', 502, 'Class', 'Cattle', 'Position', [32.0, -3.2], ...
                        'Velocity', [0.0, 1.2], 'Dimensions', [1.4, 0.7], 'Distance', 32.0, 'RelativeSpeed', 11.1, ...
                        'AngleDeg', 22.0, 'Sensors', 'CAM+LIDAR', 'Confidence', 0.94, 'RiskScore', 0.58, 'TTC', 2.8);
                    app.Obstacles(3) = struct('ID', 503, 'Class', 'Truck-Bus', 'Position', [120.0, 1.75], ...
                        'Velocity', [-12.5, 0.0], 'Dimensions', [10.5, 2.5], 'Distance', 120.0, 'RelativeSpeed', 23.6, ...
                        'AngleDeg', 180.0, 'Sensors', 'RADAR+CAM', 'Confidence', 0.95, 'RiskScore', 0.45, 'TTC', 5.1);

                case 6 % Mixed Hazard Master
                    app.ScenarioName = 'Scenario 6: Mixed Hazard Master';
                    app.RoadWidth = 7.00;
                    app.RoadLength = 260.0;
                    app.SurfaceMu = 0.82;
                    app.Visibility = 0.95;
                    app.EgoNominalSpeed = 13.89; % 50 km/h
                    app.EgoState = [0.0; -1.75; 10.0; 0.0; 0.0];

                    app.Obstacles = struct([]);
                    app.Obstacles(1) = struct('ID', 601, 'Class', 'Cattle', 'Position', [28.0, 0.4], ...
                        'Velocity', [0.0, 0.0], 'Dimensions', [2.2, 1.1], 'Distance', 28.0, 'RelativeSpeed', 10.0, ...
                        'AngleDeg', 12.0, 'Sensors', 'CAM+LIDAR+RADAR', 'Confidence', 0.97, 'RiskScore', 0.65, 'TTC', 2.8);
                    app.Obstacles(2) = struct('ID', 602, 'Class', 'Auto-Rickshaw', 'Position', [48.0, -1.8], ...
                        'Velocity', [6.5, 0.25], 'Dimensions', [2.6, 1.3], 'Distance', 48.0, 'RelativeSpeed', 3.5, ...
                        'AngleDeg', 0.0, 'Sensors', 'CAM+LIDAR+RADAR', 'Confidence', 0.98, 'RiskScore', 0.48, 'TTC', 13.7);
                    app.Obstacles(3) = struct('ID', 603, 'Class', 'Pothole', 'Position', [72.0, -1.5], ...
                        'Velocity', [0.0, 0.0], 'Dimensions', [1.3, 1.0], 'Distance', 72.0, 'RelativeSpeed', 10.0, ...
                        'AngleDeg', 0.0, 'Sensors', 'CAM+LIDAR', 'Confidence', 0.92, 'RiskScore', 0.52, 'TTC', 7.2);
                    app.Obstacles(4) = struct('ID', 604, 'Class', 'Truck-Bus', 'Position', [110.0, 0.0], ...
                        'Velocity', [4.0, 0.0], 'Dimensions', [7.5, 2.4], 'Distance', 110.0, 'RelativeSpeed', 6.0, ...
                        'AngleDeg', 0.0, 'Sensors', 'RADAR+LIDAR', 'Confidence', 0.95, 'RiskScore', 0.25, 'TTC', 18.3);
            end

            app.generatePaths(app.EgoNominalSpeed);
        end

        % --- Master UI Renderer & Update Method ---
        function updateUI(app)
            % 1. Header Information & FPS
            app.FPSBadge.Text = sprintf('FPS: %0.1f (20 Hz Loop)', app.CurrentFPS);
            app.SimTimeBadge.Text = sprintf('T: %0.2fs | Step: %d', app.SimTime, app.StepCount);

            % 2. Vehicle Speed Panel
            speed_kmh = app.EgoState(3) * 3.6;
            app.SpeedGauge.Value = min(120, max(0, speed_kmh));
            app.SpeedDigitalLabel.Text = sprintf('%0.1f', speed_kmh);
            app.SpeedSubLabel.Text = sprintf('%0.2f m/s', app.EgoState(3));
            app.TargetSpeedLabel.Text = sprintf('Target: %0.1f km/h', app.EgoNominalSpeed * 3.6);
            if app.AccelCmd > 0.3
                app.AccelStateLabel.Text = sprintf('Accel: +%0.2f m/s²', app.AccelCmd);
                app.AccelStateLabel.FontColor = [0.0, 0.9, 0.4];
            elseif app.AccelCmd < -0.3
                app.AccelStateLabel.Text = sprintf('Braking: %0.2f m/s²', app.AccelCmd);
                app.AccelStateLabel.FontColor = [1.0, 0.3, 0.2];
            else
                app.AccelStateLabel.Text = 'Cruising Steady';
                app.AccelStateLabel.FontColor = [0.4, 0.8, 1.0];
            end

            % 3. Collision Warning & TTC Banner
            if app.CurrentRiskScore >= 0.85 || app.CurrentTTC < 1.5
                app.WarningLamp.Color = [1.0, 0.1, 0.1]; % Red
                app.WarningBanner.Text = 'COLLISION IMMINENT - AEB ENGAGED';
                app.WarningBanner.FontColor = [1.0, 0.2, 0.2];
            elseif app.CurrentRiskScore >= 0.65 || app.CurrentTTC < 3.0
                app.WarningLamp.Color = [1.0, 0.6, 0.0]; % Orange
                app.WarningBanner.Text = 'FCW: FORWARD COLLISION WARNING';
                app.WarningBanner.FontColor = [1.0, 0.6, 0.0];
            elseif app.CurrentRiskScore >= 0.35
                app.WarningLamp.Color = [1.0, 0.9, 0.1]; % Yellow
                app.WarningBanner.Text = 'CAUTION: POTENTIAL THREAT AHEAD';
                app.WarningBanner.FontColor = [1.0, 0.9, 0.1];
            else
                app.WarningLamp.Color = [0.0, 0.9, 0.4]; % Green
                app.WarningBanner.Text = 'SAFE: ROAD CLEAR AHEAD';
                app.WarningBanner.FontColor = [0.0, 0.9, 0.4];
            end

            if isinf(app.CurrentTTC)
                app.TTCLabel.Text = 'TTC: Safe (Inf)';
            else
                app.TTCLabel.Text = sprintf('TTC: %0.1f s', app.CurrentTTC);
            end
            app.ThreatDescLabel.Text = sprintf('Threat: %s', app.PrimaryThreatDesc);

            % 4. Sensor Status Panel
            app.CameraLamp.Color = [0.0, 0.9, 0.4];
            app.CameraLabel.Text = 'Camera: 1080p 60Hz OK';
            app.LidarLamp.Color = [0.0, 0.9, 0.4];
            app.LidarLabel.Text = '3D LiDAR: 128-Beam 20Hz OK';
            app.RadarLamp.Color = [0.0, 0.9, 0.4];
            app.RadarLabel.Text = 'FMCW Radar: 77GHz 50Hz OK';
            app.IMULamp.Color = [0.0, 0.9, 0.4];
            app.IMULabel.Text = 'RTK GNSS/IMU: Fixed 100Hz';
            app.WeatherLabel.Text = sprintf('Friction: μ=%0.2f | Vis: %0.0f%%', app.SurfaceMu, app.Visibility * 100);

            % 5. Current Risk Panel & Factor Bar Chart
            app.RiskScoreLabel.Text = sprintf('%0.0f%%', app.CurrentRiskScore * 100);
            app.RiskGauge.Value = min(1.0, max(0.0, app.CurrentRiskScore));
            app.RiskLevelBadge.Text = sprintf('Criticality: %s', app.CurrentCriticality);
            switch app.CurrentCriticality
                case 'LOW',      app.RiskLevelBadge.FontColor = [0.0, 0.9, 0.4];
                case 'MEDIUM',   app.RiskLevelBadge.FontColor = [1.0, 0.9, 0.1];
                case 'HIGH',     app.RiskLevelBadge.FontColor = [1.0, 0.6, 0.0];
                case 'CRITICAL', app.RiskLevelBadge.FontColor = [1.0, 0.2, 0.2];
            end
            app.RiskActionLabel.Text = sprintf('Action: %s', strrep(app.CurrentAction, '_', ' '));
            app.StateflowBadge.Text = sprintf('State: %s', app.CurrentStateflow);

            % Plot 7-Factor Bar Chart
            cla(app.RiskFactorAxes);
            factors = app.RiskFactorValues;
            if isempty(factors) || all(factors == 0)
                factors = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05];
            end
            b = bar(app.RiskFactorAxes, 1:6, factors, 0.55);
            b.FaceColor = 'flat';
            b.CData(1, :) = [0.2, 0.7, 1.0]; % Kinetic
            b.CData(2, :) = [0.3, 0.9, 0.7]; % Direction
            b.CData(3, :) = [1.0, 0.5, 0.2]; % Vulnerability
            b.CData(4, :) = [0.9, 0.8, 0.2]; % Road Width
            b.CData(5, :) = [0.7, 0.3, 0.9]; % Weather
            b.CData(6, :) = [1.0, 0.3, 0.5]; % Density
            app.RiskFactorAxes.XTick = 1:6;
            app.RiskFactorAxes.XTickLabel = {'Kinetic', 'Heading', 'Vulner.', 'Bottleneck', 'Weather', 'Density'};
            app.RiskFactorAxes.YLim = [0, 1.2];
            app.RiskFactorAxes.YGrid = 'on';

            % 6. Detected Objects Table
            num_obs = length(app.Obstacles);
            if num_obs > 0
                table_data = cell(num_obs, 7);
                for k = 1:num_obs
                    ob = app.Obstacles(k);
                    table_data{k, 1} = uint32(ob.ID);
                    table_data{k, 2} = char(ob.Class);
                    table_data{k, 3} = sprintf('%0.1f m', ob.Distance);
                    table_data{k, 4} = sprintf('%+0.1f m/s', ob.RelativeSpeed);
                    table_data{k, 5} = sprintf('%0.0f%%', ob.Confidence * 100);
                    table_data{k, 6} = char(ob.Sensors);
                    table_data{k, 7} = sprintf('%0.0f%%', ob.RiskScore * 100);
                end
                app.ObjectsTable.Data = table_data;
            else
                app.ObjectsTable.Data = {};
            end

            % 7. Current Path (Bird's-Eye-View Axes) Rendering
            app.renderBEV();
        end

        function renderBEV(app)
            ax = app.BEVAxes;
            cla(ax);
            hold(ax, 'on');

            ego_x = app.EgoState(1);
            ego_y = app.EgoState(2);
            ego_yaw = app.EgoState(4);
            rw = app.RoadWidth;

            % Longitudinal viewing window following ego vehicle
            view_x_min = max(0, ego_x - 12.0);
            view_x_max = view_x_min + 75.0;
            view_y_span = max(rw / 2 + 3.0, 5.5);

            % 1. Draw Road Surface Asphalt & Boundaries
            road_x = [view_x_min, view_x_max, view_x_max, view_x_min];
            road_y = [-rw/2, -rw/2, rw/2, rw/2];
            patch(ax, road_x, road_y, [0.15, 0.17, 0.20], 'EdgeColor', 'none');

            % Left and Right Shoulder / Curb
            plot(ax, [view_x_min, view_x_max], [rw/2, rw/2], 'Color', [0.95, 0.95, 0.95], 'LineWidth', 2.5);
            plot(ax, [view_x_min, view_x_max], [-rw/2, -rw/2], 'Color', [0.95, 0.95, 0.95], 'LineWidth', 2.5);

            % Dashed Center Lane Marking
            dash_x = view_x_min:6.0:view_x_max;
            for d = 1:2:length(dash_x)-1
                plot(ax, [dash_x(d), dash_x(d+1)], [0, 0], 'Color', [1.0, 0.85, 0.1], 'LineWidth', 1.8);
            end

            % 2. Draw Historical Traveled Path
            if size(app.EgoHistory, 1) > 1
                plot(ax, app.EgoHistory(:, 1), app.EgoHistory(:, 2), ...
                    'Color', [0.3, 0.6, 0.9, 0.45], 'LineWidth', 2.0);
            end

            % 3. Draw Hybrid A* Planned Guide Path Corridor
            if ~isempty(app.GuidePath)
                plot(ax, app.GuidePath(:, 1), app.GuidePath(:, 2), ...
                    'Color', [0.0, 0.85, 1.0, 0.60], 'LineWidth', 2.2, 'LineStyle', '--');
            end

            % 4. Draw DWA Local Trajectory Execution Path
            if ~isempty(app.LocalTraj)
                plot(ax, app.LocalTraj(:, 1), app.LocalTraj(:, 2), ...
                    'Color', [0.0, 1.0, 0.5], 'LineWidth', 2.8);
            end

            % 5. Draw Tactical Nudge Offset Indicator
            if abs(app.TacticalNudge) > 0.1
                nudge_start_y = -1.75;
                nudge_end_y = -1.75 + app.TacticalNudge;
                quiver(ax, ego_x + 5.0, nudge_start_y, 0, app.TacticalNudge, 0, ...
                    'Color', [1.0, 0.4, 0.9], 'LineWidth', 2.0, 'MaxHeadSize', 2.0);
                text(ax, ego_x + 6.0, (nudge_start_y + nudge_end_y)/2, ...
                    sprintf('Nudge %+0.1fm', app.TacticalNudge), ...
                    'Color', [1.0, 0.6, 1.0], 'FontSize', 9, 'FontWeight', 'bold');
            end

            % 6. Draw Detected Obstacles with Oriented Bounding Boxes
            for k = 1:length(app.Obstacles)
                ob = app.Obstacles(k);
                ox = ob.Position(1);
                oy = ob.Position(2);
                ow = ob.Dimensions(2);
                ol = ob.Dimensions(1);

                % Color selection based on object class
                switch ob.Class
                    case 'Cattle',       box_color = [1.0, 0.4, 0.1];
                    case 'Auto-Rickshaw',box_color = [1.0, 0.8, 0.0];
                    case 'Two-Wheeler',  box_color = [0.2, 0.8, 1.0];
                    case 'Truck-Bus',    box_color = [0.9, 0.2, 0.2];
                    case 'Pedestrian',   box_color = [0.8, 0.3, 0.9];
                    case 'Pothole',      box_color = [0.5, 0.5, 0.5];
                    otherwise,           box_color = [0.7, 0.7, 0.7];
                end

                % Draw Bounding Box rectangle
                x_box = [ox - ol/2, ox + ol/2, ox + ol/2, ox - ol/2];
                y_box = [oy - ow/2, oy - ow/2, oy + ow/2, oy + ow/2];
                patch(ax, x_box, y_box, box_color, 'FaceAlpha', 0.65, 'EdgeColor', box_color, 'LineWidth', 1.5);

                % Velocity vector
                if norm(ob.Velocity(1:2)) > 0.5
                    quiver(ax, ox, oy, ob.Velocity(1)*1.5, ob.Velocity(2)*1.5, 0, ...
                        'Color', [1.0, 1.0, 1.0], 'LineWidth', 1.2, 'MaxHeadSize', 1.5);
                end

                % Label
                text(ax, ox, oy + ow/2 + 0.6, sprintf('%s (%0.0fm)', ob.Class, ob.Distance), ...
                    'Color', [1.0, 1.0, 1.0], 'FontSize', 8, 'HorizontalAlignment', 'center', ...
                    'FontWeight', 'bold');
            end

            % 7. Draw Ego Vehicle (Tata Nexon EV)
            L = app.EgoLength;
            W = app.EgoWidth;
            % 4 corners in ego coordinate frame
            corners_body = [-L/2, -W/2; L/2, -W/2; L/2, W/2; -L/2, W/2]';
            % Rotation matrix
            R = [cos(ego_yaw), -sin(ego_yaw); sin(ego_yaw), cos(ego_yaw)];
            corners_world = R * corners_body + [ego_x; ego_y];

            patch(ax, corners_world(1, :), corners_world(2, :), [0.0, 0.85, 1.0], ...
                'FaceAlpha', 0.85, 'EdgeColor', [1.0, 1.0, 1.0], 'LineWidth', 2.0);

            % Ego Heading direction indicator
            head_x = ego_x + (L/2 + 1.8) * cos(ego_yaw);
            head_y = ego_y + (L/2 + 1.8) * sin(ego_yaw);
            plot(ax, [ego_x, head_x], [ego_y, head_y], 'Color', [1.0, 1.0, 1.0], 'LineWidth', 2.2);

            % Sensor FOV Arcs (Camera 65° & LiDAR 120°)
            cam_fov = deg2rad(65.0);
            cam_r = 15.0;
            cam_ang = linspace(ego_yaw - cam_fov/2, ego_yaw + cam_fov/2, 20);
            plot(ax, [ego_x, ego_x + cam_r * cos(cam_ang), ego_x], ...
                     [ego_y, ego_y + cam_r * sin(cam_ang), ego_y], ...
                     'Color', [0.0, 0.9, 0.4, 0.3], 'LineWidth', 1.0);

            ax.XLim = [view_x_min, view_x_max];
            ax.YLim = [-view_y_span, view_y_span];
            ax.XGrid = 'on';
            ax.YGrid = 'on';
            ax.GridColor = [0.35, 0.40, 0.50];
            ax.GridAlpha = 0.4;
            hold(ax, 'off');
        end

        % --- App Designer Component Construction ---
        function createComponents(app)
            % 1. Create Main UIFigure with Dark Theme
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100, 80, 1380, 860];
            app.UIFigure.Name = 'BharatDrive-AI: Autonomous Driving Telemetry & Risk Dashboard';
            app.UIFigure.Color = [0.10, 0.12, 0.16]; % Dark slate

            % 2. Main Outer Grid Layout (Header row + Body row)
            app.MainGrid = uigridlayout(app.UIFigure, [2, 1]);
            app.MainGrid.RowHeight = {65, '1x'};
            app.MainGrid.ColumnWidth = {'1x'};
            app.MainGrid.Padding = [10, 10, 10, 10];
            app.MainGrid.RowSpacing = 8;

            % -------------------------------------------------------------
            % HEADER PANEL
            % -------------------------------------------------------------
            app.HeaderPanel = uipanel(app.MainGrid);
            app.HeaderPanel.BackgroundColor = [0.15, 0.18, 0.24];
            app.HeaderPanel.BorderType = 'none';

            app.HeaderGrid = uigridlayout(app.HeaderPanel, [1, 5]);
            app.HeaderGrid.ColumnWidth = {340, '1x', 110, 360, 150};
            app.HeaderGrid.Padding = [12, 6, 12, 6];
            app.HeaderGrid.ColumnSpacing = 10;

            % Title & Subtitle
            titleContainer = uipanel(app.HeaderGrid);
            titleContainer.BackgroundColor = 'none';
            titleContainer.BorderType = 'none';
            tcGrid = uigridlayout(titleContainer, [2, 1]);
            tcGrid.Padding = [0, 0, 0, 0];
            tcGrid.RowSpacing = 2;
            tcGrid.RowHeight = {26, 18};

            app.TitleLabel = uilabel(tcGrid);
            app.TitleLabel.Text = 'BHARATDRIVE-AI DASHBOARD';
            app.TitleLabel.FontColor = [0.0, 0.9, 1.0];
            app.TitleLabel.FontSize = 18;
            app.TitleLabel.FontWeight = 'bold';

            app.SubtitleLabel = uilabel(tcGrid);
            app.SubtitleLabel.Text = 'Indian Road Adaptive Path Planning & Risk Engine';
            app.SubtitleLabel.FontColor = [0.70, 0.75, 0.85];
            app.SubtitleLabel.FontSize = 11;

            % Scenario DropDown Label & Selector
            app.ScenarioDropDownLabel = uilabel(app.HeaderGrid);
            app.ScenarioDropDownLabel.Text = 'Active Scenario:';
            app.ScenarioDropDownLabel.FontColor = [0.90, 0.92, 0.96];
            app.ScenarioDropDownLabel.FontSize = 13;
            app.ScenarioDropDownLabel.FontWeight = 'bold';
            app.ScenarioDropDownLabel.HorizontalAlignment = 'right';

            app.ScenarioDropDown = uidropdown(app.HeaderGrid);
            app.ScenarioDropDown.Items = { ...
                'Scenario 1: Village Road (Gramin Sadak)', ...
                'Scenario 2: Busy Market (Bazaar Street)', ...
                'Scenario 3: Urban Junction (Chowk)', ...
                'Scenario 4: Highway Merge (National Highway)', ...
                'Scenario 5: Cattle Crossing (Bovine Obstruction)', ...
                'Scenario 6: Mixed Hazard Master (Cow + Auto + Pothole)' ...
            };
            app.ScenarioDropDown.Value = 'Scenario 5: Cattle Crossing (Bovine Obstruction)';
            app.ScenarioDropDown.BackgroundColor = [0.22, 0.26, 0.35];
            app.ScenarioDropDown.FontColor = [1.0, 1.0, 1.0];
            app.ScenarioDropDown.FontSize = 12;
            app.ScenarioDropDown.ValueChangedFcn = createCallbackFcn(app, @ScenarioDropDownValueChanged, true);

            % FPS & SimTime Badges
            badgeContainer = uipanel(app.HeaderGrid);
            badgeContainer.BackgroundColor = 'none';
            badgeContainer.BorderType = 'none';
            bcGrid = uigridlayout(badgeContainer, [2, 1]);
            bcGrid.Padding = [0, 0, 0, 0];
            bcGrid.RowSpacing = 2;
            bcGrid.RowHeight = {22, 20};

            app.FPSBadge = uilabel(bcGrid);
            app.FPSBadge.Text = 'FPS: 20.0 (20 Hz Loop)';
            app.FPSBadge.FontColor = [0.0, 0.95, 0.5];
            app.FPSBadge.FontSize = 12;
            app.FPSBadge.FontWeight = 'bold';
            app.FPSBadge.HorizontalAlignment = 'right';

            app.SimTimeBadge = uilabel(bcGrid);
            app.SimTimeBadge.Text = 'T: 0.00s | Step: 0';
            app.SimTimeBadge.FontColor = [0.65, 0.70, 0.80];
            app.SimTimeBadge.FontSize = 11;
            app.SimTimeBadge.HorizontalAlignment = 'right';

            % -------------------------------------------------------------
            % BODY 3-COLUMN GRID
            % -------------------------------------------------------------
            app.BodyGrid = uigridlayout(app.MainGrid, [1, 3]);
            app.BodyGrid.ColumnWidth = {310, '1x', 360};
            app.BodyGrid.Padding = [0, 0, 0, 0];
            app.BodyGrid.ColumnSpacing = 8;

            % =============================================================
            % LEFT COLUMN (Speedometer, Collision Warning, Sensors)
            % =============================================================
            app.LeftPanel = uipanel(app.BodyGrid);
            app.LeftPanel.BackgroundColor = 'none';
            app.LeftPanel.BorderType = 'none';
            app.LeftGrid = uigridlayout(app.LeftPanel, [3, 1]);
            app.LeftGrid.RowHeight = {270, 190, 290};
            app.LeftGrid.Padding = [0, 0, 0, 0];
            app.LeftGrid.RowSpacing = 8;

            % 1. Vehicle Speed Card
            app.SpeedPanel = uipanel(app.LeftGrid, 'Title', 'VEHICLE SPEED', ...
                'BackgroundColor', [0.14, 0.16, 0.22], 'ForegroundColor', [0.0, 0.85, 1.0], ...
                'FontSize', 13, 'FontWeight', 'bold');
            app.SpeedGrid = uigridlayout(app.SpeedPanel, [4, 1]);
            app.SpeedGrid.RowHeight = {150, 32, 20, 20};
            app.SpeedGrid.Padding = [10, 8, 10, 8];
            app.SpeedGrid.RowSpacing = 4;

            app.SpeedGauge = uigauge(app.SpeedGrid, 'circular');
            app.SpeedGauge.Limits = [0, 100];
            app.SpeedGauge.ScaleColors = [0.0 0.8 0.4; 1.0 0.8 0.0; 1.0 0.2 0.2];
            app.SpeedGauge.ScaleColorLimits = [0 45; 45 70; 70 100];
            app.SpeedGauge.FontColor = [0.9, 0.95, 1.0];
            app.SpeedGauge.Value = 30.0;

            app.SpeedDigitalLabel = uilabel(app.SpeedGrid);
            app.SpeedDigitalLabel.Text = '30.0 km/h';
            app.SpeedDigitalLabel.FontColor = [1.0, 1.0, 1.0];
            app.SpeedDigitalLabel.FontSize = 24;
            app.SpeedDigitalLabel.FontWeight = 'bold';
            app.SpeedDigitalLabel.HorizontalAlignment = 'center';

            app.TargetSpeedLabel = uilabel(app.SpeedGrid);
            app.TargetSpeedLabel.Text = 'Target: 50.0 km/h (Nominal)';
            app.TargetSpeedLabel.FontColor = [0.70, 0.75, 0.85];
            app.TargetSpeedLabel.FontSize = 11;
            app.TargetSpeedLabel.HorizontalAlignment = 'center';

            app.AccelStateLabel = uilabel(app.SpeedGrid);
            app.AccelStateLabel.Text = 'Status: Cruising Steady';
            app.AccelStateLabel.FontColor = [0.4, 0.8, 1.0];
            app.AccelStateLabel.FontSize = 11;
            app.AccelStateLabel.FontWeight = 'bold';
            app.AccelStateLabel.HorizontalAlignment = 'center';

            % 2. Collision Warning Card
            app.WarningPanel = uipanel(app.LeftGrid, 'Title', 'COLLISION WARNING (FCW/AEB)', ...
                'BackgroundColor', [0.14, 0.16, 0.22], 'ForegroundColor', [1.0, 0.7, 0.2], ...
                'FontSize', 13, 'FontWeight', 'bold');
            app.WarningGrid = uigridlayout(app.WarningPanel, [4, 1]);
            app.WarningGrid.RowHeight = {35, 30, 24, 24};
            app.WarningGrid.Padding = [12, 10, 12, 10];
            app.WarningGrid.RowSpacing = 6;

            warnHeader = uipanel(app.WarningGrid);
            warnHeader.BackgroundColor = 'none';
            warnHeader.BorderType = 'none';
            whGrid = uigridlayout(warnHeader, [1, 2]);
            whGrid.ColumnWidth = {30, '1x'};
            whGrid.Padding = [0, 0, 0, 0];

            app.WarningLamp = uilamp(whGrid);
            app.WarningLamp.Color = [0.0, 0.9, 0.4];

            app.WarningBanner = uilabel(whGrid);
            app.WarningBanner.Text = 'SAFE: ROAD CLEAR';
            app.WarningBanner.FontColor = [0.0, 0.9, 0.4];
            app.WarningBanner.FontSize = 12;
            app.WarningBanner.FontWeight = 'bold';

            app.TTCLabel = uilabel(app.WarningGrid);
            app.TTCLabel.Text = 'TTC: Safe (Inf)';
            app.TTCLabel.FontColor = [1.0, 1.0, 1.0];
            app.TTCLabel.FontSize = 15;
            app.TTCLabel.FontWeight = 'bold';

            app.ThreatDescLabel = uilabel(app.WarningGrid);
            app.ThreatDescLabel.Text = 'Threat: Clear Corridor';
            app.ThreatDescLabel.FontColor = [0.80, 0.85, 0.95];
            app.ThreatDescLabel.FontSize = 11;

            % 3. Sensor Status Card
            app.SensorPanel = uipanel(app.LeftGrid, 'Title', 'SENSOR SUITE & HEALTH', ...
                'BackgroundColor', [0.14, 0.16, 0.22], 'ForegroundColor', [0.3, 0.8, 0.9], ...
                'FontSize', 13, 'FontWeight', 'bold');
            app.SensorGrid = uigridlayout(app.SensorPanel, [5, 2]);
            app.SensorGrid.ColumnWidth = {26, '1x'};
            app.SensorGrid.RowHeight = {30, 30, 30, 30, 35};
            app.SensorGrid.Padding = [12, 10, 12, 10];
            app.SensorGrid.RowSpacing = 6;

            app.CameraLamp = uilamp(app.SensorGrid);
            app.CameraLamp.Color = [0.0, 0.9, 0.4];
            app.CameraLabel = uilabel(app.SensorGrid);
            app.CameraLabel.Text = 'Camera: 1080p 60Hz Active';
            app.CameraLabel.FontColor = [0.85, 0.90, 0.98];
            app.CameraLabel.FontSize = 11;

            app.LidarLamp = uilamp(app.SensorGrid);
            app.LidarLamp.Color = [0.0, 0.9, 0.4];
            app.LidarLabel = uilabel(app.SensorGrid);
            app.LidarLabel.Text = '3D LiDAR: 128-Beam Active';
            app.LidarLabel.FontColor = [0.85, 0.90, 0.98];
            app.LidarLabel.FontSize = 11;

            app.RadarLamp = uilamp(app.SensorGrid);
            app.RadarLamp.Color = [0.0, 0.9, 0.4];
            app.RadarLabel = uilabel(app.SensorGrid);
            app.RadarLabel.Text = 'Doppler Radar: 77GHz Active';
            app.RadarLabel.FontColor = [0.85, 0.90, 0.98];
            app.RadarLabel.FontSize = 11;

            app.IMULamp = uilamp(app.SensorGrid);
            app.IMULamp.Color = [0.0, 0.9, 0.4];
            app.IMULabel = uilabel(app.SensorGrid);
            app.IMULabel.Text = 'RTK GNSS / IMU: Fixed 100Hz';
            app.IMULabel.FontColor = [0.85, 0.90, 0.98];
            app.IMULabel.FontSize = 11;

            % Weather label span
            app.WeatherLabel = uilabel(app.SensorGrid);
            app.WeatherLabel.Layout.Row = 5;
            app.WeatherLabel.Layout.Column = [1, 2];
            app.WeatherLabel.Text = 'Friction: μ=0.82 | Visibility: 100%';
            app.WeatherLabel.FontColor = [0.65, 0.85, 0.75];
            app.WeatherLabel.FontSize = 11;
            app.WeatherLabel.FontWeight = 'bold';

            % =============================================================
            % CENTER COLUMN (Bird's-Eye View Current Path & Playback Controls)
            % =============================================================
            app.CenterPanel = uipanel(app.BodyGrid);
            app.CenterPanel.BackgroundColor = 'none';
            app.CenterPanel.BorderType = 'none';
            app.CenterGrid = uigridlayout(app.CenterPanel, [2, 1]);
            app.CenterGrid.RowHeight = {'1x', 75};
            app.CenterGrid.Padding = [0, 0, 0, 0];
            app.CenterGrid.RowSpacing = 8;

            % BEV Trajectory Axes Card
            app.BEVCard = uipanel(app.CenterGrid, 'Title', 'CURRENT PATH & BIRDS-EYE VIEW (BEV)', ...
                'BackgroundColor', [0.12, 0.14, 0.19], 'ForegroundColor', [0.0, 0.9, 1.0], ...
                'FontSize', 13, 'FontWeight', 'bold');
            bevLayout = uigridlayout(app.BEVCard, [1, 1]);
            bevLayout.Padding = [6, 6, 6, 6];

            app.BEVAxes = uiaxes(bevLayout);
            app.BEVAxes.BackgroundColor = [0.08, 0.10, 0.14];
            app.BEVAxes.XColor = [0.70, 0.75, 0.85];
            app.BEVAxes.YColor = [0.70, 0.75, 0.85];
            app.BEVAxes.XLabel.String = 'Longitudinal Position X (meters)';
            app.BEVAxes.YLabel.String = 'Lateral Offset Y (meters)';
            app.BEVAxes.XLabel.FontColor = [0.8, 0.85, 0.95];
            app.BEVAxes.YLabel.FontColor = [0.8, 0.85, 0.95];
            app.BEVAxes.Title.String = 'Ego Trajectory, Hybrid A* Corridor, DWA Path & Obstacle OBBs';
            app.BEVAxes.Title.FontColor = [0.0, 0.85, 1.0];
            app.BEVAxes.Title.FontSize = 11;
            app.BEVAxes.DataAspectRatio = [1, 1, 1];

            % Playback Control Panel
            app.ControlPanel = uipanel(app.CenterGrid);
            app.ControlPanel.BackgroundColor = [0.14, 0.16, 0.22];
            app.ControlPanel.BorderType = 'none';
            app.ControlGrid = uigridlayout(app.ControlPanel, [1, 8]);
            app.ControlGrid.ColumnWidth = {80, 80, 80, 80, 60, 110, 130, '1x'};
            app.ControlGrid.Padding = [10, 10, 10, 10];
            app.ControlGrid.ColumnSpacing = 8;

            app.PlayButton = uibutton(app.ControlGrid, 'push', 'Text', '▶ Play', ...
                'BackgroundColor', [0.10, 0.50, 0.25], 'FontColor', [1, 1, 1], 'FontWeight', 'bold', ...
                'ButtonPushedFcn', createCallbackFcn(app, @PlayButtonPushed, true));

            app.PauseButton = uibutton(app.ControlGrid, 'push', 'Text', '⏸ Pause', ...
                'BackgroundColor', [0.18, 0.22, 0.29], 'FontColor', [1, 1, 1], 'FontWeight', 'bold', ...
                'ButtonPushedFcn', createCallbackFcn(app, @PauseButtonPushed, true));

            app.StepButton = uibutton(app.ControlGrid, 'push', 'Text', '⏭ Step', ...
                'BackgroundColor', [0.18, 0.22, 0.29], 'FontColor', [1, 1, 1], ...
                'ButtonPushedFcn', createCallbackFcn(app, @StepButtonPushed, true));

            app.ResetButton = uibutton(app.ControlGrid, 'push', 'Text', '↺ Reset', ...
                'BackgroundColor', [0.18, 0.22, 0.29], 'FontColor', [1, 1, 1], ...
                'ButtonPushedFcn', createCallbackFcn(app, @ResetButtonPushed, true));

            app.SpeedLabel = uilabel(app.ControlGrid, 'Text', 'Speed:', ...
                'FontColor', [0.85, 0.90, 0.95], 'FontSize', 12, 'HorizontalAlignment', 'right');

            app.SpeedMultiplierDropDown = uidropdown(app.ControlGrid);
            app.SpeedMultiplierDropDown.Items = {'0.5x', '1.0x (Real-Time)', '2.0x', '5.0x'};
            app.SpeedMultiplierDropDown.Value = '1.0x (Real-Time)';
            app.SpeedMultiplierDropDown.BackgroundColor = [0.22, 0.26, 0.35];
            app.SpeedMultiplierDropDown.FontColor = [1, 1, 1];
            app.SpeedMultiplierDropDown.ValueChangedFcn = createCallbackFcn(app, @SpeedMultiplierValueChanged, true);

            app.EmergencyStopBtn = uibutton(app.ControlGrid, 'push', 'Text', '⚠ MANUAL AEB', ...
                'BackgroundColor', [0.70, 0.15, 0.15], 'FontColor', [1, 1, 1], 'FontWeight', 'bold', ...
                'ButtonPushedFcn', createCallbackFcn(app, @EmergencyStopBtnPushed, true));

            app.StateflowBadge = uilabel(app.ControlGrid);
            app.StateflowBadge.Text = 'State: Accelerate';
            app.StateflowBadge.FontColor = [0.0, 0.9, 1.0];
            app.StateflowBadge.FontSize = 13;
            app.StateflowBadge.FontWeight = 'bold';
            app.StateflowBadge.HorizontalAlignment = 'center';

            % =============================================================
            % RIGHT COLUMN (Risk Assessment Engine & Detected Objects Table)
            % =============================================================
            app.RightPanel = uipanel(app.BodyGrid);
            app.RightPanel.BackgroundColor = 'none';
            app.RightPanel.BorderType = 'none';
            app.RightGrid = uigridlayout(app.RightPanel, [2, 1]);
            app.RightGrid.RowHeight = {360, '1x'};
            app.RightGrid.Padding = [0, 0, 0, 0];
            app.RightGrid.RowSpacing = 8;

            % 1. Risk Assessment Card
            app.RiskCard = uipanel(app.RightGrid, 'Title', 'CURRENT RISK: 7-FACTOR IRRM', ...
                'BackgroundColor', [0.14, 0.16, 0.22], 'ForegroundColor', [1.0, 0.4, 0.2], ...
                'FontSize', 13, 'FontWeight', 'bold');
            app.RiskGrid = uigridlayout(app.RiskCard, [4, 1]);
            app.RiskGrid.RowHeight = {35, 30, 24, '1x'};
            app.RiskGrid.Padding = [12, 8, 12, 8];
            app.RiskGrid.RowSpacing = 4;

            riskScoreHeader = uipanel(app.RiskGrid);
            riskScoreHeader.BackgroundColor = 'none';
            riskScoreHeader.BorderType = 'none';
            rshGrid = uigridlayout(riskScoreHeader, [1, 2]);
            rshGrid.ColumnWidth = {110, '1x'};
            rshGrid.Padding = [0, 0, 0, 0];

            app.RiskScoreLabel = uilabel(rshGrid);
            app.RiskScoreLabel.Text = '0%';
            app.RiskScoreLabel.FontColor = [0.0, 0.9, 0.4];
            app.RiskScoreLabel.FontSize = 26;
            app.RiskScoreLabel.FontWeight = 'bold';

            app.RiskLevelBadge = uilabel(rshGrid);
            app.RiskLevelBadge.Text = 'Criticality: LOW';
            app.RiskLevelBadge.FontColor = [0.0, 0.9, 0.4];
            app.RiskLevelBadge.FontSize = 14;
            app.RiskLevelBadge.FontWeight = 'bold';
            app.RiskLevelBadge.HorizontalAlignment = 'right';

            app.RiskGauge = uigauge(app.RiskGrid, 'linear');
            app.RiskGauge.Limits = [0, 1.0];
            app.RiskGauge.MajorTicks = [0, 0.35, 0.65, 0.85, 1.0];
            app.RiskGauge.ScaleColors = [0.0 0.8 0.4; 1.0 0.85 0.0; 1.0 0.5 0.0; 1.0 0.2 0.2];
            app.RiskGauge.ScaleColorLimits = [0 0.35; 0.35 0.65; 0.65 0.85; 0.85 1.0];
            app.RiskGauge.FontColor = [0.85, 0.9, 1.0];
            app.RiskGauge.Value = 0.0;

            app.RiskActionLabel = uilabel(app.RiskGrid);
            app.RiskActionLabel.Text = 'Action: MAINTAIN CRUISE';
            app.RiskActionLabel.FontColor = [0.80, 0.85, 0.95];
            app.RiskActionLabel.FontSize = 12;
            app.RiskActionLabel.FontWeight = 'bold';

            % Factor breakdown bar axes
            app.RiskFactorAxes = uiaxes(app.RiskGrid);
            app.RiskFactorAxes.BackgroundColor = [0.09, 0.11, 0.16];
            app.RiskFactorAxes.XColor = [0.70, 0.75, 0.85];
            app.RiskFactorAxes.YColor = [0.70, 0.75, 0.85];
            app.RiskFactorAxes.Title.String = '7-Factor IRRM Risk Contribution Breakdown';
            app.RiskFactorAxes.Title.FontColor = [0.90, 0.95, 1.0];
            app.RiskFactorAxes.Title.FontSize = 10;
            app.RiskFactorAxes.FontSize = 8;

            % 2. Detected Objects Table Card
            app.ObjectsCard = uipanel(app.RightGrid, 'Title', 'DETECTED OBJECTS & FUSION TRACKS', ...
                'BackgroundColor', [0.14, 0.16, 0.22], 'ForegroundColor', [0.0, 0.85, 1.0], ...
                'FontSize', 13, 'FontWeight', 'bold');
            objLayout = uigridlayout(app.ObjectsCard, [1, 1]);
            objLayout.Padding = [6, 6, 6, 6];

            app.ObjectsTable = uitable(objLayout);
            app.ObjectsTable.ColumnName = {'ID', 'Class', 'Range', 'VRel', 'Conf', 'Sensors', 'Risk'};
            app.ObjectsTable.ColumnWidth = {38, 86, 56, 56, 44, 76, 44};
            app.ObjectsTable.BackgroundColor = [0.11, 0.13, 0.18];
            app.ObjectsTable.FontColor = [0.9, 0.95, 1.0];
            app.ObjectsTable.FontSize = 10;

            % -------------------------------------------------------------
            % TIMER INITIALIZATION (20 Hz Master Telemetry Loop)
            % -------------------------------------------------------------
            app.SimTimer = timer( ...
                'ExecutionMode', 'fixedRate', ...
                'Period', app.DeltaT, ...
                'TimerFcn', createCallbackFcn(app, @onTimerTick, true), ...
                'ErrorFcn', @(~,~) disp('Dashboard Timer Error') ...
            );

            % Finalize and show figure
            app.loadScenario(app.ActiveScenarioID);
            app.updateUI();
            app.UIFigure.Visible = 'on';
        end
    end

    % Public App Lifecycle Methods
    methods (Access = public)

        % Construct app
        function app = BharatDriveDashboard()
            % Create and configure components
            createComponents(app);

            % Register the app with App Designer
            registerApp(app, app.UIFigure);

            if nargout == 0
                clear app;
            end
        end

        % Code that executes before app deletion
        function delete(app)
            % Stop and delete simulation timer
            if ~isempty(app.SimTimer) && isvalid(app.SimTimer)
                stop(app.SimTimer);
                delete(app.SimTimer);
            end

            % Delete UIFigure
            if ~isempty(app.UIFigure) && isvalid(app.UIFigure)
                delete(app.UIFigure);
            end
        end
    end
end
