classdef StateflowDecisionSupervisor < handle
    % STATEFLOWDECISIONSUPERVISOR Tactical Behavioral Decision Engine for Autonomous Driving.
    % BharatDrive-AI - MathWorks Autonomous Driving Hackathon.
    %
    % Implements Stateflow decision logic deciding among 8 discrete states:
    %   1. Accelerate
    %   2. Brake
    %   3. Overtake
    %   4. Wait
    %   5. Stop
    %   6. Emergency Stop
    %   7. Move Left
    %   8. Move Right

    properties
        CurrentState            % Current active ManeuverState enum
        PreviousState           % State in prior time step
        StateTime               % Time spent continuously in current state (s)
        OvertakePhase           % 0: None, 1: ChangeRight, 2: Passing, 3: CutBack
        OvertakeLeadId          % ID of obstacle currently being overtaken
        NominalCruiseSpeed      % Free-flow target speed (m/s) [13.89 m/s = 50 km/h]
        MinFollowDistance       % Minimum desired headway distance (m) [15.0 m]
        CriticalDistance        % Minimum emergency clearance threshold (m) [3.8 m]
        HysteresisTime          % Minimum time required before voluntary state transition (s) [0.4 s]
        HornActive              % Acoustic alert for cattle/pedestrians (Boolean)
        HazardLightsActive      % Hazard blinkers for emergency stop / wait (Boolean)
    end

    methods
        function obj = StateflowDecisionSupervisor(cruise_speed)
            if nargin < 1, cruise_speed = 13.89; end
            obj.NominalCruiseSpeed = cruise_speed;
            obj.CurrentState = bharatdrive.decision.ManeuverState.Accelerate;
            obj.PreviousState = bharatdrive.decision.ManeuverState.Accelerate;
            obj.StateTime = 0.0;
            obj.OvertakePhase = 0;
            obj.OvertakeLeadId = -1;
            obj.MinFollowDistance = 15.0;
            obj.CriticalDistance = 3.8;
            obj.HysteresisTime = 0.4;
            obj.HornActive = false;
            obj.HazardLightsActive = false;
        end

        function decision = step(obj, ego_state, threat_obs, env_context, dt)
            % STEP Executes a single Stateflow transition evaluation cycle (10 Hz).
            %
            % ego_state: struct or array [x, y, v, yaw, yaw_rate, accel, steer]
            % threat_obs: struct with primary obstacle information:
            %             .Distance, .VRel, .Position [x,y], .Class, .RiskScore, .TTC
            % env_context: struct with:
            %              .RoadWidth, .LeftClearance, .RightClearance, .OncomingDist, .SignalState
            % dt: time step in seconds [default 0.1s]

            if nargin < 5, dt = 0.1; end

            ego_x = ego_state(1);
            ego_y = ego_state(2);
            ego_v = ego_state(3);

            % Extract threat obstacle attributes
            if isempty(threat_obs)
                obs_d = 100.0;
                obs_vrel = 0.0;
                obs_pos = [ego_x + 100.0, 0.0];
                obs_class = 'None';
                obs_risk = 0.0;
                obs_ttc = inf;
            else
                obs_d = threat_obs.Distance;
                obs_vrel = threat_obs.VRel;
                obs_pos = threat_obs.Position;
                obs_class = threat_obs.Class;
                obs_risk = threat_obs.RiskScore;
                obs_ttc = threat_obs.TTC;
            end

            % Extract environment context
            if isempty(env_context)
                road_w = 7.0;
                left_clear = 2.5;
                right_clear = 2.5;
                oncoming_d = 150.0;
                signal = 'None';
            else
                road_w = env_context.RoadWidth;
                left_clear = env_context.LeftClearance;
                right_clear = env_context.RightClearance;
                oncoming_d = env_context.OncomingDist;
                signal = env_context.SignalState;
            end

            % Update state duration timer
            obj.StateTime = obj.StateTime + dt;
            next_state = obj.CurrentState;
            obj.HornActive = false;
            obj.HazardLightsActive = false;

            % -----------------------------------------------------------------
            % 1. GLOBAL SUPERVISOR: EMERGENCY STOP (Highest Priority Preemption)
            % -----------------------------------------------------------------
            is_emergency = (obs_ttc > 0 && obs_ttc <= 1.1) || ...
                           (obs_risk >= 0.88) || ...
                           (obs_d <= obj.CriticalDistance && ego_v > 0.5 && abs(obs_pos(2) - ego_y) < 1.4);

            if is_emergency
                next_state = bharatdrive.decision.ManeuverState.EmergencyStop;
            elseif obj.CurrentState == bharatdrive.decision.ManeuverState.EmergencyStop
                % Exit emergency stop once ego is stopped and critical threat clears
                if ego_v < 0.2 && obs_risk < 0.55
                    if obs_d < 10.0
                        next_state = bharatdrive.decision.ManeuverState.Wait;
                    else
                        next_state = bharatdrive.decision.ManeuverState.Accelerate;
                    end
                else
                    next_state = bharatdrive.decision.ManeuverState.EmergencyStop;
                end

            % -----------------------------------------------------------------
            % 2. CONTROLLED STOP (Traffic Signal / Stop Line / Roadblock)
            % -----------------------------------------------------------------
            elseif strcmp(signal, 'Red') || strcmp(signal, 'Stop') || strcmp(obs_class, 'Roadblock')
                next_state = bharatdrive.decision.ManeuverState.Stop;

            elseif obj.CurrentState == bharatdrive.decision.ManeuverState.Stop
                if strcmp(signal, 'Green') || strcmp(signal, 'None')
                    next_state = bharatdrive.decision.ManeuverState.Accelerate;
                end

            % -----------------------------------------------------------------
            % 3. WAIT (Crossing Cattle, Pedestrian on Zebra, or Blocked Corridor)
            % -----------------------------------------------------------------
            elseif (strcmp(obs_class, 'Cattle') || strcmp(obs_class, 'Pedestrian')) && ...
                   (obs_d < 12.0) && (ego_v < 2.5 || obs_d < 6.0) && abs(obs_pos(2) - ego_y) < 1.6
                next_state = bharatdrive.decision.ManeuverState.Wait;
                if obj.StateTime > 1.5
                    obj.HornActive = true; % Sound courteous horn for cattle clearing
                end

            elseif obj.CurrentState == bharatdrive.decision.ManeuverState.Wait
                % Wait until animal or pedestrian clears road corridor
                if obs_d > 14.0 || abs(obs_pos(2) - ego_y) > 2.0 || strcmp(obs_class, 'None')
                    next_state = bharatdrive.decision.ManeuverState.Accelerate;
                else
                    next_state = bharatdrive.decision.ManeuverState.Wait;
                    if obj.StateTime > 1.5
                        obj.HornActive = true;
                    end
                end

            % -----------------------------------------------------------------
            % 4. OVERTAKE (Multi-Phase Overtake of Slow Vehicle)
            % -----------------------------------------------------------------
            elseif obj.CurrentState == bharatdrive.decision.ManeuverState.Overtake
                next_state = obj.evaluate_overtake_phases(ego_x, ego_v, obs_pos, obs_vrel, oncoming_d);

            elseif (obj.StateTime >= obj.HysteresisTime) && ...
                   (strcmp(obs_class, 'Auto-Rickshaw') || strcmp(obs_class, 'Cycle') || strcmp(obs_class, 'Truck-Bus')) && ...
                   (obs_d >= 10.0 && obs_d <= 25.0) && ...
                   (obs_vrel > 2.0 || (ego_v - obs_vrel) < 0.6 * obj.NominalCruiseSpeed) && ...
                   (road_w >= 6.5) && (oncoming_d >= 75.0) && (right_clear >= 2.2)

                % Safe overtaking conditions met
                next_state = bharatdrive.decision.ManeuverState.Overtake;
                obj.OvertakePhase = 1; % Start Lane Change Right

            % -----------------------------------------------------------------
            % 5. MOVE LEFT (Nudge Left to Avoid Center/Right Obstacle or Oncoming Traffic)
            % -----------------------------------------------------------------
            elseif (obj.StateTime >= obj.HysteresisTime) && ...
                   (obs_d < 30.0) && (obs_pos(2) >= -0.2) && (left_clear >= 1.6) && ...
                   (strcmp(obs_class, 'Cattle') || strcmp(obs_class, 'Truck-Bus') || obs_pos(2) > 0.0)

                next_state = bharatdrive.decision.ManeuverState.MoveLeft;

            elseif obj.CurrentState == bharatdrive.decision.ManeuverState.MoveLeft
                % Stay in MoveLeft until obstruction is behind vehicle
                if (obs_pos(1) < ego_x - 3.0) || (obs_d > 35.0)
                    next_state = bharatdrive.decision.ManeuverState.Accelerate;
                end

            % -----------------------------------------------------------------
            % 6. MOVE RIGHT (Nudge Right to Straddle Left Pothole or Pass Push-Cart)
            % -----------------------------------------------------------------
            elseif (obj.StateTime >= obj.HysteresisTime) && ...
                   (obs_d < 30.0) && (obs_pos(2) <= 0.2) && (right_clear >= 1.6) && (oncoming_d >= 45.0) && ...
                   (strcmp(obs_class, 'Pothole') || strcmp(obs_class, 'Push-Cart') || strcmp(obs_class, 'Pedestrian'))

                next_state = bharatdrive.decision.ManeuverState.MoveRight;

            elseif obj.CurrentState == bharatdrive.decision.ManeuverState.MoveRight
                if (obs_pos(1) < ego_x - 3.0) || (obs_d > 35.0)
                    next_state = bharatdrive.decision.ManeuverState.Accelerate;
                end

            % -----------------------------------------------------------------
            % 7. BRAKE (Lead Vehicle Slower or Elevated Threat Ahead)
            % -----------------------------------------------------------------
            elseif (obs_d < 24.0 && obs_vrel > 0.5) || (obs_risk >= 0.42 && obs_risk < 0.88)
                next_state = bharatdrive.decision.ManeuverState.Brake;

            elseif obj.CurrentState == bharatdrive.decision.ManeuverState.Brake
                if obs_d >= 28.0 && obs_risk < 0.35
                    next_state = bharatdrive.decision.ManeuverState.Accelerate;
                elseif ego_v < 0.5 && obs_d < 8.0
                    next_state = bharatdrive.decision.ManeuverState.Wait;
                end

            % -----------------------------------------------------------------
            % 8. ACCELERATE (Default Nominal Driving State)
            % -----------------------------------------------------------------
            else
                next_state = bharatdrive.decision.ManeuverState.Accelerate;
            end

            % State Transition Handling
            if next_state ~= obj.CurrentState
                obj.PreviousState = obj.CurrentState;
                obj.CurrentState = next_state;
                obj.StateTime = 0.0;
                if next_state ~= bharatdrive.decision.ManeuverState.Overtake
                    obj.OvertakePhase = 0;
                end
            end

            % Compute Command Set based on Active State
            decision = obj.generate_state_commands(ego_v, obs_d, obs_vrel, obs_risk);
        end
    end

    methods (Access = private)
        function next_state = evaluate_overtake_phases(obj, ego_x, ego_v, obs_pos, obs_vrel, oncoming_d)
            % Multi-phase sequential Stateflow subchart for Overtaking
            lead_x = obs_pos(1);

            % Abort condition: oncoming vehicle closes in rapidly
            if oncoming_d < 45.0 && obj.OvertakePhase < 3
                next_state = bharatdrive.decision.ManeuverState.Brake;
                obj.OvertakePhase = 0;
                return;
            end

            switch obj.OvertakePhase
                case 1 % Phase 1: Lane Change Right
                    if obj.StateTime >= 1.2 || (ego_x >= lead_x - 8.0)
                        obj.OvertakePhase = 2; % Transition to Passing
                    end
                    next_state = bharatdrive.decision.ManeuverState.Overtake;

                case 2 % Phase 2: Passing Lead Vehicle
                    if ego_x > (lead_x + 6.0) % Cleared lead vehicle with safe bumper margin
                        obj.OvertakePhase = 3; % Transition to Cut-Back Left
                    end
                    next_state = bharatdrive.decision.ManeuverState.Overtake;

                case 3 % Phase 3: Cut Back into Travel Lane
                    if obj.StateTime >= 1.5 || (ego_x > lead_x + 14.0)
                        obj.OvertakePhase = 0;
                        next_state = bharatdrive.decision.ManeuverState.Accelerate; % Completed!
                    else
                        next_state = bharatdrive.decision.ManeuverState.Overtake;
                    end

                otherwise
                    obj.OvertakePhase = 0;
                    next_state = bharatdrive.decision.ManeuverState.Accelerate;
            end
        end

        function cmd = generate_state_commands(obj, ego_v, obs_d, obs_vrel, obs_risk)
            % Translates active Stateflow state into physical actuator references
            cmd.State = obj.CurrentState;
            cmd.StateName = bharatdrive.decision.ManeuverState.get_name(obj.CurrentState);
            cmd.OvertakePhase = obj.OvertakePhase;
            cmd.Horn = obj.HornActive;
            cmd.HazardLights = obj.HazardLightsActive;

            switch obj.CurrentState
                case bharatdrive.decision.ManeuverState.EmergencyStop
                    cmd.TargetSpeed = 0.0;
                    cmd.TargetAccel = -6.50; % Max emergency braking
                    cmd.TargetLateralOffset = 0.0;
                    cmd.HazardLights = true;

                case bharatdrive.decision.ManeuverState.Stop
                    cmd.TargetSpeed = 0.0;
                    cmd.TargetAccel = -2.50; % Smooth service braking
                    cmd.TargetLateralOffset = 0.0;

                case bharatdrive.decision.ManeuverState.Wait
                    cmd.TargetSpeed = 0.0;
                    cmd.TargetAccel = -1.50;
                    cmd.TargetLateralOffset = 0.0;
                    cmd.HazardLights = true;

                case bharatdrive.decision.ManeuverState.Brake
                    lead_speed = max(0.0, ego_v - obs_vrel);
                    cmd.TargetSpeed = max(0.0, lead_speed * 0.90);
                    cmd.TargetAccel = max(-4.50, (lead_speed - ego_v) / 1.20);
                    cmd.TargetLateralOffset = 0.0;

                case bharatdrive.decision.ManeuverState.Accelerate
                    cmd.TargetSpeed = obj.NominalCruiseSpeed;
                    cmd.TargetAccel = min(2.50, (obj.NominalCruiseSpeed - ego_v) / 2.0);
                    cmd.TargetLateralOffset = 0.0;

                case bharatdrive.decision.ManeuverState.MoveLeft
                    cmd.TargetSpeed = min(ego_v, obj.NominalCruiseSpeed * 0.85);
                    cmd.TargetAccel = 0.0;
                    cmd.TargetLateralOffset = -1.25; % Lateral shift left

                case bharatdrive.decision.ManeuverState.MoveRight
                    cmd.TargetSpeed = min(ego_v, obj.NominalCruiseSpeed * 0.85);
                    cmd.TargetAccel = 0.0;
                    cmd.TargetLateralOffset = +1.25; % Lateral shift right

                case bharatdrive.decision.ManeuverState.Overtake
                    if obj.OvertakePhase == 1
                        cmd.TargetSpeed = min(obj.NominalCruiseSpeed, ego_v + 1.0);
                        cmd.TargetAccel = 1.0;
                        cmd.TargetLateralOffset = +1.80; % Shift right into passing corridor
                    elseif obj.OvertakePhase == 2
                        cmd.TargetSpeed = min(obj.NominalCruiseSpeed * 1.15, ego_v + 2.5);
                        cmd.TargetAccel = 2.0; % Accelerated pass
                        cmd.TargetLateralOffset = +1.80;
                    elseif obj.OvertakePhase == 3
                        cmd.TargetSpeed = obj.NominalCruiseSpeed;
                        cmd.TargetAccel = 0.0;
                        cmd.TargetLateralOffset = 0.0; % Return to center
                    else
                        cmd.TargetSpeed = obj.NominalCruiseSpeed;
                        cmd.TargetAccel = 0.0;
                        cmd.TargetLateralOffset = 0.0;
                    end
            end
        end
    end
end
