function [state_id, target_speed, target_accel, target_lateral_offset, overtake_phase, horn, hazard_lights] = ...
    block_stateflow_decision_logic(ego_bus, env_bus, threat_obs, env_signals)
%#codegen
% BLOCK_STATEFLOW_DECISION_LOGIC Discrete Stateflow Decision Logic for Autonomous Driving.
% BharatDrive-AI - MathWorks Autonomous Driving Hackathon.
%
% Inputs:
%   ego_bus: BusVehicleState [x, y, v, yaw, yaw_rate, accel, steer]
%   env_bus: BusRiskContext [RoadWidth, FrictionMu, Visibility, TrafficDensity]
%   threat_obs: 1x7 array [Distance, VRel, PosX, PosY, ClassID, RiskScore, TTC]
%   env_signals: 1x4 array [LeftClearance, RightClearance, OncomingDist, SignalState (0:None, 1:Green, 2:Yellow, 3:Red)]
%
% Outputs:
%   state_id: int32 (1:Acc, 2:Brake, 3:Overtake, 4:Wait, 5:Stop, 6:EmergencyStop, 7:MoveLeft, 8:MoveRight)
%   target_speed: double (m/s)
%   target_accel: double (m/s^2)
%   target_lateral_offset: double (m)
%   overtake_phase: int32 (0:None, 1:ChangeRight, 2:Passing, 3:CutBack)
%   horn: boolean
%   hazard_lights: boolean

persistent current_state state_time active_overtake_phase
if isempty(current_state)
    current_state = int32(1); % ManeuverState.Accelerate
    state_time = 0.0;
    active_overtake_phase = int32(0);
end

dt = 0.1; % 10 Hz Stateflow sampling step
state_time = state_time + dt;

ego_x = ego_bus.x;
ego_y = ego_bus.y;
ego_v = ego_bus.v;

if numel(threat_obs) >= 7
    obs_d     = threat_obs(1);
    obs_vrel  = threat_obs(2);
    obs_x     = threat_obs(3);
    obs_y     = threat_obs(4);
    obs_class = int32(threat_obs(5));
    obs_risk  = threat_obs(6);
    obs_ttc   = threat_obs(7);
else
    obs_d     = 100.0;
    obs_vrel  = 0.0;
    obs_x     = 100.0;
    obs_y     = 0.0;
    obs_class = int32(0);
    obs_risk  = 0.0;
    obs_ttc   = 99.0;
end

if numel(env_signals) >= 4
    left_clear  = env_signals(1);
    right_clear = env_signals(2);
    oncoming_d  = env_signals(3);
    sig_state   = int32(env_signals(4)); % 3 = Red
else
    left_clear  = 3.5;
    right_clear = 3.5;
    oncoming_d  = 100.0;
    sig_state   = int32(0);
end

next_state = current_state;
horn_cmd = false;
hazard_cmd = false;

% -------------------------------------------------------------------------
% STATEFLOW TRANSITION GUARDS (Evaluated Hierarchically)
% -------------------------------------------------------------------------

% 1. Global Preemption: Emergency Stop (TTC <= 1.1s, Risk >= 0.88, or critical distance)
is_emergency = (obs_ttc > 0.0 && obs_ttc <= 1.1) || ...
               (obs_risk >= 0.88) || ...
               (obs_d <= 3.8 && ego_v > 0.5 && abs(obs_y - ego_y) < 1.4);

if is_emergency
    next_state = int32(6); % EmergencyStop

elseif current_state == int32(6) % In EmergencyStop
    if ego_v < 0.2 && obs_risk < 0.55
        if obs_d < 10.0
            next_state = int32(4); % Transition to Wait
        else
            next_state = int32(1); % Accelerate
        end
    else
        next_state = int32(6);
    end

% 2. Controlled Stop (Red Light or Blockage)
elseif sig_state == 3
    next_state = int32(5); % Stop

elseif current_state == int32(5)
    if sig_state ~= 3
        next_state = int32(1); % Accelerate
    end

% 3. Wait (Cattle=7 or Pedestrian=9 crossing roadway within 12m)
elseif (obs_class == 7 || obs_class == 9) && ...
       (obs_d < 12.0) && (ego_v < 2.5 || obs_d < 6.0) && abs(obs_y - ego_y) < 1.6
    next_state = int32(4); % Wait
    if state_time > 1.5
        horn_cmd = true;
    end

elseif current_state == int32(4)
    if obs_d > 14.0 || abs(obs_y - ego_y) > 2.0
        next_state = int32(1); % Accelerate
    else
        next_state = int32(4);
        if state_time > 1.5
            horn_cmd = true;
        end
    end

% 4. Overtake (Sequential Subchart for Slow Lead Vehicle)
elseif current_state == int32(3)
    % Abort if oncoming vehicle suddenly appears
    if oncoming_d < 45.0 && active_overtake_phase < 3
        next_state = int32(2); % Abort to Brake
        active_overtake_phase = int32(0);
    else
        if active_overtake_phase == 1
            if state_time >= 1.2 || (ego_x >= obs_x - 8.0)
                active_overtake_phase = int32(2); % Passing
            end
            next_state = int32(3);
        elseif active_overtake_phase == 2
            if ego_x > (obs_x + 6.0)
                active_overtake_phase = int32(3); % Cut-back
            end
            next_state = int32(3);
        elseif active_overtake_phase == 3
            if state_time >= 1.5 || (ego_x > obs_x + 14.0)
                active_overtake_phase = int32(0);
                next_state = int32(1); % Completed!
            else
                next_state = int32(3);
            end
        else
            active_overtake_phase = int32(0);
            next_state = int32(1);
        end
    end

elseif (state_time >= 0.4) && ...
       (obs_class == 5 || obs_class == 6 || obs_class == 2 || obs_class == 3) && ...
       (obs_d >= 10.0 && obs_d <= 25.0) && ...
       (obs_vrel > 2.0 || (ego_v - obs_vrel) < 8.33) && ...
       (env_bus.RoadWidth >= 6.5) && (oncoming_d >= 75.0) && (right_clear >= 2.2)

    next_state = int32(3); % Overtake
    active_overtake_phase = int32(1);

% 5. Move Left (Obstruction on Center/Right: Cattle=7, Truck=2, etc.)
elseif (state_time >= 0.4) && ...
       (obs_d < 30.0) && (obs_y >= -0.2) && (left_clear >= 1.6) && ...
       (obs_class == 7 || obs_class == 2 || obs_class == 3 || obs_y > 0.0)

    next_state = int32(7); % MoveLeft

elseif current_state == int32(7)
    if (obs_x < ego_x - 3.0) || (obs_d > 35.0)
        next_state = int32(1); % Accelerate
    end

% 6. Move Right (Obstruction on Left: Pothole=11, Cart=10, Pedestrian=9)
elseif (state_time >= 0.4) && ...
       (obs_d < 30.0) && (obs_y <= 0.2) && (right_clear >= 1.6) && (oncoming_d >= 45.0) && ...
       (obs_class == 11 || obs_class == 10 || obs_class == 9)

    next_state = int32(8); % MoveRight

elseif current_state == int32(8)
    if (obs_x < ego_x - 3.0) || (obs_d > 35.0)
        next_state = int32(1); % Accelerate
    end

% 7. Brake (Decelerate behind slower lead or elevated risk)
elseif (obs_d < 24.0 && obs_vrel > 0.5) || (obs_risk >= 0.42 && obs_risk < 0.88)
    next_state = int32(2); % Brake

elseif current_state == int32(2)
    if obs_d >= 28.0 && obs_risk < 0.35
        next_state = int32(1);
    elseif ego_v < 0.5 && obs_d < 8.0
        next_state = int32(4); % Wait
    end

% 8. Accelerate (Default Free-Flow State)
else
    next_state = int32(1);
end

% State transition update
if next_state ~= current_state
    current_state = next_state;
    state_time = 0.0;
    if next_state ~= int32(3)
        active_overtake_phase = int32(0);
    end
end

% -------------------------------------------------------------------------
% STATE-TO-COMMAND GENERATION
% -------------------------------------------------------------------------
cruise_speed = 13.89; % 50 km/h nominal
state_id = current_state;
overtake_phase = active_overtake_phase;
horn = horn_cmd;
hazard_lights = hazard_cmd;

switch current_state
    case 6 % EmergencyStop
        target_speed = 0.0;
        target_accel = -6.50;
        target_lateral_offset = 0.0;
        hazard_lights = true;

    case 5 % Stop
        target_speed = 0.0;
        target_accel = -2.50;
        target_lateral_offset = 0.0;

    case 4 % Wait
        target_speed = 0.0;
        target_accel = -1.50;
        target_lateral_offset = 0.0;
        hazard_lights = true;

    case 2 % Brake
        lead_speed = max(0.0, ego_v - obs_vrel);
        target_speed = max(0.0, lead_speed * 0.90);
        target_accel = max(-4.50, (lead_speed - ego_v) / 1.20);
        target_lateral_offset = 0.0;

    case 1 % Accelerate
        target_speed = cruise_speed;
        target_accel = min(2.50, (cruise_speed - ego_v) / 2.0);
        target_lateral_offset = 0.0;

    case 7 % MoveLeft
        target_speed = min(ego_v, cruise_speed * 0.85);
        target_accel = 0.0;
        target_lateral_offset = -1.25;

    case 8 % MoveRight
        target_speed = min(ego_v, cruise_speed * 0.85);
        target_accel = 0.0;
        target_lateral_offset = +1.25;

    case 3 % Overtake
        if active_overtake_phase == 1
            target_speed = min(cruise_speed, ego_v + 1.0);
            target_accel = 1.0;
            target_lateral_offset = +1.80;
        elseif active_overtake_phase == 2
            target_speed = min(cruise_speed * 1.15, ego_v + 2.5);
            target_accel = 2.0;
            target_lateral_offset = +1.80;
        elseif active_overtake_phase == 3
            target_speed = cruise_speed;
            target_accel = 0.0;
            target_lateral_offset = 0.0;
        else
            target_speed = cruise_speed;
            target_accel = 0.0;
            target_lateral_offset = 0.0;
        end

    otherwise
        target_speed = cruise_speed;
        target_accel = 0.0;
        target_lateral_offset = 0.0;
end

end
