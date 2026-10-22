function [threat_obstacle, overall_risk_score] = block_dynamic_risk_assessment(fused_tracks, predicted_trajectories, ego_bus, env_bus)
%#codegen
% BLOCK_DYNAMIC_RISK_ASSESSMENT 7-Factor Indian Road Risk Metric (IRRM) Engine.
% Evaluates dynamic threat levels across distance, speed, direction, class, road width, and weather.
% BharatDrive-AI - MathWorks Autonomous Driving Hackathon.
%
% Output:
%   threat_obstacle: 1x7 array [Distance, VRel, PosX, PosY, ClassID, RiskScore, TTC]
%   overall_risk_score: double in [0, 1]

max_risk = 0.0;
best_obs_idx = 1;
threat_obstacle = [100.0, 0.0, 100.0, 0.0, 0.0, 0.0, 999.0];

ego_v = ego_bus.v;
road_w = env_bus.RoadWidth;
mu = env_bus.FrictionMu;

% Contextual scaling factors
weather_factor = 1.0 + (0.85 - mu) * 0.60;
width_factor = max(1.0, 7.0 / max(3.5, road_w));

for i = 1:5
    dist = fused_tracks(i, 7);
    if dist < 0.1 || dist > 80.0
        continue;
    end

    x = fused_tracks(i, 1);
    y = fused_tracks(i, 2);
    vx = fused_tracks(i, 3);
    class_id = int32(fused_tracks(i, 5));

    % Relative speed along longitudinal axis
    v_rel = max(0.0, ego_v - vx);

    % Time-to-Collision (TTC)
    if v_rel > 0.5 && x > 0.0
        ttc = x / v_rel;
    else
        ttc = 999.0;
    end

    % Class vulnerability & unpredictability weight
    switch class_id
        case 7  % Cattle (High unpredictability, sudden jumps)
            m_type = 1.35;
        case 9  % Pedestrian
            m_type = 1.45;
        case 5  % Auto-Rickshaw (Erratic lane weave)
            m_type = 1.25;
        case 11 % Pothole (Structural impact hazard)
            m_type = 1.20;
        case 2  % Heavy Bus / Truck
            m_type = 1.30;
        otherwise
            m_type = 1.00;
    end

    % Directional approach multiplier
    if abs(y) < 1.2
        phi_dir = 1.30; % Directly in ego travel lane
    else
        phi_dir = 0.85; % Shoulder or adjacent lane
    end

    % Proximity risk
    r_dist = exp(-dist / 18.0);
    r_speed = min(1.0, v_rel / 20.0);
    r_ttc = min(1.0, 3.0 / max(0.5, ttc));

    base_risk = 0.45 * r_dist + 0.30 * r_ttc + 0.25 * r_speed;
    risk_score = min(1.0, base_risk * m_type * phi_dir * width_factor * weather_factor);

    if risk_score > max_risk
        max_risk = risk_score;
        best_obs_idx = i;
        threat_obstacle = [dist, v_rel, x, y, double(class_id), risk_score, ttc];
    end
end

overall_risk_score = max_risk;

end
