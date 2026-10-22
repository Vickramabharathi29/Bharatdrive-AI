function fused_tracks = block_yolo_fusion_detection(camera_detections, lidar_clusters, radar_targets)
%#codegen
% BLOCK_YOLO_FUSION_DETECTION YOLOv11 11-Class Detection & Multi-Sensor Fusion.
% Fuses Camera (Classification), LiDAR (Centroid XYZ), and Radar (Doppler Velocity).
% BharatDrive-AI - MathWorks Autonomous Driving Hackathon.
%
% Output:
%   fused_tracks: 5x7 array [x_rel, y_rel, vx_rel, vy_rel, class_id, confidence, distance]

fused_tracks = zeros(5, 7);

for i = 1:5
    cam = camera_detections(i, :);
    lid = lidar_clusters(i, :);
    rad = radar_targets(i, :);

    % Fused Position: LiDAR centroid has highest spatial accuracy (sigma = 0.05m)
    if lid(4) > 0.0 % Valid LiDAR cluster
        x = lid(1);
        y = lid(2);
    elseif cam(5) > 0.0 % Fallback to camera pinhole inverse projection
        x = cam(1);
        y = cam(2);
    else
        x = rad(1) * cosd(rad(3));
        y = rad(1) * sind(rad(3));
    end

    % Fused Velocity: Doppler radar provides instantaneous range rate
    if rad(1) > 0.0
        vx = rad(2) * cosd(rad(3));
        vy = rad(2) * sind(rad(3));
    else
        vx = 0.0;
        vy = 0.0;
    end

    % Classification: YOLOv11 neural network output
    class_id = cam(5);
    if class_id == 0.0 && rad(1) > 0.0
        class_id = 1.0; % Default to vehicle if radar-only
    end

    dist = hypot(x, y);
    confidence = 0.92; % Nominal fused confidence score

    if dist > 0.1
        fused_tracks(i, :) = [x, y, vx, vy, class_id, confidence, dist];
    end
end

end
