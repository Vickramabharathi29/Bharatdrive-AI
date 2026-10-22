classdef SensorSuite < handle
    % SENSORSUITE Simulates Camera, LiDAR, and Radar sensor measurements
    % from a drivingScenario environment, calibrated for Indian driving dynamics.

    properties
        % Sensor Mounting Configurations relative to rear axle (m)
        CameraMountPos    % [x, y, z] e.g. [2.0, 0, 1.4] (windshield)
        LidarMountPos     % [x, y, z] e.g. [1.5, 0, 1.8] (roof)
        RadarMountPos     % [x, y, z] e.g. [3.5, 0, 0.5] (front bumper)

        % Field of View and Range Limits
        CameraFOV         % Horizontal FOV [deg], max range [m]
        LidarFOV          % Horizontal FOV [deg], max range [m]
        RadarFOV          % Horizontal FOV [deg], max range [m]

        % Measurement Noise Standard Deviations
        CameraNoise       % [sigma_azimuth_deg, sigma_range_pct]
        LidarNoise        % [sigma_x_m, sigma_y_m, sigma_z_m]
        RadarNoise        % [sigma_range_m, sigma_azimuth_deg, sigma_range_rate_mps]

        % Environmental Weather / Degradation Factor (0.0 = clear, 1.0 = heavy monsoon/dust)
        WeatherDegradation
    end

    methods
        function obj = SensorSuite(weather_factor)
            if nargin < 1
                weather_factor = 0.15; % Typical moderate dust/haze
            end
            obj.WeatherDegradation = weather_factor;

            % Calibrated mounting geometries (Tata Nexon EV roof/windshield/bumper)
            obj.CameraMountPos = [2.0, 0.0, 1.45];
            obj.LidarMountPos  = [1.4, 0.0, 1.75];
            obj.RadarMountPos  = [3.4, 0.0, 0.45];

            % Operational ranges
            obj.CameraFOV = struct('FOV_deg', 65.0, 'MaxRange_m', 80.0, 'MinRange_m', 1.0);
            obj.LidarFOV  = struct('FOV_deg', 120.0, 'MaxRange_m', 90.0, 'MinRange_m', 0.5);
            obj.RadarFOV  = struct('FOV_deg', 45.0, 'MaxRange_m', 150.0, 'MinRange_m', 1.5);

            % Noise characteristics
            obj.CameraNoise = [0.8, 0.04]; % 0.8 deg azimuth, 4% range
            obj.LidarNoise  = [0.06, 0.06, 0.03]; % 6cm X, 6cm Y, 3cm Z
            obj.RadarNoise  = [0.25, 1.2, 0.15];  % 25cm range, 1.2 deg azimuth, 0.15 m/s Doppler
        end

        function detections = generate_synthetic_measurements(obj, ego_state, target_actors, current_time)
            % Generates raw, un-fused measurement packets from Camera, LiDAR, and Radar
            % ego_state: [x, y, v, yaw, ...]
            % target_actors: struct array of ground-truth actors

            detections = struct('Camera', [], 'Lidar', [], 'Radar', []);
            cam_list = {};
            lidar_list = {};
            radar_list = {};

            ego_x = ego_state(1);
            ego_y = ego_state(2);
            ego_v = ego_state(3);
            ego_yaw = ego_state(4);

            cos_y = cos(ego_yaw);
            sin_y = sin(ego_yaw);

            for i = 1:length(target_actors)
                act = target_actors(i);
                % Relative position in global frame
                dx_glob = act.Position(1) - ego_x;
                dy_glob = act.Position(2) - ego_y;

                % Transform to ego vehicle body frame (X = Forward, Y = Left)
                x_b = dx_glob * cos_y + dy_glob * sin_y;
                y_b = -dx_glob * sin_y + dy_glob * cos_y;
                z_b = act.Position(3) - 0.0;

                range = hypot(x_b, y_b);
                azimuth_deg = atan2d(y_b, x_b);

                % Relative velocity
                vx_glob = act.Velocity(1) - ego_v * cos_y;
                vy_glob = act.Velocity(2) - ego_v * sin_y;
                vx_b = vx_glob * cos_y + vy_glob * sin_y;
                vy_b = -vx_glob * sin_y + vy_glob * cos_y;
                radial_velocity = (x_b * vx_b + y_b * vy_b) / max(range, 0.1);

                %% 1. Camera Detection Model
                % Camera has rich classification, but range degrades with weather/distance
                if range <= obj.CameraFOV.MaxRange_m && abs(azimuth_deg) <= (obj.CameraFOV.FOV_deg / 2)
                    cam_noise_r = (obj.CameraNoise(2) + obj.WeatherDegradation * 0.05) * range * randn();
                    cam_noise_az = obj.CameraNoise(1) * randn();
                    meas_range = max(0.5, range + cam_noise_r);
                    meas_az = azimuth_deg + cam_noise_az;

                    meas_x = meas_range * cosd(meas_az);
                    meas_y = meas_range * sind(meas_az);

                    cam_det.SensorType = 'Camera';
                    cam_det.TargetID = act.ActorID;
                    cam_det.Measurement = [meas_x; meas_y; z_b];
                    cam_det.MeasurementNoise = diag([max(0.5, 0.05*meas_x)^2, 0.15^2, 0.2^2]);
                    cam_det.ObjectClass = act.ObjectClass;
                    cam_det.ClassificationConfidence = max(0.65, 0.98 - (range/100.0) - obj.WeatherDegradation*0.25);
                    cam_det.Dimensions = act.Dimensions;
                    cam_det.Time = current_time;
                    cam_list{end+1} = cam_det;
                end

                %% 2. LiDAR Detection Model
                % LiDAR delivers precise 3D spatial point clusters & bounding box
                if range <= obj.LidarFOV.MaxRange_m && abs(azimuth_deg) <= (obj.LidarFOV.FOV_deg / 2)
                    % High precision 3D coordinates
                    lid_x = x_b + obj.LidarNoise(1) * randn();
                    lid_y = y_b + obj.LidarNoise(2) * randn();
                    lid_z = z_b + obj.LidarNoise(3) * randn();

                    lid_det.SensorType = 'LiDAR';
                    lid_det.TargetID = act.ActorID;
                    lid_det.Measurement = [lid_x; lid_y; lid_z];
                    lid_det.MeasurementNoise = diag([0.08^2, 0.08^2, 0.04^2]);
                    lid_det.Dimensions = act.Dimensions + 0.05 * randn(1, 3);
                    lid_det.PointCloudPoints = max(15, round(800.0 / max(range, 2.0)));
                    lid_det.ObjectClass = act.ObjectClass;
                    lid_det.Time = current_time;
                    lidar_list{end+1} = lid_det;
                end

                %% 3. Radar Detection Model
                % Radar provides direct Doppler range rate, immune to weather
                if range <= obj.RadarFOV.MaxRange_m && abs(azimuth_deg) <= (obj.RadarFOV.FOV_deg / 2)
                    rad_range = range + obj.RadarNoise(1) * randn();
                    rad_az = azimuth_deg + obj.RadarNoise(2) * randn();
                    rad_rate = radial_velocity + obj.RadarNoise(3) * randn();

                    rad_x = rad_range * cosd(rad_az);
                    rad_y = rad_range * sind(rad_az);

                    rad_det.SensorType = 'Radar';
                    rad_det.TargetID = act.ActorID;
                    rad_det.Measurement = [rad_x; rad_y; rad_rate]; % [x, y, range_rate]
                    rad_det.MeasurementNoise = diag([0.3^2, 0.8^2, 0.2^2]);
                    rad_det.DopplerVelocity = rad_rate;
                    rad_det.RCS_dBsm = 10.0; % Radar cross section
                    rad_det.Time = current_time;
                    radar_list{end+1} = rad_det;
                end
            end

            detections.Camera = cam_list;
            detections.Lidar = lidar_list;
            detections.Radar = radar_list;
        end
    end
end
