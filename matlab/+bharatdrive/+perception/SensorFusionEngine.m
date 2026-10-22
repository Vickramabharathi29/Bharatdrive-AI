classdef SensorFusionEngine < handle
    % SENSORFUSIONENGINE Central Multi-Sensor Fusion and Tracking Engine.
    % Fuses asynchronous/synchronized Camera, LiDAR, and Radar measurements
    % using Extended Kalman Filtering (EKF) and Global Nearest Neighbor (GNN) gating.

    properties
        TrackList               % Array of active internal track structures
        NextTrackID             % Counter for unique track assignment
        ConfirmationThreshold   % Required hits to confirm track [e.g. 2 out of 3]
        DeletionThreshold       % Maximum coasted frames before track removal
        ProcessNoiseQ           % Process noise covariance matrix (4x4)
        GateThreshold           % Mahalanobis gating distance threshold
        DeltaT                  % Simulation step time (s)
    end

    methods
        function obj = SensorFusionEngine(dt)
            if nargin < 1
                dt = 0.05; % 20 Hz perception cycle
            end
            obj.DeltaT = dt;
            obj.TrackList = [];
            obj.NextTrackID = uint32(1);
            obj.ConfirmationThreshold = 2;
            obj.DeletionThreshold = 4;
            obj.GateThreshold = 12.0; % Chi-Square threshold for 2-DOF gating

            % Process noise for Constant Velocity (CV) motion model
            % State vector: [x; y; vx; vy]
            q_pos = 0.15;
            q_vel = 1.20;
            obj.ProcessNoiseQ = diag([q_pos^2, q_pos^2, q_vel^2, q_vel^2]) * dt;
        end

        function confirmed_tracks = step(obj, sensor_detections, current_time)
            % Executes single cycle: Predict -> Associate -> Update -> Manage -> Output
            % sensor_detections: struct with fields .Camera, .Lidar, .Radar

            obj.predict_tracks();
            obj.update_with_sensor(sensor_detections.Lidar,  'LiDAR');
            obj.update_with_sensor(sensor_detections.Camera, 'Camera');
            obj.update_with_sensor(sensor_detections.Radar,  'Radar');
            obj.manage_tracks();

            % Format confirmed and active tracks for output
            confirmed_tracks = obj.format_output_tracks(current_time);
        end

        function predict_tracks(obj)
            % Propagate tracks forward using Constant Velocity transition matrix F
            dt = obj.DeltaT;
            F = [1, 0, dt,  0;
                 0, 1,  0, dt;
                 0, 0,  1,  0;
                 0, 0,  0,  1];

            for i = 1:length(obj.TrackList)
                tr = obj.TrackList(i);
                tr.State = F * tr.State;
                tr.Covariance = F * tr.Covariance * F' + obj.ProcessNoiseQ;
                tr.Age = tr.Age + 1;
                tr.ConsecutiveMisses = tr.ConsecutiveMisses + 1;
                tr.ActiveSensors = {}; % Reset per cycle
                obj.TrackList(i) = tr;
            end
        end

        function update_with_sensor(obj, detections, sensor_type)
            if isempty(detections)
                return;
            end

            for d = 1:length(detections)
                meas = detections{d};
                z_pos = meas.Measurement(1:2); % [x; y]

                % Association via Minimum Mahalanobis Distance
                best_idx = 0;
                min_dist = obj.GateThreshold;

                for t = 1:length(obj.TrackList)
                    tr = obj.TrackList(t);
                    H = [1, 0, 0, 0;
                         0, 1, 0, 0];
                    z_pred = H * tr.State;
                    y = z_pos - z_pred; % Innovation
                    S = H * tr.Covariance * H' + meas.MeasurementNoise(1:2, 1:2);

                    d_mahalanobis = y' * (S \ y);
                    if d_mahalanobis < min_dist
                        min_dist = d_mahalanobis;
                        best_idx = t;
                    end
                end

                if best_idx > 0
                    % Update existing track
                    obj.apply_kalman_update(best_idx, meas, sensor_type);
                else
                    % Initiate tentative new track
                    obj.initiate_track(meas, sensor_type);
                end
            end
        end

        function apply_kalman_update(obj, track_idx, meas, sensor_type)
            tr = obj.TrackList(track_idx);

            if strcmp(sensor_type, 'Radar') && isfield(meas, 'DopplerVelocity')
                % Non-linear Extended Kalman Filter for Radar Range + Doppler
                x = tr.State(1);
                y = tr.State(2);
                vx = tr.State(3);
                vy = tr.State(4);
                range = max(hypot(x, y), 0.1);

                % Radar measurement vector: [x; y; range_rate]
                z = [meas.Measurement(1); meas.Measurement(2); meas.DopplerVelocity];
                z_pred = [x; y; (x*vx + y*vy)/range];

                % Measurement Jacobian H
                dr_dx = (y^2 * vx - x*y*vy) / (range^3);
                dr_dy = (x^2 * vy - x*y*vx) / (range^3);
                H = [1,      0,      0,        0;
                     0,      1,      0,        0;
                     dr_dx,  dr_dy,  x/range,  y/range];
                R = meas.MeasurementNoise;
            else
                % Linear measurement update for Camera and LiDAR: [x; y]
                z = meas.Measurement(1:2);
                H = [1, 0, 0, 0;
                     0, 1, 0, 0];
                z_pred = H * tr.State;
                R = meas.MeasurementNoise(1:2, 1:2);
            end

            y = z - z_pred;
            S = H * tr.Covariance * H' + R;
            K = (tr.Covariance * H') / S;

            tr.State = tr.State + K * y;
            tr.Covariance = (eye(4) - K * H) * tr.Covariance;
            tr.TotalHits = tr.TotalHits + 1;
            tr.ConsecutiveMisses = 0;

            % Record sensor contributor
            if ~any(strcmp(tr.ActiveSensors, sensor_type))
                tr.ActiveSensors{end+1} = sensor_type;
            end

            % Update semantic class if camera observed
            if strcmp(sensor_type, 'Camera') && isfield(meas, 'ObjectClass')
                tr.ObjectClass = meas.ObjectClass;
            end
            if isfield(meas, 'Dimensions')
                tr.Dimensions = meas.Dimensions;
            end

            obj.TrackList(track_idx) = tr;
        end

        function initiate_track(obj, meas, sensor_type)
            new_tr.ID = obj.NextTrackID;
            obj.NextTrackID = obj.NextTrackID + 1;

            new_tr.State = [meas.Measurement(1); meas.Measurement(2); 0.0; 0.0];
            new_tr.Covariance = diag([0.5^2, 0.5^2, 3.0^2, 3.0^2]);
            new_tr.Age = 1;
            new_tr.TotalHits = 1;
            new_tr.ConsecutiveMisses = 0;
            new_tr.Status = 'Tentative';
            new_tr.ActiveSensors = {sensor_type};

            if isfield(meas, 'ObjectClass')
                new_tr.ObjectClass = meas.ObjectClass;
            else
                new_tr.ObjectClass = 'Unknown';
            end

            if isfield(meas, 'Dimensions')
                new_tr.Dimensions = meas.Dimensions;
            else
                new_tr.Dimensions = [2.0, 1.0, 1.2];
            end

            if isempty(obj.TrackList)
                obj.TrackList = new_tr;
            else
                obj.TrackList(end+1) = new_tr;
            end
        end

        function manage_tracks(obj)
            valid_tracks = [];
            for i = 1:length(obj.TrackList)
                tr = obj.TrackList(i);

                % Track promotion rule: 2 hits upgrades tentative to confirmed
                if strcmp(tr.Status, 'Tentative') && tr.TotalHits >= obj.ConfirmationThreshold
                    tr.Status = 'Confirmed';
                end

                % Coasting status
                if tr.ConsecutiveMisses > 0 && strcmp(tr.Status, 'Confirmed')
                    tr.Status = 'Coasted';
                end

                % Pruning rule
                if tr.ConsecutiveMisses <= obj.DeletionThreshold
                    valid_tracks = [valid_tracks, tr];
                end
            end
            obj.TrackList = valid_tracks;
        end

        function output_objs = format_output_tracks(obj, current_time)
            output_objs = {};
            for i = 1:length(obj.TrackList)
                tr = obj.TrackList(i);
                % Only output active confirmed or recent tentative tracks
                if tr.ConsecutiveMisses > 2
                    continue;
                end

                pos = [tr.State(1), tr.State(2), 0.0];
                vel = [tr.State(3), tr.State(4), 0.0];

                % Compute confidence score based on hit ratio & multi-sensor synergy
                hit_ratio = min(1.0, tr.TotalHits / max(1, tr.Age));
                sensor_bonus = min(0.30, 0.10 * length(tr.ActiveSensors));
                cov_factor = 1.0 / (1.0 + trace(tr.Covariance(1:2, 1:2)));
                conf = min(0.99, max(0.20, 0.50 * hit_ratio + sensor_bonus + 0.20 * cov_factor));

                out = bharatdrive.perception.TrackedObject( ...
                    tr.ID, tr.ObjectClass, pos, vel, tr.Dimensions, conf, tr.ActiveSensors);
                out.TrackStatus = tr.Status;
                out.Age = tr.Age;
                out.TotalHits = tr.TotalHits;
                out.update_metrics();

                output_objs{end+1} = out;
            end
        end
    end
end
