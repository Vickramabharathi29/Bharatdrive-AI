classdef TrackedObject < handle
    % TRACKEDOBJECT Standardized data structure for fused perception outputs.
    % Adheres to BharatDrive-AI perception specifications for Indian roads.

    properties
        ObjectID            % Unique positive integer tracking identifier
        ObjectClass         % Semantic class name: 'Auto-Rickshaw', 'Two-Wheeler', 'Cattle', 'Pedestrian', 'Car', 'Truck-Bus', 'Pothole'
        ClassID             % Numeric identifier for Automated Driving Toolbox compatibility
        Position            % 1x3 vector [x, y, z] in ego vehicle body coordinates (m)
        Velocity            % 1x3 vector [vx, vy, vz] in ego vehicle body coordinates (m/s)
        Distance            % Euclidean distance from ego vehicle front bumper (m)
        RelativeSpeed       % Relative closing speed (m/s) (positive = approaching)
        ConfidenceScore     % Fused confidence score in range [0.0, 1.0]
        Dimensions          % 1x3 vector [length, width, height] (m)
        Yaw                 % Orientation angle with respect to ego heading (rad)
        SensorContributors  % Cell array of active sensors, e.g. {'Camera', 'Radar', 'LiDAR'}
        TrackStatus         % 'Tentative', 'Confirmed', 'Coasted'
        Age                 % Number of total simulation steps since track initiation
        TotalHits           % Total number of measurement updates associated with this track
    end

    methods
        function obj = TrackedObject(id, class_name, pos, vel, dims, conf, sensors)
            if nargin > 0
                obj.ObjectID = id;
                obj.ObjectClass = class_name;
                obj.Position = pos;
                obj.Velocity = vel;
                obj.Dimensions = dims;
                obj.ConfidenceScore = conf;
                obj.SensorContributors = sensors;
                obj.Distance = hypot(pos(1), pos(2));
                obj.RelativeSpeed = -vel(1); % closing speed
                obj.Yaw = 0.0;
                obj.TrackStatus = 'Tentative';
                obj.Age = 1;
                obj.TotalHits = 1;
            else
                obj.Position = [0, 0, 0];
                obj.Velocity = [0, 0, 0];
                obj.Dimensions = [1, 1, 1];
                obj.ConfidenceScore = 0.5;
                obj.SensorContributors = {};
                obj.Distance = 0.0;
                obj.RelativeSpeed = 0.0;
            end
        end

        function update_metrics(obj)
            % Recalculate range and closing speed after position/velocity updates
            obj.Distance = hypot(obj.Position(1), obj.Position(2));
            obj.RelativeSpeed = -obj.Velocity(1);
        end

        function s = to_struct(obj)
            % Converts object to plain MATLAB struct for Simulink bus compatibility
            s.ObjectID = uint32(obj.ObjectID);
            s.ObjectClass = string(obj.ObjectClass);
            s.Position = double(obj.Position);
            s.Velocity = double(obj.Velocity);
            s.Distance = double(obj.Distance);
            s.RelativeSpeed = double(obj.RelativeSpeed);
            s.ConfidenceScore = double(obj.ConfidenceScore);
            s.Dimensions = double(obj.Dimensions);
            s.Yaw = double(obj.Yaw);
            s.TrackStatus = string(obj.TrackStatus);
        end
    end
end
