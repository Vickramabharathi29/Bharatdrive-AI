classdef ManeuverState < int32
    % MANEUVERSTATE Enumeration of Tactical Driving Decisions for Autonomous Vehicles.
    % BharatDrive-AI - MathWorks Autonomous Driving Hackathon.

    enumeration
        Accelerate     (1)
        Brake          (2)
        Overtake       (3)
        Wait           (4)
        Stop           (5)
        EmergencyStop  (6)
        MoveLeft       (7)
        MoveRight      (8)
    end

    methods (Static)
        function name = get_name(state_val)
            switch int32(state_val)
                case 1, name = 'Accelerate';
                case 2, name = 'Brake';
                case 3, name = 'Overtake';
                case 4, name = 'Wait';
                case 5, name = 'Stop';
                case 6, name = 'Emergency Stop';
                case 7, name = 'Move Left';
                case 8, name = 'Move Right';
                otherwise, name = 'Unknown';
            end
        end

        function is_lateral = is_lateral_maneuver(state_val)
            s = int32(state_val);
            is_lateral = (s == 3 || s == 7 || s == 8); % Overtake, MoveLeft, MoveRight
        end

        function is_braking = is_braking_maneuver(state_val)
            s = int32(state_val);
            is_braking = (s == 2 || s == 4 || s == 5 || s == 6); % Brake, Wait, Stop, EmergencyStop
        end
    end
end
