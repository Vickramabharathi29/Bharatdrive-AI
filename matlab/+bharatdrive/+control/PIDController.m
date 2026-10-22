classdef PIDController < handle
    % PIDCONTROLLER Proportional-Integral-Derivative Controller for Autonomous Vehicles.
    % Supports both Longitudinal Speed Tracking and Lateral Path Tracking with Anti-Windup.
    % BharatDrive-AI - MathWorks Autonomous Driving Hackathon.

    properties
        Kp              % Proportional gain
        Ki              % Integral gain
        Kd              % Derivative gain
        Ktheta          % Heading error gain (for lateral tracking)
        IntegralError   % Accumulated integral error
        PrevError       % Previous step error for finite difference derivative
        OutputMin       % Lower output saturation limit
        OutputMax       % Upper output saturation limit
        AntiWindupLimit % Maximum allowable integral accumulation
    end

    methods
        function obj = PIDController(kp, ki, kd, out_min, out_max, ktheta)
            if nargin < 6, ktheta = 0.0; end
            if nargin < 5, out_max = 1.0; end
            if nargin < 4, out_min = -1.0; end
            if nargin < 3, kd = 0.0; end
            if nargin < 2, ki = 0.0; end
            if nargin < 1, kp = 1.0; end

            obj.Kp = kp;
            obj.Ki = ki;
            obj.Kd = kd;
            obj.Ktheta = ktheta;
            obj.IntegralError = 0.0;
            obj.PrevError = 0.0;
            obj.OutputMin = out_min;
            obj.OutputMax = out_max;
            obj.AntiWindupLimit = (out_max - out_min) * 0.5 / max(1e-4, abs(ki));
        end

        function reset(obj)
            obj.IntegralError = 0.0;
            obj.PrevError = 0.0;
        end

        function u = compute_longitudinal(obj, target_v, current_v, dt)
            % COMPUTE_LONGITUDINAL Calculates commanded acceleration (m/s^2)
            if nargin < 4, dt = 0.01; end
            err = target_v - current_v;

            % Anti-windup clamped integration
            obj.IntegralError = obj.IntegralError + err * dt;
            obj.IntegralError = max(-obj.AntiWindupLimit, min(obj.AntiWindupLimit, obj.IntegralError));

            derivative = (err - obj.PrevError) / max(1e-5, dt);
            obj.PrevError = err;

            raw_u = obj.Kp * err + obj.Ki * obj.IntegralError + obj.Kd * derivative;
            u = max(obj.OutputMin, min(obj.OutputMax, raw_u));
        end

        function delta = compute_lateral(obj, ey, etheta, dt)
            % COMPUTE_LATERAL Calculates commanded steering angle (rad)
            if nargin < 4, dt = 0.01; end

            obj.IntegralError = obj.IntegralError + ey * dt;
            obj.IntegralError = max(-obj.AntiWindupLimit, min(obj.AntiWindupLimit, obj.IntegralError));

            derivative = (ey - obj.PrevError) / max(1e-5, dt);
            obj.PrevError = ey;

            raw_delta = obj.Kp * ey + obj.Ki * obj.IntegralError + obj.Kd * derivative + obj.Ktheta * etheta;
            delta = max(obj.OutputMin, min(obj.OutputMax, raw_delta));
        end
    end
end
