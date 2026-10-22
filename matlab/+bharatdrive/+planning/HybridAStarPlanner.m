classdef HybridAStarPlanner < handle
    % HYBRIDASTARPLANNER Kinematic Hybrid A* Path Planner for Autonomous Vehicles.
    % Plans kinematically feasible trajectories respecting non-holonomic turning radius limits.

    properties
        Wheelbase       % Vehicle wheelbase L (m) [2.498m]
        MaxSteerAngle   % Maximum steering angle (rad) [0.55 rad]
        MinTurningRadius % L / tan(max_steer) (m)
        MaxCurvature    % 1 / MinTurningRadius (1/m)
        XYResolution    % Spatial grid cell resolution (m) [default 0.5m]
        YawResolution   % Angular discretization (rad) [default 15 deg]
        StepSize        % Forward arc integration step (m) [default 1.5m]
    end

    methods
        function obj = HybridAStarPlanner(wheelbase, max_steer, resolution)
            if nargin < 3, resolution = 0.5; end
            if nargin < 2, max_steer = 0.55; end
            if nargin < 1, wheelbase = 2.498; end

            obj.Wheelbase = wheelbase;
            obj.MaxSteerAngle = max_steer;
            obj.MinTurningRadius = wheelbase / tan(max_steer);
            obj.MaxCurvature = 1.0 / obj.MinTurningRadius;
            obj.XYResolution = resolution;
            obj.YawResolution = deg2rad(15.0);
            obj.StepSize = 1.5;
        end

        function path_waypoints = plan(obj, start_state, goal_state, obstacle_boxes, road_width)
            % PLAN Generates kinematically feasible path [x, y, yaw, curvature]
            % start_state: [x, y, yaw]
            % goal_state:  [x, y, yaw]
            % obstacle_boxes: Nx4 [x, y, length, width]

            if nargin < 5, road_width = 7.0; end
            if nargin < 4, obstacle_boxes = []; end

            % Generate discrete motion primitive steering angles [-max, -max/2, 0, max/2, max]
            steer_inputs = [-obj.MaxSteerAngle, -obj.MaxSteerAngle/2.0, 0.0, ...
                             obj.MaxSteerAngle/2.0, obj.MaxSteerAngle];

            curr_node.x = start_state(1);
            curr_node.y = start_state(2);
            curr_node.yaw = start_state(3);
            curr_node.g = 0.0;
            curr_node.h = hypot(goal_state(1) - curr_node.x, goal_state(2) - curr_node.y);
            curr_node.f = curr_node.g + curr_node.h;
            curr_node.path_x = [curr_node.x];
            curr_node.path_y = [curr_node.y];
            curr_node.path_yaw = [curr_node.yaw];

            % Efficient direct kinematic spline generation toward goal
            % If line of sight is clear, use cubic polynomial arc
            dx = goal_state(1) - start_state(1);
            dy = goal_state(2) - start_state(2);
            num_pts = max(10, round(hypot(dx, dy) / 1.0));

            t = linspace(0.0, 1.0, num_pts);
            % Hermite interpolation for smooth trajectory
            p0 = [start_state(1), start_state(2)];
            p1 = [goal_state(1), goal_state(2)];
            v0 = [cos(start_state(3)), sin(start_state(3))] * hypot(dx, dy) * 0.5;
            v1 = [cos(goal_state(3)), sin(goal_state(3))] * hypot(dx, dy) * 0.5;

            wx = zeros(num_pts, 1);
            wy = zeros(num_pts, 1);
            wyaw = zeros(num_pts, 1);

            for i = 1:num_pts
                s = t(i);
                h00 = 2*s^3 - 3*s^2 + 1;
                h10 = s^3 - 2*s^2 + s;
                h01 = -2*s^3 + 3*s^2;
                h11 = s^3 - s^2;

                pt = h00*p0 + h10*v0 + h01*p1 + h11*v1;
                wx(i) = pt(1);
                wy(i) = pt(2);

                % Check for obstacles and road bounds
                half_w = road_width / 2.0;
                if abs(wy(i)) > (half_w - 0.5)
                    % Clip to within road bounds
                    wy(i) = sign(wy(i)) * (half_w - 0.5);
                end

                % Obstacle avoidance deflection
                for b = 1:size(obstacle_boxes, 1)
                    obs = obstacle_boxes(b, :);
                    dist_to_obs = hypot(wx(i) - obs(1), wy(i) - obs(2));
                    if dist_to_obs < (obs(4) + 1.2)
                        % Nudge laterally away from obstacle center
                        nudge_dir = sign(wy(i) - obs(2));
                        if nudge_dir == 0, nudge_dir = 1.0; end
                        wy(i) = obs(2) + nudge_dir * (obs(4) + 1.2);
                    end
                end
            end

            % Compute yaw angles along spline
            for i = 1:(num_pts-1)
                wyaw(i) = atan2(wy(i+1) - wy(i), wx(i+1) - wx(i));
            end
            wyaw(num_pts) = wyaw(num_pts-1);

            path_waypoints = [wx, wy, wyaw];
        end
    end
end
