classdef SpatioTemporalCostmap < handle
    % SPATIOTEMPORALCOSTMAP Generates a 2D dynamic risk costmap in vehicle body frame.
    % Synthesizes potential field risk contours for Frenet optimal trajectory generation.

    properties
        XRange      % [min_x, max_x] relative to ego vehicle (m) [e.g. -10, 80]
        YRange      % [min_y, max_y] relative to ego vehicle (m) [e.g. -12, 12]
        Resolution  % Grid cell size (m) [default 0.25m]
        GridWidth   % Number of grid cells along X
        GridHeight  % Number of grid cells along Y
        CostGrid    % 2D numeric matrix [Height x Width] representing collision cost
        XGrid       % Meshgrid X coordinates (m)
        YGrid       % Meshgrid Y coordinates (m)
    end

    methods
        function obj = SpatioTemporalCostmap(x_range, y_range, resolution)
            if nargin < 3, resolution = 0.25; end
            if nargin < 2, y_range = [-10.0, 10.0]; end
            if nargin < 1, x_range = [-10.0, 70.0]; end

            obj.XRange = x_range;
            obj.YRange = y_range;
            obj.Resolution = resolution;

            x_vec = x_range(1):resolution:x_range(2);
            y_vec = y_range(1):resolution:y_range(2);

            obj.GridWidth = length(x_vec);
            obj.GridHeight = length(y_vec);

            [obj.XGrid, obj.YGrid] = meshgrid(x_vec, y_vec);
            obj.CostGrid = zeros(obj.GridHeight, obj.GridWidth);
        end

        function reset(obj)
            obj.CostGrid = zeros(obj.GridHeight, obj.GridWidth);
        end

        function update_costmap(obj, risk_assessments, road_width)
            % Resets and injects dynamic risk potential fields for all assessed obstacles
            obj.reset();

            % 1. Road Boundary Confinement Cost
            half_w = road_width / 2.0;
            dist_to_left_edge = half_w - obj.YGrid;
            dist_to_right_edge = obj.YGrid - (-half_w);

            % Exponential repulsive cost near or beyond road shoulders
            boundary_cost = 25.0 * (exp(-max(0.0, dist_to_left_edge) / 0.5) + ...
                                   exp(-max(0.0, dist_to_right_edge) / 0.5));
            % Penalize off-road areas with saturation
            off_road_mask = (obj.YGrid > half_w) | (obj.YGrid < -half_w);
            boundary_cost(off_road_mask) = 150.0;
            obj.CostGrid = obj.CostGrid + boundary_cost;

            % 2. Obstacle Risk Potentials
            for i = 1:length(risk_assessments)
                res = risk_assessments{i};
                x_obs = res.Position(1);
                y_obs = res.Position(2);
                risk = res.RiskScore;

                % Asymmetric ellipse based on object class and risk
                sigma_x = 2.0 + 0.3 * max(0.0, res.Factors.R_kin * 10.0);
                if strcmp(res.ObjectClass, 'Two-Wheeler') || strcmp(res.ObjectClass, 'Cattle')
                    sigma_y = 1.8; % Wider lateral buffer
                else
                    sigma_y = 1.2;
                end

                % Gaussian potential field peak height proportional to IRRM score
                peak_cost = 250.0 * risk;
                obstacle_potential = peak_cost * exp(-(((obj.XGrid - x_obs).^2) / (2 * sigma_x^2) + ...
                                                      ((obj.YGrid - y_obs).^2) / (2 * sigma_y^2)));

                obj.CostGrid = obj.CostGrid + obstacle_potential;
            end

            % Saturate maximum cost
            obj.CostGrid = min(300.0, obj.CostGrid);
        end
    end
end
