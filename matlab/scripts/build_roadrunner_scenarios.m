function scenarios = build_roadrunner_scenarios(scenario_id)
% BUILD_ROADRUNNER_SCENARIOS Programmatically generates the 5 Indian road
% driving scenarios for MATLAB Automated Driving Toolbox & RoadRunner export.
%
% Usage:
%   scenarios = build_roadrunner_scenarios(); % builds all 5 scenarios
%   s1 = build_roadrunner_scenarios(1);      % builds Scenario 1: Village Road
%   s5 = build_roadrunner_scenarios(5);      % builds Scenario 5: Cattle Crossing

    if nargin < 1
        scenario_id = 0; % All
    end

    scenarios = struct();

    if scenario_id == 0 || scenario_id == 1
        scenarios.village_road = build_village_road();
    end
    if scenario_id == 0 || scenario_id == 2
        scenarios.busy_market = build_busy_market();
    end
    if scenario_id == 0 || scenario_id == 3
        scenarios.urban_junction = build_urban_junction();
    end
    if scenario_id == 0 || scenario_id == 4
        scenarios.highway_merge = build_highway_merge();
    end
    if scenario_id == 0 || scenario_id == 5
        scenarios.cattle_crossing = build_cattle_crossing();
    end

    fprintf('=== BharatDrive-AI: RoadRunner / Automated Driving Scenarios Generated ===\n');
end

%% Scenario 1: Village Road (Gramin Sadak)
function scenario = build_village_road()
    scenario = drivingScenario('SampleTime', 0.05, 'StopTime', 15.0);

    % Road center waypoints (S-curve rural single lane)
    roadCenters = [
          0,   0, 0;
         50,   2, 0;
        100,  15, 0;
        160,  18, 0;
        220,   5, 0;
        300,   0, 0
    ];
    % 3.75m single lane carriageway (unmarked)
    road(scenario, roadCenters, 'Width', 3.75);

    % Ego Vehicle (Tata Nexon EV class)
    ego = vehicle(scenario, ...
        'ClassID', 1, ...
        'Length', 3.993, ...
        'Width', 1.811, ...
        'Height', 1.606, ...
        'Mesh', driving.scenario.carMesh);
    trajectory(ego, roadCenters(1:4, :), 8.33); % 30 km/h

    % Oncoming Farm Tractor with Sugarcane Trolley
    tractor = vehicle(scenario, ...
        'ClassID', 2, ...
        'Length', 8.20, ...
        'Width', 2.30, ...
        'Height', 2.80, ...
        'Mesh', driving.scenario.truckMesh);
    tractorPath = [
        250,  3, 0;
        180, 16, 0;
        110, 14, 0;
         50,  1, 0
    ];
    trajectory(tractor, tractorPath, 5.00); % 18 km/h oncoming

    % Villager Pedestrian walking on shoulder edge
    ped = actor(scenario, ...
        'ClassID', 4, ...
        'Length', 0.5, ...
        'Width', 0.5, ...
        'Height', 1.7, ...
        'Mesh', driving.scenario.pedestrianMesh);
    trajectory(ped, [85, 9, 0; 120, 18, 0], 1.10);
end

%% Scenario 2: Busy Market (Bazaar Street)
function scenario = build_busy_market()
    scenario = drivingScenario('SampleTime', 0.05, 'StopTime', 12.0);

    roadCenters = [
          0, 0, 0;
        100, 0, 0;
        250, 0, 0
    ];
    % Constrained 9.0m market corridor
    road(scenario, roadCenters, 'Width', 9.0);

    % Ego vehicle crawling at 15 km/h
    ego = vehicle(scenario, 'ClassID', 1, 'Length', 3.993, 'Width', 1.811);
    trajectory(ego, [0, 0, 0; 150, 0, 0], 4.17);

    % Aggressive Auto-Rickshaw cutting in and stopping
    rickshaw = vehicle(scenario, ...
        'ClassID', 3, ...
        'Length', 2.70, ...
        'Width', 1.40, ...
        'Height', 1.80);
    rickshawPath = [
         25,  2.0, 0;
         45,  0.5, 0; % Cut-in directly ahead of ego
         70, -2.5, 0  % Sudden swerve to curb to drop passenger
    ];
    trajectory(rickshaw, rickshawPath, [5.0, 4.0, 1.0]);

    % Shopper crossing blind between roadside carts
    shopper = actor(scenario, 'ClassID', 4, 'Length', 0.5, 'Width', 0.5, 'Height', 1.7);
    trajectory(shopper, [48, 3.8, 0; 48, -3.8, 0], 1.25);
end

%% Scenario 3: Urban Junction (Uncontrolled Roundabout)
function scenario = build_urban_junction()
    scenario = drivingScenario('SampleTime', 0.05, 'StopTime', 15.0);

    % 4 approach legs intersecting at (0, 0)
    road(scenario, [ -80, 0, 0; 80, 0, 0 ], 'Width', 7.0); % East-West
    road(scenario, [ 0, -80, 0; 0, 80, 0 ], 'Width', 7.0); % North-South

    % Ego approaching from South intending to turn right (East)
    ego = vehicle(scenario, 'ClassID', 1, 'Length', 3.993, 'Width', 1.811);
    egoPath = [
          0, -60, 0;
          0, -10, 0;
          8,   0, 0;
         40,   0, 0
    ];
    trajectory(ego, egoPath, 6.94);

    % Heavy City Bus crossing from West without yielding
    bus = vehicle(scenario, ...
        'ClassID', 5, ...
        'Length', 11.5, ...
        'Width', 2.6, ...
        'Height', 3.2, ...
        'Mesh', driving.scenario.truckMesh);
    busPath = [
        -60, 0, 0;
         10, 0, 0;
         60, 0, 0
    ];
    trajectory(bus, busPath, 8.33);
end

%% Scenario 4: Highway Merge with Wrong-Way Vehicle
function scenario = build_highway_merge()
    scenario = drivingScenario('SampleTime', 0.05, 'StopTime', 12.0);

    % Divided mainline (dual 7.0m lanes)
    road(scenario, [ 0, -1.75, 0; 400, -1.75, 0 ], 'Width', 7.0);

    % Merge on-ramp
    rampCenters = [
        100, -25.0, 0;
        180, -12.0, 0;
        260,  -4.0, 0;
        320,  -1.75, 0
    ];
    road(scenario, rampCenters, 'Width', 4.5);

    % Fast-moving Ego in left lane
    ego = vehicle(scenario, 'ClassID', 1, 'Length', 3.993, 'Width', 1.811);
    trajectory(ego, [ 50, -1.75, 0; 350, -1.75, 0 ], 22.22); % 80 km/h

    % Merging vehicle from on-ramp
    merger = vehicle(scenario, 'ClassID', 1, 'Length', 4.4, 'Width', 1.8);
    trajectory(merger, rampCenters, 11.11); % 40 km/h

    % Wrong-way two-wheeler riding against traffic on shoulder
    wrongWay = vehicle(scenario, 'ClassID', 6, 'Length', 2.0, 'Width', 0.8);
    trajectory(wrongWay, [ 350, -5.5, 0; 150, -5.5, 0 ], 6.94); % Oncoming along shoulder
end

%% Scenario 5: Cattle Crossing on Undivided Road
function scenario = build_cattle_crossing()
    scenario = drivingScenario('SampleTime', 0.05, 'StopTime', 10.0);

    % 7.0m undivided two-lane road
    road(scenario, [ 0, 0, 0; 300, 0, 0 ], 'Width', 7.0);

    % Ego vehicle at 50 km/h (13.89 m/s) in left lane (y = -1.75)
    ego = vehicle(scenario, 'ClassID', 1, 'Length', 3.993, 'Width', 1.811);
    trajectory(ego, [ 10, -1.75, 0; 250, -1.75, 0 ], 13.89);

    % Stationary Cow standing across ego lane
    leadCow = actor(scenario, ...
        'ClassID', 7, ...
        'Length', 2.20, ...
        'Width', 0.90, ...
        'Height', 1.50);
    % Placed perpendicular across ego lane at s = 90m
    leadCow.Position = [90, -1.50, 0];
    leadCow.Yaw = 90.0; % Perpendicular to road

    % Calf sprinting across from shoulder
    calf = actor(scenario, 'ClassID', 7, 'Length', 1.30, 'Width', 0.60, 'Height', 1.00);
    trajectory(calf, [80, -4.0, 0; 88, -1.0, 0], 2.80);

    % Oncoming commercial bus in right lane (y = +1.75)
    bus = vehicle(scenario, ...
        'ClassID', 5, ...
        'Length', 11.2, ...
        'Width', 2.6, ...
        'Height', 3.2, ...
        'Mesh', driving.scenario.truckMesh);
    trajectory(bus, [ 220, 1.75, 0; 30, 1.75, 0 ], 12.50); % Oncoming at 45 km/h
end
