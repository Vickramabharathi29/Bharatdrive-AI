% SETUP_PLANNER_BUSES Defines Simulink Bus Objects for the Adaptive Path Planning System.
% Run this script before loading or simulating the Simulink planner model.

clear BusVehicleState BusPerceptionTrack BusRiskContext BusPlannerOutput;

% 1. Vehicle State Bus (7 elements)
elems(1) = Simulink.BusElement; elems(1).Name = 'x'; elems(1).DataType = 'double';
elems(2) = Simulink.BusElement; elems(2).Name = 'y'; elems(2).DataType = 'double';
elems(3) = Simulink.BusElement; elems(3).Name = 'v'; elems(3).DataType = 'double';
elems(4) = Simulink.BusElement; elems(4).Name = 'yaw'; elems(4).DataType = 'double';
elems(5) = Simulink.BusElement; elems(5).Name = 'yaw_rate'; elems(5).DataType = 'double';
elems(6) = Simulink.BusElement; elems(6).Name = 'accel'; elems(6).DataType = 'double';
elems(7) = Simulink.BusElement; elems(7).Name = 'steer'; elems(7).DataType = 'double';
BusVehicleState = Simulink.Bus;
BusVehicleState.Elements = elems;
clear elems;

% 2. Environmental & Risk Context Bus (4 elements)
elems(1) = Simulink.BusElement; elems(1).Name = 'RoadWidth'; elems(1).DataType = 'double';
elems(2) = Simulink.BusElement; elems(2).Name = 'FrictionMu'; elems(2).DataType = 'double';
elems(3) = Simulink.BusElement; elems(3).Name = 'Visibility'; elems(3).DataType = 'double';
elems(4) = Simulink.BusElement; elems(4).Name = 'TrafficDensity'; elems(4).DataType = 'double';
BusRiskContext = Simulink.Bus;
BusRiskContext.Elements = elems;
clear elems;

% 3. Planner Output Command Bus (6 elements)
elems(1) = Simulink.BusElement; elems(1).Name = 'TargetVelocity'; elems(1).DataType = 'double';
elems(2) = Simulink.BusElement; elems(2).Name = 'TargetYawRate'; elems(2).DataType = 'double';
elems(3) = Simulink.BusElement; elems(3).Name = 'CommandedSteer'; elems(3).DataType = 'double';
elems(4) = Simulink.BusElement; elems(4).Name = 'CommandedAccel'; elems(4).DataType = 'double';
elems(5) = Simulink.BusElement; elems(5).Name = 'TacticalNudge'; elems(5).DataType = 'double';
elems(6) = Simulink.BusElement; elems(6).Name = 'RiskIndex'; elems(6).DataType = 'double';
BusPlannerOutput = Simulink.Bus;
BusPlannerOutput.Elements = elems;
clear elems;

fprintf('=== Simulink Planner Bus Objects Initialized in Base Workspace ===\n');
fprintf('  - BusVehicleState\n');
fprintf('  - BusRiskContext\n');
fprintf('  - BusPlannerOutput\n');
