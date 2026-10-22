% SETUP_ALL_SYSTEM_BUSES Initializes all Simulink Bus Objects for BharatDrive-AI.
% Run this master script before compiling or simulating BharatDrive_Complete_System.slx.
% BharatDrive-AI - MathWorks Autonomous Driving Hackathon.

clear BusVehicleState BusRiskContext BusPerceptionTracks BusPlannerOutput BusDecisionOutput BusControlCommand;

% -------------------------------------------------------------------------
% 1. BusVehicleState: 8-element Vehicle Odometry & Dynamic State
% -------------------------------------------------------------------------
clear elems;
elems(1) = Simulink.BusElement; elems(1).Name = 'x'; elems(1).DataType = 'double';
elems(2) = Simulink.BusElement; elems(2).Name = 'y'; elems(2).DataType = 'double';
elems(3) = Simulink.BusElement; elems(3).Name = 'v'; elems(3).DataType = 'double';
elems(4) = Simulink.BusElement; elems(4).Name = 'yaw'; elems(4).DataType = 'double';
elems(5) = Simulink.BusElement; elems(5).Name = 'yaw_rate'; elems(5).DataType = 'double';
elems(6) = Simulink.BusElement; elems(6).Name = 'accel'; elems(6).DataType = 'double';
elems(7) = Simulink.BusElement; elems(7).Name = 'steer'; elems(7).DataType = 'double';
elems(8) = Simulink.BusElement; elems(8).Name = 'lateral_slip'; elems(8).DataType = 'double';
BusVehicleState = Simulink.Bus;
BusVehicleState.Elements = elems;

% -------------------------------------------------------------------------
% 2. BusRiskContext: Environmental Road, Weather & Traffic Conditions
% -------------------------------------------------------------------------
clear elems;
elems(1) = Simulink.BusElement; elems(1).Name = 'RoadWidth'; elems(1).DataType = 'double';
elems(2) = Simulink.BusElement; elems(2).Name = 'FrictionMu'; elems(2).DataType = 'double';
elems(3) = Simulink.BusElement; elems(3).Name = 'Visibility'; elems(3).DataType = 'double';
elems(4) = Simulink.BusElement; elems(4).Name = 'TrafficDensity'; elems(4).DataType = 'double';
BusRiskContext = Simulink.Bus;
BusRiskContext.Elements = elems;

% -------------------------------------------------------------------------
% 3. BusPlannerOutput: High-Frequency Continuous Replanning Output
% -------------------------------------------------------------------------
clear elems;
elems(1) = Simulink.BusElement; elems(1).Name = 'TargetVelocity'; elems(1).DataType = 'double';
elems(2) = Simulink.BusElement; elems(2).Name = 'TargetYawRate'; elems(2).DataType = 'double';
elems(3) = Simulink.BusElement; elems(3).Name = 'CommandedSteer'; elems(3).DataType = 'double';
elems(4) = Simulink.BusElement; elems(4).Name = 'CommandedAccel'; elems(4).DataType = 'double';
elems(5) = Simulink.BusElement; elems(5).Name = 'TacticalNudge'; elems(5).DataType = 'double';
elems(6) = Simulink.BusElement; elems(6).Name = 'RiskIndex'; elems(6).DataType = 'double';
BusPlannerOutput = Simulink.Bus;
BusPlannerOutput.Elements = elems;

% -------------------------------------------------------------------------
% 4. BusDecisionOutput: Stateflow 8-State Tactical Supervisor Output
% -------------------------------------------------------------------------
clear elems;
elems(1) = Simulink.BusElement; elems(1).Name = 'State'; elems(1).DataType = 'int32';
elems(2) = Simulink.BusElement; elems(2).Name = 'TargetSpeed'; elems(2).DataType = 'double';
elems(3) = Simulink.BusElement; elems(3).Name = 'TargetAccel'; elems(3).DataType = 'double';
elems(4) = Simulink.BusElement; elems(4).Name = 'TargetLateralOffset'; elems(4).DataType = 'double';
elems(5) = Simulink.BusElement; elems(5).Name = 'OvertakePhase'; elems(5).DataType = 'int32';
elems(6) = Simulink.BusElement; elems(6).Name = 'Horn'; elems(6).DataType = 'boolean';
elems(7) = Simulink.BusElement; elems(7).Name = 'HazardLights'; elems(7).DataType = 'boolean';
BusDecisionOutput = Simulink.Bus;
BusDecisionOutput.Elements = elems;

% -------------------------------------------------------------------------
% 5. BusControlCommand: Actuator Commands for Vehicle Dynamics Plant
% -------------------------------------------------------------------------
clear elems;
elems(1) = Simulink.BusElement; elems(1).Name = 'SteerCommand'; elems(1).DataType = 'double';
elems(2) = Simulink.BusElement; elems(2).Name = 'AccelCommand'; elems(2).DataType = 'double';
elems(3) = Simulink.BusElement; elems(3).Name = 'BrakePressure'; elems(3).DataType = 'double';
elems(4) = Simulink.BusElement; elems(4).Name = 'CrossTrackError'; elems(4).DataType = 'double';
elems(5) = Simulink.BusElement; elems(5).Name = 'HeadingError'; elems(5).DataType = 'double';
BusControlCommand = Simulink.Bus;
BusControlCommand.Elements = elems;

clear elems;

fprintf('=== Master Simulink Bus Setup Complete: All 5 System Buses Initialized ===\n');
fprintf('  1. BusVehicleState       (Odometry & Dynamics)\n');
fprintf('  2. BusRiskContext        (Road Context & IRRM Risk)\n');
fprintf('  3. BusPlannerOutput      (Continuous Replanning)\n');
fprintf('  4. BusDecisionOutput     (Stateflow 8-State Supervisor)\n');
fprintf('  5. BusControlCommand     (Stanley & Longitudinal Actuation)\n');
