% SETUP_DECISION_BUSES Defines Simulink Bus Objects for Stateflow Decision Logic.
% BharatDrive-AI - MathWorks Autonomous Driving Hackathon.

clear BusDecisionOutput;

elems(1) = Simulink.BusElement; elems(1).Name = 'State'; elems(1).DataType = 'int32';
elems(2) = Simulink.BusElement; elems(2).Name = 'TargetSpeed'; elems(2).DataType = 'double';
elems(3) = Simulink.BusElement; elems(3).Name = 'TargetAccel'; elems(3).DataType = 'double';
elems(4) = Simulink.BusElement; elems(4).Name = 'TargetLateralOffset'; elems(4).DataType = 'double';
elems(5) = Simulink.BusElement; elems(5).Name = 'OvertakePhase'; elems(5).DataType = 'int32';
elems(6) = Simulink.BusElement; elems(6).Name = 'Horn'; elems(6).DataType = 'boolean';
elems(7) = Simulink.BusElement; elems(7).Name = 'HazardLights'; elems(7).DataType = 'boolean';

BusDecisionOutput = Simulink.Bus;
BusDecisionOutput.Elements = elems;
clear elems;

fprintf('=== Simulink Decision Bus Object Initialized: BusDecisionOutput ===\n');
