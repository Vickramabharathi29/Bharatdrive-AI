% SETUP_CONTROLLER_BUSES Defines Simulink Bus Objects for Vehicle Motion Control.
% BharatDrive-AI - MathWorks Autonomous Driving Hackathon.

clear BusControlCommand;

elems(1) = Simulink.BusElement; elems(1).Name = 'SteerCommand'; elems(1).DataType = 'double';
elems(2) = Simulink.BusElement; elems(2).Name = 'AccelCommand'; elems(2).DataType = 'double';
elems(3) = Simulink.BusElement; elems(3).Name = 'BrakePressure'; elems(3).DataType = 'double';
elems(4) = Simulink.BusElement; elems(4).Name = 'CrossTrackError'; elems(4).DataType = 'double';
elems(5) = Simulink.BusElement; elems(5).Name = 'HeadingError'; elems(5).DataType = 'double';

BusControlCommand = Simulink.Bus;
BusControlCommand.Elements = elems;
clear elems;

fprintf('=== Simulink Controller Bus Object Initialized: BusControlCommand ===\n');
