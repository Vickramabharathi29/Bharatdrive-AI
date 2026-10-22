% STARTUP Initialization script for BharatDrive-AI MATLAB workspace.
% Adds all MATLAB module subdirectories to the MATLAB path.

project_root = fileparts(mfilename('fullpath'));
addpath(project_root);
addpath(fullfile(project_root, '+bharatdrive'));
addpath(fullfile(project_root, 'simulink'));
addpath(fullfile(project_root, 'scripts'));

fprintf('=== BharatDrive-AI MATLAB Workspace Initialized ===\n');
fprintf('Adaptive Path Planning and Collision Avoidance for Unstructured Indian Roads\n');
fprintf('Modules loaded: BharatDrive Models, Frenet Geometry, Configuration Engine, Dashboard.\n');
fprintf('Type "run_dashboard" to open the MATLAB App Designer Interface.\n');
