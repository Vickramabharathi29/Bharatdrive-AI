% STARTUP Initialization script at BharatDrive-AI workspace root.
% Configures MATLAB path for all modules, packages, and scripts.

root_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(root_dir, 'matlab'));
addpath(fullfile(root_dir, 'matlab', 'scripts'));
addpath(fullfile(root_dir, 'simulink'));
addpath(fullfile(root_dir, 'simulink', 'buses'));
addpath(fullfile(root_dir, 'simulink', 'models', 'blocks'));
addpath(fullfile(root_dir, 'config'));

fprintf('=== BharatDrive-AI Workspace Initialized ===\n');
fprintf('Root: %s\n', root_dir);
fprintf('Adaptive Path Planning and Dynamic Collision Avoidance on Unstructured Indian Roads\n');
fprintf('• Type "run_dashboard" to open the MATLAB App Designer Telemetry Interface.\n');
fprintf('• Type "run_evaluation_metrics" to display benchmark evaluation figures.\n');
fprintf('• Type "run_end_to_end_simulation" to execute closed-loop test runs.\n\n');
