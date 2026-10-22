% RUN_DASHBOARD Root wrapper for launching BharatDrive-AI Telemetry Dashboard.
% Works whether called from root directory or matlab directory.

root_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(root_dir, 'matlab'));
addpath(fullfile(root_dir, 'matlab', 'scripts'));
addpath(fullfile(root_dir, 'simulink'));

app = run_dashboard();
if nargout > 0
    varargout{1} = app;
end
