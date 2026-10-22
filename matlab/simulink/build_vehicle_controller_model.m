function model_name = build_vehicle_controller_model()
% BUILD_VEHICLE_CONTROLLER_MODEL Programmatically generates the Vehicle Motion
% Controller Simulink model for BharatDrive-AI.
%
% Builds VehicleControllerSystem.slx running at 100 Hz (Ts = 0.01s).

    model_name = 'VehicleControllerSystem';
    fprintf('====================================================================\n');
    fprintf('   BharatDrive-AI: Simulink Vehicle Controller Model Builder        \n');
    fprintf('====================================================================\n');

    % Setup Bus Objects
    bus_script = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'simulink', 'buses', 'setup_controller_buses.m');
    if exist(bus_script, 'file')
        run(bus_script);
    end
    planner_bus_script = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'simulink', 'buses', 'setup_planner_buses.m');
    if exist(planner_bus_script, 'file')
        run(planner_bus_script);
    end

    output_dir = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'simulink', 'models');
    if ~exist(output_dir, 'dir'), mkdir(output_dir); end
    model_path = fullfile(output_dir, [model_name, '.slx']);

    has_simulink = exist('new_system', 'file') && license('test', 'Simulink');
    if ~has_simulink
        fprintf('Simulink license not detected or running in headless mode.\n');
        fprintf('Exporting block definitions to %s...\n', fullfile(output_dir, 'blocks'));
        return;
    end

    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end

    new_system(model_name);
    open_system(model_name);

    % Solver configuration: 100 Hz fixed-step discrete
    set_param(model_name, 'SolverType', 'Fixed-step');
    set_param(model_name, 'Solver', 'FixedStepDiscrete');
    set_param(model_name, 'FixedStep', '0.01');
    set_param(model_name, 'StopTime', '30.0');

    % Add Inports
    add_block('simulink/Sources/In1', [model_name, '/EgoVehicleState'], ...
        'Position', [60, 80, 90, 94], 'OutDataTypeStr', 'Bus: BusVehicleState');

    add_block('simulink/Sources/In1', [model_name, '/PlannerOutput'], ...
        'Position', [60, 160, 90, 174], 'OutDataTypeStr', 'Bus: BusPlannerOutput');

    add_block('simulink/Sources/In1', [model_name, '/TrajectoryWaypoints'], ...
        'Position', [60, 240, 90, 254], 'OutDataTypeStr', 'double', 'PortDimensions', '[20 3]');

    % Add Stanley Controller MATLAB Function Block
    ctrl_sub = [model_name, '/Stanley_Motion_Controller'];
    add_block('simulink/User-Defined Functions/MATLAB Function', ctrl_sub, ...
        'Position', [250, 120, 520, 240]);

    % Populate block script
    sf = sfroot;
    m = sf.find('Name', model_name, '-isa', 'Stateflow.Machine');
    if ~isempty(m)
        charts = m.find('-isa', 'Stateflow.EMChart');
        if ~isempty(charts)
            func_file = fullfile(output_dir, 'blocks', 'block_stanley_controller.m');
            if exist(func_file, 'file')
                fid = fopen(func_file, 'r');
                charts(1).Script = fread(fid, '*char')';
                fclose(fid);
            end
        end
    end

    % Add Bus Creator for Control Command
    add_block('simulink/Signal Routing/Bus Creator', [model_name, '/BusCreator_ControlCommand'], ...
        'Position', [620, 120, 630, 250], 'Inputs', '5', 'OutDataTypeStr', 'Bus: BusControlCommand');

    % Add Outport
    add_block('simulink/Sinks/Out1', [model_name, '/ControlCommandBus'], ...
        'Position', [700, 175, 730, 189], 'OutDataTypeStr', 'Bus: BusControlCommand');

    % Connect lines
    try
        add_line(model_name, 'EgoVehicleState/1', 'Stanley_Motion_Controller/1', 'autorouting', 'on');
        add_line(model_name, 'PlannerOutput/1', 'Stanley_Motion_Controller/2', 'autorouting', 'on');
        add_line(model_name, 'TrajectoryWaypoints/1', 'Stanley_Motion_Controller/3', 'autorouting', 'on');

        add_line(model_name, 'Stanley_Motion_Controller/1', 'BusCreator_ControlCommand/1', 'autorouting', 'on');
        add_line(model_name, 'Stanley_Motion_Controller/2', 'BusCreator_ControlCommand/2', 'autorouting', 'on');
        add_line(model_name, 'Stanley_Motion_Controller/3', 'BusCreator_ControlCommand/3', 'autorouting', 'on');
        add_line(model_name, 'Stanley_Motion_Controller/4', 'BusCreator_ControlCommand/4', 'autorouting', 'on');
        add_line(model_name, 'Stanley_Motion_Controller/5', 'BusCreator_ControlCommand/5', 'autorouting', 'on');

        add_line(model_name, 'BusCreator_ControlCommand/1', 'ControlCommandBus/1', 'autorouting', 'on');
    catch ME
        fprintf('Note during line connection: %s\n', ME.message);
    end

    save_system(model_name, model_path);
    fprintf('Vehicle Controller Simulink Model saved to:\n  %s\n', model_path);
end
