function model_name = build_stateflow_model()
% BUILD_STATEFLOW_MODEL Programmatically generates the Stateflow Decision Logic
% Simulink model for BharatDrive-AI.
%
% Generates StateflowDecisionSystem.slx with discrete execution at 10 Hz (Ts = 0.1s).

    model_name = 'StateflowDecisionSystem';
    fprintf('====================================================================\n');
    fprintf('   BharatDrive-AI: Simulink Stateflow Decision Model Builder        \n');
    fprintf('====================================================================\n');

    % Setup Bus Objects
    bus_script = fullfile(fileparts(mfilename('fullpath')), '..', '..', 'simulink', 'buses', 'setup_decision_buses.m');
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

    % Solver configuration
    set_param(model_name, 'SolverType', 'Fixed-step');
    set_param(model_name, 'Solver', 'FixedStepDiscrete');
    set_param(model_name, 'FixedStep', '0.1');
    set_param(model_name, 'StopTime', '30.0');

    % Add Inports
    add_block('simulink/Sources/In1', [model_name, '/EgoVehicleState'], ...
        'Position', [60, 80, 90, 94], 'OutDataTypeStr', 'Bus: BusVehicleState');

    add_block('simulink/Sources/In1', [model_name, '/EnvironmentalRiskContext'], ...
        'Position', [60, 150, 90, 164], 'OutDataTypeStr', 'Bus: BusRiskContext');

    add_block('simulink/Sources/In1', [model_name, '/ThreatObstacle'], ...
        'Position', [60, 220, 90, 234], 'OutDataTypeStr', 'double', 'PortDimensions', '[1 7]');

    add_block('simulink/Sources/In1', [model_name, '/EnvironmentalSignals'], ...
        'Position', [60, 290, 90, 304], 'OutDataTypeStr', 'double', 'PortDimensions', '[1 4]');

    % Add Stateflow / MATLAB Function Block
    sf_block = [model_name, '/Stateflow_Decision_Supervisor'];
    add_block('simulink/User-Defined Functions/MATLAB Function', sf_block, ...
        'Position', [250, 140, 520, 260]);

    % Populate block script
    sf = sfroot;
    m = sf.find('Name', model_name, '-isa', 'Stateflow.Machine');
    if ~isempty(m)
        charts = m.find('-isa', 'Stateflow.EMChart');
        if ~isempty(charts)
            func_file = fullfile(output_dir, 'blocks', 'block_stateflow_decision_logic.m');
            if exist(func_file, 'file')
                fid = fopen(func_file, 'r');
                charts(1).Script = fread(fid, '*char')';
                fclose(fid);
            end
        end
    end

    % Add Bus Creator for Decision Output
    add_block('simulink/Signal Routing/Bus Creator', [model_name, '/BusCreator_DecisionOutput'], ...
        'Position', [620, 130, 630, 270], 'Inputs', '7', 'OutDataTypeStr', 'Bus: BusDecisionOutput');

    % Add Outport
    add_block('simulink/Sinks/Out1', [model_name, '/DecisionCommandBus'], ...
        'Position', [700, 190, 730, 204], 'OutDataTypeStr', 'Bus: BusDecisionOutput');

    % Connect lines
    try
        add_line(model_name, 'EgoVehicleState/1', 'Stateflow_Decision_Supervisor/1', 'autorouting', 'on');
        add_line(model_name, 'EnvironmentalRiskContext/1', 'Stateflow_Decision_Supervisor/2', 'autorouting', 'on');
        add_line(model_name, 'ThreatObstacle/1', 'Stateflow_Decision_Supervisor/3', 'autorouting', 'on');
        add_line(model_name, 'EnvironmentalSignals/1', 'Stateflow_Decision_Supervisor/4', 'autorouting', 'on');

        add_line(model_name, 'Stateflow_Decision_Supervisor/1', 'BusCreator_DecisionOutput/1', 'autorouting', 'on');
        add_line(model_name, 'Stateflow_Decision_Supervisor/2', 'BusCreator_DecisionOutput/2', 'autorouting', 'on');
        add_line(model_name, 'Stateflow_Decision_Supervisor/3', 'BusCreator_DecisionOutput/3', 'autorouting', 'on');
        add_line(model_name, 'Stateflow_Decision_Supervisor/4', 'BusCreator_DecisionOutput/4', 'autorouting', 'on');
        add_line(model_name, 'Stateflow_Decision_Supervisor/5', 'BusCreator_DecisionOutput/5', 'autorouting', 'on');
        add_line(model_name, 'Stateflow_Decision_Supervisor/6', 'BusCreator_DecisionOutput/6', 'autorouting', 'on');
        add_line(model_name, 'Stateflow_Decision_Supervisor/7', 'BusCreator_DecisionOutput/7', 'autorouting', 'on');

        add_line(model_name, 'BusCreator_DecisionOutput/1', 'DecisionCommandBus/1', 'autorouting', 'on');
    catch ME
        fprintf('Note during line connection: %s\n', ME.message);
    end

    save_system(model_name, model_path);
    fprintf('Stateflow Decision Simulink Model saved to:\n  %s\n', model_path);
end
