% ----------------------------------------------------------------
%% Main script for self-driving vehicle control framework 
%  authors: Mattia Piccinini & Gastone Pietro Papini Rosati
%  emails:  mattia.piccinini@unitn.it, gastone.rosatipapini@unitn.it
%  date:    26/11/2020
% ----------------------------------------------------------------

% ----------------------------
%% Initialization
% ----------------------------
initialize_environment;

% ----------------------------
%% Load vehicle data
% ----------------------------
vehicle_data = getVehicleDataStruct();
tau_D = vehicle_data.steering_system.tau_D;
pacejkaParam = loadPacejkaParam();

% ----------------------------
%% Define initial conditions for the simulation
% ----------------------------
X0 = loadInitialConditions;

% ----------------------------
%% Longitudinal controller parameters
% ----------------------------
longCTRparam = longitController();

% ----------------------------
%% Simulation parameters
% ----------------------------
simulationPars = getSimulationParams(); 
Ts = simulationPars.times.step_size;  % integration step for the simulation (fixed step)
T0 = simulationPars.times.t0;         % starting time of the simulation
Tf = simulationPars.times.tf;         % stop time of the simulation

% ----------------------------
%% Load lateral controller parameters
% ----------------------------
LLC = load_LowLevelControlData();
LLC_sampleTime = LLC.sample_time;

purePursuitParams   = purePursuitControllerParams();
stanleyParams       = stanleyControllerParams();
clothoidBasedParams = clothoidBasedControllerParams();

% ----------------------------
%% Select lateral controller type
% ----------------------------
select_lateralController;

% ----------------------------
%% Load road scenario
% ----------------------------
road_path = load('./Scenario/scenario_test');
road_data = [road_path.path.x, road_path.path.y, road_path.path.theta];
road_data_sampled = [road_path.path.x_sampled', road_path.path.y_sampled'];

% ----------------------------
%% Define graphical interface settings
% ----------------------------
% Set this flag to 1 in order to enable online plots during the simulation
enable_onlinePlots = 0;
% Set this flag to 1 in order to enable a zoomed view in online simulation plots
enable_zoom = 1;

if (enable_onlinePlots)
    % Initialize figure for online plots
    figure('Name','Road Scenario','NumberTitle','off')
    grid on
    axis equal
    xlabel('x [m]')
    ylabel('y [m]')
    title('Road Scenario')
    hold on
end

% ----------------------------
%% Start Simulation
% ----------------------------
fprintf('Starting Simulation\n')
tic;
% Simulink simulation
model_sim = sim('framework_sim.slx');
total_simul_time = toc; 
fprintf('Simulation completed\n')
fprintf('The total simulation time was %.1f seconds\n',total_simul_time)

% ----------------------------
%% Post-Processing
% ----------------------------
postProcessing(model_sim,vehicle_data,Ts,road_data_sampled);



