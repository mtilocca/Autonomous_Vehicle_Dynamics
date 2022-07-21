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

% Set this flag to 0 to disable route planning and load a precomputed route
% to decrease simulation time
enable_routePlan = 0; % set it to 1 

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
simulationPars = getSimulationParams(); % might need to be incremented 
Ts = simulationPars.times.step_size;  % integration step for the simulation (fixed step)
T0 = simulationPars.times.t0;         % starting time of the simulation
Tf = simulationPars.times.tf;         % stop time of the simulation

% ----------------------------
%% Load lateral controller parameters
% ----------------------------
LLC = load_LowLevelControlData();
LLC_sampleTime = LLC.sample_time;

purePursuitParams   = purePursuitControllerParams();
stanleyParams       = stanleyControllerParams(); % bonus assignment from assignment 6 
clothoidBasedParams = clothoidBasedControllerParams();

% ----------------------------
%% Select lateral controller type
% ----------------------------
select_lateralController;

% ----------------------------
%% Load road scenario
% ----------------------------
scenario = load('./Scenario/scenario');
costmap = scenario.scenario_data.costmap;
mapObjects = scenario.scenario_data.mapObjects;
vehicleDims = scenario.scenario_data.vehicleDims;

% ----------------------------
%% Define graphical interface settings
% ----------------------------
% Set this flag to 1 in order to enable online plots during the simulation
enable_onlinePlots =0;
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
if (enable_routePlan)
    % Perform route planning
    routePlanner;
else
    % Load a precomputed route to decrease simulation time
    referencePath_points_original = load('refRoute_poses_RRT'); % RRT* solution
    referencePath_fewPoints = load('refPath_poses_fewPoints');  % interpolated RRT* solution with few points
    cpuTime_routePlan = load('cpuTime_routePlan');         
    refRoute_points_orig = referencePath_points_original.refRoute_points_orig;
    refRoute_fewPoints   = referencePath_fewPoints.refRoute_fewPoints;
    elapsed_time_routePlan = cpuTime_routePlan.elapsed_time_routePlan;    
end
% Simulink simulation
model_sim = sim('framework_sim.slx');
elapsed_time_simulation = toc; 
fprintf('Simulation completed\n')
total_simul_time = elapsed_time_simulation + elapsed_time_routePlan;
fprintf('It took %.1f seconds to compute the route with RRT*\n',elapsed_time_routePlan)
fprintf('The total simulation time is %.1f seconds\n',total_simul_time)

% ----------------------------
%% Post-Processing
% ----------------------------
dataAnalysis;  %0.2968  lh = 8 
