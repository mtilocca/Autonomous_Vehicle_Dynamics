% ----------------------------------------------------------------
%% Main script for a basic simulation framework with a Formula SAE vehicle model 
%  authors: Mattia Piccinini & Gastone Pietro Papini Rosati
%  email:   mattia.piccinini@unitn.it & gastone.rosatipapini@unitn.it
%  date:    13/10/2020
% ----------------------------------------------------------------

% ----------------------------
%% Initialization
% ----------------------------
initialize_environment;

% ----------------------------
%% Load vehicle data
% ----------------------------
vehicle_data = getVehicleDataStruct();
pacejkaParam = loadPacejkaParam();

% ----------------------------
%% Longitudinal controller parameters
% ----------------------------
longCTRparam = longitController();

% ----------------------------
%% Define initial conditions for the simulation
% ----------------------------
X0 = loadInitialConditions;

% ----------------------------
%% Simulation parameters
% ----------------------------
simulationPars = getSimulationParams(); 
Ts = simulationPars.times.step_size;  % integration step for the simulation (fixed step)
T0 = simulationPars.times.t0;         % starting time of the simulation
Tf = simulationPars.times.tf;         % stop time of the simulation

%% Start Simulation
% ----------------------------
fprintf('Starting Simulation\n')
tic;
model_sim = sim('Vehicle_Model_2Track');
elapsed_time_simulation = toc;
fprintf('Simulation completed\n')
fprintf('The total simulation time was %.2f seconds\n',elapsed_time_simulation)

%% Post-Processing
% ----------------------------
dataAnalysis(model_sim,vehicle_data,Ts);

%% Handl diagram 

Hand_diag(model_sim.states.delta.Data, model_sim.states.Omega.Data, model_sim.states.u.Data, vehicle_data.vehicle.L );

%% Fitting  - uncomment this when plotting the fitted parts 


fitted_param = m_fitting (model_sim.states.delta.Data, model_sim.states.Omega.Data, model_sim.states.u.Data, vehicle_data.vehicle.L );

fitted_param(1)  % 50 km /h--  -4.865648970989296e-04
                           % 80 km /h --  -5.855245738485693e-04
                           % 100 km /h --   -6.875897089988750e-04
                            % 30 km/h --  -4.710879256496287e-04
                            % 40 km/h --   -4.715285981810922e-04
                            % 20 km/h -- -5.062935410300344e-04



                            % delta = 70 -- km/h = 50    coeff =
                            % -5.447293182859597e-04 


                            % delta = 24 -- km/h = 80 -- coef = -6.784774346266695e-04

                            % delta = 12 -- km/h = 100 -- coef =
                            % -7.496739104347664e-04 



                            % -6.211638280800491e-05