function simulation = getSimulationParams()
 
    % Timings
    times.t0        = 0;     % [s]  <--- starting time
    times.step_size = 0.001; % [s]  <--- discrete solver step
    times.tf        = 20;    % [s]  <--- stop simulation time
    
    simulation.times = times;

end