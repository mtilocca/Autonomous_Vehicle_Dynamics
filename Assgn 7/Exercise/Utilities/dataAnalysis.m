% ----------------------------------------------------------------
%% Post-Processing and Data Analysis
% ----------------------------------------------------------------

% ---------------------------------
%% Interface settings
% ---------------------------------
% set this to 1 to enable plots
enable_plots = 1;

% ---------------------------------
%% Load vehicle data
% ---------------------------------
Lf = vehicle_data.vehicle.Lf;  % [m] Distance between vehicle CoG and front wheels axle
Lr = vehicle_data.vehicle.Lr;  % [m] Distance between vehicle CoG and front wheels axle
L  = vehicle_data.vehicle.L;   % [m] Vehicle length
Wf = vehicle_data.vehicle.Wf;  % [m] Width of front wheels axle 
Wr = vehicle_data.vehicle.Wr;  % [m] Width of rear wheels axle                   
m  = vehicle_data.vehicle.m;   % [kg] Vehicle Mass
g  = vehicle_data.vehicle.g;   % [m/s^2] Gravitational acceleration
tau_D = vehicle_data.steering_system.tau_D;  % [-] steering system ratio (pinion-rack)

         
% ---------------------------------
%% Extract data from simulink model
% ---------------------------------
time_sim = model_sim.states.u.time;
dt = time_sim(2)-time_sim(1);

% -----------------
% Inputs
% -----------------
ped_0      = model_sim.inputs.ped_0.data;
delta_D    = squeeze(model_sim.inputs.delta_D.data);

% -----------------
% States
% -----------------
x_CoM      = model_sim.states.x.data;
y_CoM      = model_sim.states.y.data;
psi        = model_sim.states.psi.data;
u          = model_sim.states.u.data;
v          = model_sim.states.v.data;
Omega      = squeeze(model_sim.states.Omega.data);
Fz_rr      = model_sim.states.Fz_rr.data;
Fz_rl      = model_sim.states.Fz_rl.data;
Fz_fr      = model_sim.states.Fz_fr.data;
Fz_fl      = model_sim.states.Fz_fl.data;
delta      = squeeze(model_sim.states.delta.data);
omega_rr   = model_sim.states.omega_rr.data;
omega_rl   = model_sim.states.omega_rl.data;
omega_fr   = model_sim.states.omega_fr.data;
omega_fl   = model_sim.states.omega_fl.data;
alpha_rr   = model_sim.states.alpha_rr.data;
alpha_rl   = model_sim.states.alpha_rl.data;
alpha_fr   = model_sim.states.alpha_fr.data;
alpha_fl   = model_sim.states.alpha_fl.data;
kappa_rr   = model_sim.states.kappa_rr.data;
kappa_rl   = model_sim.states.kappa_rl.data;
kappa_fr   = model_sim.states.kappa_fr.data;
kappa_fl   = model_sim.states.kappa_fl.data;
ped        = model_sim.states.pedal.data;

% -----------------
% Extra Parameters
% -----------------
Tw_rr      = model_sim.extra_params.Tw_rr.data;
Tw_rl      = model_sim.extra_params.Tw_rl.data;
Tw_fr      = model_sim.extra_params.Tw_fr.data;
Tw_fl      = model_sim.extra_params.Tw_fl.data;
Fx_rr      = model_sim.extra_params.Fx_rr.data;
Fx_rl      = model_sim.extra_params.Fx_rl.data;
Fx_fr      = model_sim.extra_params.Fx_fr.data;
Fx_fl      = model_sim.extra_params.Fx_fl.data;
Fy_rr      = model_sim.extra_params.Fy_rr.data;
Fy_rl      = model_sim.extra_params.Fy_rl.data;
Fy_fr      = model_sim.extra_params.Fy_fr.data;
Fy_fl      = model_sim.extra_params.Fy_fl.data;
Mz_rr      = model_sim.extra_params.Mz_rr.data;
Mz_rl      = model_sim.extra_params.Mz_rl.data;
Mz_fr      = model_sim.extra_params.Mz_fr.data;
Mz_fl      = model_sim.extra_params.Mz_fl.data;
gamma_rr   = model_sim.extra_params.gamma_rr.data;
gamma_rl   = model_sim.extra_params.gamma_rl.data;
gamma_fr   = model_sim.extra_params.gamma_fr.data;
gamma_fl   = model_sim.extra_params.gamma_fl.data;
delta_fr   = model_sim.extra_params.delta_fr.data;
delta_fl   = model_sim.extra_params.delta_fl.data;

% Chassis side slip angle beta [rad]
beta = atan(v./u);

% -----------------
% Accelerations
% -----------------
% Derivatives of u, v [m/s^2]
dot_u = diff(u)/Ts;
dot_v = diff(v)/Ts;
% Total longitudinal and lateral accelerations
Ax = dot_u(1:end) - Omega(2:end).*v(2:end);
Ay = dot_v(1:end) + Omega(2:end).*u(2:end);
% Ax low-pass filtered signal (zero-phase digital low-pass filtering)
Wn_filter = 0.01;
[b_butt,a_butt] = butter(4,Wn_filter,'low');
Ax_filt = filtfilt(b_butt,a_butt,Ax);  
dot_u_filt = filtfilt(b_butt,a_butt,dot_u);  
% Steady state lateral acceleration
Ay_ss = Omega.*u;
% Longitudinal jerk [m/s^3]
jerk_x = diff(dot_u)/Ts;

% -----------------
% Other parameters
% -----------------
% Total CoM speed [m/s]
vG = sqrt(u.^2 + v.^2);
% Steady state and transient curvature [m]
rho_ss   = Omega./vG;
rho_tran = ((dot_v.*u(1:end-1) - dot_u.*v(1:end-1)) ./ ((vG(1:end-1)).^3)) + rho_ss(1:end-1);
% Desired sinusoidal steering angle for the equivalent single track front wheel
desired_steer_atWheel = delta_D/tau_D;

% -----------------
% Vehicle route fitted with clothoids
% -----------------
vehRoute = ClothoidList();
numOfClothoids_route = size(refRoute_fewPoints,1);
for i = 1:numOfClothoids_route-1
    vehRoute.push_back_G1(refRoute_fewPoints(i,1),refRoute_fewPoints(i,2),refRoute_fewPoints(i,3), refRoute_fewPoints(i+1,1),refRoute_fewPoints(i+1,2),refRoute_fewPoints(i+1,3)); 
end

% -----------------
% Data format manipulation
% -----------------
speed_request = squeeze(model_sim.req_speed.Data);

% ---------------------------------
%% PLOTS
% ---------------------------------

if (enable_plots)
    % -------------------------------
    %% PLOT MAIN STATES
    % -------------------------------
    figure('Name','States and controls','NumberTitle','off'), clf
    % --- u --- %
    ax(1) = subplot(221);
    plot(time_sim,u,'LineWidth',2)
    hold on
    plot(time_sim,speed_request,'--r','LineWidth',2)
    legend('real','desired','Location','best')
    grid on
    title('$u$ [m/s]')  
    xlim([0 time_sim(end)])
    % --- delta_0 --- %
    ax(2) = subplot(222);
    plot(time_sim,rad2deg(delta_D),'LineWidth',2)
    grid on
    title('$\delta_0$ [deg]')
    xlim([0 time_sim(end)])
    % --- Omega --- %
    ax(3) = subplot(223);
    plot(time_sim,Omega,'LineWidth',2)
    grid on
    title('$\Omega$ [rad/s]')
    xlim([0 time_sim(end)])

    % linkaxes(ax,'x')
    clear ax    

    % -------------------------------
    %% Plot vehicle pose x,y,psi
    % -------------------------------
    figure('Name','Pose','NumberTitle','off'), clf
    % --- x --- %
    ax(1) = subplot(221);
    plot(time_sim,x_CoM,'LineWidth',2)
    grid on
    title('$x$ [m]')
    xlim([0 time_sim(end)])
    % --- y --- %
    ax(2) = subplot(222);
    plot(time_sim,y_CoM,'LineWidth',2)
    grid on
    title('$y$ [m]')
    xlim([0 time_sim(end)])
    % --- psi --- %
    ax(3) = subplot(223);
    plot(time_sim,rad2deg(psi),'LineWidth',2)
    grid on
    title('$\psi$ [deg]')
    xlim([0 time_sim(end)])

    % linkaxes(ax,'x')
    clear ax
    
    % -------------------------------
    %% Plot vehicle path
    % -------------------------------
    N = length(time_sim);
    [x_route,y_route] = vehRoute.evaluate(0:0.1:vehRoute.length);
    
    figure('Name','Real Vehicle Path','NumberTitle','off'), clf
    set(gca,'fontsize',16)
    hold on
    axis equal
    xlabel('x-coord [m]')
    ylabel('y-coord [m]')
    title('Scenario')  %Real Vehicle Path
    % Plot road scenario
    for ii=1:size(mapObjects,1)
        rectangle('Position',mapObjects(ii,:),'FaceColor',color('purple'),'EdgeColor',color('purple'),'LineWidth',2)
    end
    %plot(costmap)
    % Plot interpolated reference route 
    plot(refRoute_fewPoints(:,1),refRoute_fewPoints(:,2),'go','MarkerFaceColor','g','MarkerSize',8) %,'DisplayName','Route'
    % Plot original (not interpolated) reference route 
    plot(refRoute_points_orig(:,1),refRoute_points_orig(:,2),'o','Color',color('orange'),'MarkerFaceColor',color('orange'),'MarkerSize',8) %,'DisplayName','Interpolated Route'
    % Plot vehicle CoM trajectory
    plot(x_CoM,y_CoM,'Color',color('gold'),'LineWidth',4)
    vehRoute.plot;
    for i = 1:floor(N/20):N
        rot_mat = [cos(psi(i)) -sin(psi(i)) ; sin(psi(i)) cos(psi(i))];
        pos_rr = rot_mat*[-Lr -Wr/2]';
        pos_rl = rot_mat*[-Lr +Wr/2]';
        pos_fr = rot_mat*[+Lf -Wf/2]';
        pos_fl = rot_mat*[+Lf +Wf/2]';
        pos = [pos_rr pos_rl pos_fl pos_fr];
        p = patch(x_CoM(i) + pos(1,:),y_CoM(i) + pos(2,:),'blue');
        quiver(x_CoM(i), y_CoM(i), u(i)*cos(psi(i)), u(i)*sin(psi(i)), 'color', [1,0,0]);
        quiver(x_CoM(i), y_CoM(i), -v(i)*sin(psi(i)), v(i)*cos(psi(i)), 'color', [0.23,0.37,0.17]);
    end
    plot(5,5,'mo','MarkerSize',6,'MarkerFaceColor','m')
    plot(196,134.5,'go','MarkerSize',6,'MarkerFaceColor','g')
    text(8,5,'A','FontSize',16);
    text(187,134.5,'B','FontSize',16);
    grid on
    legend('interpolating points','route','vehicle path','clothoid fitting','clothoid fitting','location','best')
%     %% Path tracking error 
%     %-------------------------------------------------------------
%     trajectory = [x_CoM y_CoM];
%     rrt_planned = [x_route'  y_route'];
%     npoint = dsearchn(rrt_planned, trajectory);
%     differ = rrt_planned(npoint,:) - trajectory; 
%     pathE = hypot( differ(: ,1), differ(: ,2));
%     pathE = smoothdata(pathE);
%     figure('Name', 'Path tracking error', 'NumberTitle','off'), clf
%     hold on 
%     xlabel('time [s]'); ylabel('Error [m]'); title('Path Tracking Error');
%     plot(time_sim, pathE, 'r'); grid on ;
%     hold off 

end    


% -------------------------------
%% Compute the final time
% -------------------------------
parking_lot = mapObjects(1:2,:);
for ii=1:length(time_sim)
    if (x_CoM(ii)>=parking_lot(1,1) && y_CoM(ii)>=parking_lot(2,2) && y_CoM(ii)<=parking_lot(1,2))
        final_time = ii*Ts;
        break;
    end
end
endOfCircuit_idx = ii;


% -------------------------------
%% Path tracking error
% -------------------------------
%desired_path = [x_route' y_route'];

% Error between real and desired trajectories
% COMPLETE WITH YOUR CODE
    trajectory = [x_CoM y_CoM];
    rrt_planned = [x_route'  y_route'];
    npoint = dsearchn(rrt_planned, trajectory);
    differ = rrt_planned(npoint,:) - trajectory; 
    pathE = hypot( differ(: ,1), differ(: ,2));
    pathE = smoothdata(pathE);
    figure('Name', 'Path tracking error', 'NumberTitle','off'), clf
    hold on 
    xlabel('time [s]'); ylabel('Error [m]'); title('Path Tracking Error');
    plot(time_sim, pathE, 'r'); grid on ;
    xlim([0 final_time]); 
    hold off 
% -------------------------------
%% Messages to display for the user
% -------------------------------
if (exist('final_time','var'))
    fprintf('The scenario is compled by the vehicle in %.3f seconds\n',final_time);
else
    fprintf('The vehicle did not manage to complete the scenario!\n');
end


