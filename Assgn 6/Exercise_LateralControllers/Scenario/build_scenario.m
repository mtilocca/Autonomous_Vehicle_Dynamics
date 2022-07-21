function build_scenario()

    % ----------------------------------
    %% Define the layout of the scenario
    % ----------------------------------
   
    % Starting point 
    x_path = 0;
    y_path = 0;
    theta_path = 0;
    
    % ------------------
    % Initial straight line
    % ------------------
    length_initial_straight = 10;
    x_path = [x_path; length_initial_straight];
    y_path = [y_path; 0];
    theta_path = [theta_path; 0];
    
    % ------------------
    % S-shape path
    % ------------------
    x_path = [x_path; x_path(end)+10; x_path(end)+20];
    y_path = [y_path; y_path(end)+5; y_path(end)+10];
    theta_path = [theta_path; theta_path(end)+pi/3; 0];
    
    % ------------------
    % Final straight line
    % ------------------
    length_final_straight = 200;
    x_path = [x_path; x_path(end)+length_final_straight];
    y_path = [y_path; y_path(end)];
    theta_path = [theta_path; 0];
    
    % ------------------
    %% Plot the scenario 
    % ------------------
    % Build the clothoid list 
    S_clothoid = ClothoidList; 
    for ii=1:length(x_path)-1
        S_clothoid.push_back_G1(x_path(ii),y_path(ii),theta_path(ii), x_path(ii+1),y_path(ii+1),theta_path(ii+1));
    end
    % Evaluate the points belonging to the path (clothoid list)
    curv_absc_sample = 0:0.1:S_clothoid.length;
    [x_path_sample,y_path_sample,~,~] = S_clothoid.evaluate(curv_absc_sample);
    
    figure('Name','Scenario','NumberTitle','off'), clf  
    hold on
    axis equal
    S_clothoid.plot;
    grid on
    xlabel('x [m]')
    ylabel('y [m]')
    title('Target maneuver')
    
    % ------------------
    %% Save the points of the path
    % ------------------
    path.x = x_path;
    path.y = y_path;
    path.theta = theta_path;
    path.x_sampled = x_path_sample;
    path.y_sampled = y_path_sample;
%     save('./Scenario/scenario_test','path')
    
end