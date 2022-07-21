function finalPose = loadFinalPose()
    
    %% ICs for the vehicle pose
    xF     = 196;     % [m] final x-coordinate of the vehicle CoM
    yF     = 134.5;   % [m] final y-coordinate of the vehicle CoM
    thetaF = 0;       % [rad] final vehicle attitude
    
    finalPose = [xF,yF,thetaF];
       
end

