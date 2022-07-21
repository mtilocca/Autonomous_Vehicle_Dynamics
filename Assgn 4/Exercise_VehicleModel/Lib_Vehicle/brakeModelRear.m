function [Tw__rr,Tw__rl] = brakeModelRear(reqBrakeTorque,omega__rr,omega__rl,params)

    % ----------------------------------------------------------------
    %% Function purpose: compute braking torques at rear wheels with brake model
    % ----------------------------------------------------------------
    
    max_brake_torque_rear = params.braking.max_brake_torque_rear;
    regularSignScale = params.braking.regularSignScale;
    
    % Check that the braking torques have correctly been specified as
    % negative quantities. Otherwise, correct them by changing the sign
    if (reqBrakeTorque > 0)
        reqBrakeTorque = - reqBrakeTorque;
    end
    
    % Check that the requested braking torque is lower than the one that
    % the hydraulic system can apply. 
    % Use regularized sign functions (sin(atan(.))) to make sure that the 
    % vehicle correctly stops at zero forward speed, to avoid that negative 
    % speed values could be reached during braking
    if (abs(reqBrakeTorque) < abs(max_brake_torque_rear))
        Tw__rr = reqBrakeTorque*sin(atan(omega__rr/regularSignScale));
        Tw__rl = reqBrakeTorque*sin(atan(omega__rl/regularSignScale));
    else
        Tw__rr = -max_brake_torque_rear*sin(atan(omega__rr/regularSignScale));
        Tw__rl = -max_brake_torque_rear*sin(atan(omega__rl/regularSignScale));
    end

end

