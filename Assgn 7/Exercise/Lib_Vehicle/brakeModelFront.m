function [Tw__fr,Tw__fl] = brakeModelFront(reqBrakeTorque,omega__fr,omega__fl,params)

    % ----------------------------------------------------------------
    %% Function purpose: compute braking torques at front wheels with brake model
    % ----------------------------------------------------------------

    max_brake_torque_front = params.braking.max_brake_torque_front;
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
    if (abs(reqBrakeTorque) < abs(max_brake_torque_front))
        Tw__fr = reqBrakeTorque*sin(atan(omega__fr/regularSignScale));
        Tw__fl = reqBrakeTorque*sin(atan(omega__fl/regularSignScale));
    else
        Tw__fr = -max_brake_torque_front*sin(atan(omega__fr/regularSignScale));
        Tw__fl = -max_brake_torque_front*sin(atan(omega__fl/regularSignScale));
    end

end

