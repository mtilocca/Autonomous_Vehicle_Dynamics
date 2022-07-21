function clothoidBasedParams = clothoidBasedControllerParams()

    % ----------------------------------------------------------------
    %% Function purpose: define the clothoid-based lateral controller parameters
    % ----------------------------------------------------------------
    
    % Look ahead distance (in meters) used by the controller
    clothoidBasedParams.lookAhead = 14; 
    
    % Understeering gradient
    clothoidBasedParams.Kus = -4.715285981810922e-04 ;  % 20 km/h 
                                                    
    % -4.715285981810922e-04 --  40 km/h 


end

