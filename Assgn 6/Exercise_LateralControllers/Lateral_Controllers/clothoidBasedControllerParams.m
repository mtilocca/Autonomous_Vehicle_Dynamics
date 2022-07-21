function clothoidBasedParams = clothoidBasedControllerParams()

    % ----------------------------------------------------------------
    %% Function purpose: define the clothoid-based lateral controller parameters
    % ----------------------------------------------------------------
    
    % Look ahead distance (in meters) used by the controller
    clothoidBasedParams.lookAhead = 23; 
    
    % Understeering gradient
    clothoidBasedParams.Kus = -5.855245738485693e-04;  % 60 m/h 

                         % 60 km/h -- -5.114163963830335e-04
                          % 50 km /h--  -4.865648970989296e-04

                           % 80 km /h --  -5.855245738485693e-04
                           % 100 km /h --   -6.875897089988750e-04
                            % 30 km/h --  -4.710879256496287e-04
                            % 40 km/h --   -4.715285981810922e-04
                            % 20 km/h -- -5.062935410300344e-04     




end

