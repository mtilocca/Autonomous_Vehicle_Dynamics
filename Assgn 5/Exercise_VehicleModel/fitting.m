% ----------------------------------------------------------------
%% Handling diagram fitting
% ----------------------------------------------------------------

% For example, suppose you want to fit the diagram with a 2nd order polynomial:
% y = c_1*x + c_2*x^2, with x being the independent variable and y being
% the dependent variable. You need to replace x and y with the
% names of the arrays that you are using to fit the handling diagram, and
% of course you can add/remove terms from the polynomial of this example

% define the fitting law
fitting_law = @(c,x)(c(1)*x + c(2)*x.^2);  
% define the initial conditions
c0 = [0,0];  
% define the fitting algorithm and fitting options
options = optimoptions('lsqcurvefit','MaxFunctionEvaluations',50000,'MaxIterations',50000,'FunctionTolerance',1e-9,...
    'StepTolerance',1e-9,'Display','final-detailed');
% fit the data
[optim_coeffs,resnorm,~,exitflag,output] = lsqcurvefit(fitting_law,c0,x,y,[],[],options); 
% extract the optimal fitting coefficients
c_1 = optim_coeffs(1);
c_2 = optim_coeffs(2);
fitted_diagram = c_1*x + c_2*x.^2;

% Note: if the fitting is not good or you are not able to identify a proper
% fitting law for your data, you can use the Curve Fitting App of Matlab,
% which may also be used to obtain a good initial condition c0 from which
% to start the optimization process

% Plot the fitted diagram (together with the not fitted one)
% ...


