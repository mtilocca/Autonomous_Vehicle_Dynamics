% ----------------------------------------------------------------
%% Select the desired lateral controller
% ----------------------------------------------------------------

% -------------------
% Selection logic
%   o latContr_select = 1 --> arc path following
%   o latContr_select = 2 --> Stanley kinematic
%   o latContr_select = 3 --> Stanley dynamic % quite some error 
%   o latContr_select = 4 --> clothoid-based % okayish
% -------------------
latContr_select = 1;