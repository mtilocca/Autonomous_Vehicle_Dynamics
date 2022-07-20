% -------------------------------------
%% Pure longitudinal slip conditions
% -------------------------------------

clc
clear all
close all   

% set (LaTeX as default interpreter for axis labels, ticks and legends)
% set(0,'defaulttextinterpreter','latex')
% set(groot, 'defaultAxesTickLabelInterpreter','latex');
% set(groot, 'defaultLegendInterpreter','latex');
% 
% set(0,'DefaultFigureWindowStyle','docked');
% set(0,'defaultAxesFontSize',20)
% set(0,'DefaultLegendFontSize',20)

% ------------------
%% Load the raw tyre data
% ------------------
load('B1464run30'); 

disp(testid)
disp(tireid)
%% Plot the data
figure(1);
subplot(5,1, 1);
plot(SL);
grid on 
title('longitudinal slip')
subplot(5,1,2);
plot(SA, 'k');
grid on 
title('side slip')

subplot(5,1,3);
plot(IA);
grid on 
title('camber angle');
subplot(5,1,4);
plot(FZ);
grid on 
title('Vertical tire force')
subplot(5,1,5);
plot(P);
grid on 
title('Pressure')
%% Organize the data
%   1. create a vector of idxs dividing the dataset by parameters
DT = horzcat(FX, FY, FZ, MZ, SA, SL, P, IA);

%Ia = gamma 
% SA = alpha 

FX1 = FX; FX2 = FX; FX3 = FX; 
FZ1 = FZ;
SL1 = SL; SL2 = SL; SL3 = SL;
RowtoD = abs(SA)> 0.04 | IA > 0.04; % range
RowtoD1 = IA > 0.04 | abs(SA) < 2.8 | abs(SA) > 3.1;
RowtoD2 = IA > 0.04 | abs(SA) < 5.8 | abs(SA) > 6.1;

FX1(RowtoD) = []; FX2(RowtoD1) = [];FX3(RowtoD2) = [];
FZ1(RowtoD) = [];
SL1(RowtoD) = []; SL2(RowtoD1) = []; SL3(RowtoD2) = []; 
% deleting the rows where alpha and gamma are non zero

% 670 load % 950 load % 220 load % 800 load % 
FZ1 = abs(FZ1); 
L1 =  FZ1 < 610 | FZ1 > 722 ; L2 =  FZ1 < 942 | FZ1 > 1195 ; L3 =  FZ1 < 190 | FZ1 > 240 ; L4 =  FZ1 < 810 | FZ1 > 930 ; 
FXG1 = FX1; FXG2 = FX1; FXG3 = FX1; FXG4 = FX1; 
FXG1(L1) = [] ;  FXG2(L2) = [];  FXG3(L3) = [];  FXG4(L4) = []; 
SLG1 = SL1; SLG2 = SL1; SLG3 = SL1; SLG4 = SL1;
SLG1(L1) = []; SLG2(L2) = []; SLG3(L3) = []; SLG4(L4) = []; 

figure(2);
plot(SLG1, FXG1);
grid on;
hold on ;
plot(SLG2, FXG2, '-r');
plot(SLG3, FXG3, '-k');
plot(SLG4, FXG4, '-m');
legend('F_z = 670 N','F_z = 950 N', 'F_z = 220', 'F_z = 800 N', 'Location','southeast'); 
title('F_x vs \kappa');


FX2(L1)= []; FX3(L1) = []; 
SL2(L1) = []; SL3(L1) = [];

figure(3) 
plot(SLG1, FXG1, '-r '); % alpha 0 deg
grid on;
hold on;
plot(SL2, FX2, '-k');% alpha 3 deg
plot(SL3, FX3) % alpha 6 deg
legend('\alpha = 0 deg','\alpha = 3 deg', '\alpha = 6 deg', 'Location','southeast'); 
title('longitudinal tire force as function of \alpha');
        
%   2. Perfom some plot (es. k VS Fx for all Fz, with gamma = 0, alpha = 0)
%% Execute the first fitting

%   1. Select the data alpha = 0, gamma = 0, Fz=Fz_nom
RDP = P < 78.000 | abs(SA)> 0.04 | IA > 0.04 | abs(FZ) < 810 | abs(FZ) > 930; % select rows to delete 
P1 = P; P1(RDP) = []; % filter the pressure 
FZF = abs(FZ); FZF(RDP) =[];% filter FZ0
SLF = SL; SLF(RDP) = []; % filter k 
FXF = FX; FXF(RDP) =[]; %filter fx

%   2. Write Pacejka MC and fit the data   

X0 = zeros(1,7); 
X0 = [0.0006 0.0001 0.0002 0.0001 0.0001 0.0005 0.00008];
IA0 = 0; FZ0 = -890; 


Xop1 = fmincon(@(X)resid_pure_Fx(X,FXF,SLF,IA0, FZF),X0, [], [], [], [], [], []);


Fx0F1 = FxP(Xop1, FXF,SLF,IA0, FZF); 

figure(4);
plot(SLF, FXF, '-r');
hold on 
grid on 
plot(SLF, Fx0F1, '-b'); 
legend('raw data','fitted data' ,'Location','southeast'); 
title('First Fitting');


%   3. Get the parameter
%% Execute the second fitting
%   1. Select the data for Fz=change and gamma = 0

RDP2 = P < 78.000 | abs(SA)> 0.04 | IA > 0.04 | abs(FZ) < 610 | abs(FZ) > 722 ; % select rows to delete 
P2 = P; P2(RDP2) = []; % filter the pressure 
FZF2 = abs(FZ); FZF2(RDP2) =[];% filter FZ0
SLF2 = SL; SLF2(RDP2) = []; % filter k 
FXF2 = FX; FXF2(RDP2) =[]; %filter fx
% second FZ parameters -----------------------------------------------------

RDP3 = P < 78.000 | abs(SA)> 0.04 | IA > 0.04 | abs(FZ) < 190 | abs(FZ) > 240 ; % select rows to delete 
P3 = P; P3(RDP3) = []; % filter the pressure 
FZF3 = abs(FZ); FZF3(RDP3) =[];% filter FZ0
SLF3 = SL; SLF3(RDP3) = []; % filter k 
FXF3 = FX; FXF3(RDP3) =[]; %filter fx


% third FZ parameters -----------------------------------------------------

RDP4 = P < 78.000 | abs(SA)> 0.04 | IA > 0.04 | abs(FZ) < 942 | abs(FZ) > 1195 ; % select rows to delete 
P4 = P; P4(RDP4) = []; % filter the pressure 
FZF4 = abs(FZ); FZF4(RDP4) =[];% filter FZ0
SLF4 = SL; SLF4(RDP4) = []; % filter k 
FXF4 = FX; FXF4(RDP4) =[]; %filter fx


X2F =  zeros(1,7); 
X2F= [0.01 0.05 0.006 0.008 0.002 0.004 0.002];
FXM = [FXF FXF2(1:486) FXF3(1:486) FXF4(1:486)]; 
FZM = [FZF FZF(1:486)  FZF3(1:486)  FZF4(1:486) ];
SLFM = [SLF SLF2(1:486)  SLF3(1:486)  SLF4(1:486) ];

Xop2 = fmincon(@(X)resid_pure_Fx_varFz(X,FXM,SLFM,IA0, FZM, Xop1),X2F, [], [], [], [], [], []);

Xplot2 = horzcat(Xop1,Xop2)

                %FxP2(X,FX,K,gamma, Fz)
Fx0F2 = FxP2(Xplot2, FXM,SLFM,IA0, FZM); 

figure(5);
plot(SLFM, FXM, 'b');
hold on 
grid on 
plot(SLFM, Fx0F2, 'k'); 

%Fx0F2 = resid_pure_Fx_varFz(X,FXM,SLFM,IA0, FZM, Xop1, a)
%   2. Fit the data using the previus parameters
%   4. Get the new parameters
%% Execute the third fitting
%   1. Select the data for Fz=-890 and gamma = 0
RDP31 = P < 78.000 | abs(SA)> 0.04 | IA > 0.04 | abs(FZ) < 810 | abs(FZ) > 930 ; % select rows to delete 
P5 = P; P5(RDP31) = []; % filter the pressure 
FzTF = -890;
SLF31 = SL; SLF31(RDP31) = []; % filter k 
FXF31 = FX; FXF31(RDP31) =[]; %filter fx
IA31 = IA; IA31(RDP31) = [];

% ------- gamma = 2 -------- %
RDP32 = P < 78.000 | abs(SA)> 0.04 | IA < 1.98 | IA > 2.06 | abs(FZ) < 810 | abs(FZ) > 930 ; % select rows to delete 
P6 = P; P6(RDP32) = []; % filter the pressure 

SLF32 = SL; SLF32(RDP32) = []; % filter k 
FXF32 = FX; FXF32(RDP32) =[]; %filter fx
IA32 = IA; IA32(RDP32) = [];

% ------ gamma = 4 ------ % 

RDP33 = P < 78.000 | abs(SA)> 0.04 | IA < 3.95 | IA > 4.05 | abs(FZ) < 810 | abs(FZ) > 930 ; % select rows to delete 
P7 = P; P7(RDP33) = []; % filter the pressure 
FzTF = -890;
SLF33 = SL; SLF33(RDP33) = []; % filter k 
FXF33 = FX; FXF33(RDP33) =[]; %filter fx
IA33 = IA; IA33(RDP33) = [];


X3F = zeros(1,1);
X3F(1) = 0.02;

FXM3 = [FXF31; FXF32; FXF33];
SLFM3 =[SLF31; SLF32; SLF33];
IAM3 =[IA31; IA32; IA33]; 


Xop3 = fmincon(@(X)resid_pure_Fx_varCamber(X,FXM3,SLFM3,IAM3, FzTF, Xop1, Xop2),X3F, [], [], [], [], [], []);



Xopt = [Xop1 Xop2 Xop3]; 
%   2. Fit the data using the previus parameters
%   4. Get the last parameters