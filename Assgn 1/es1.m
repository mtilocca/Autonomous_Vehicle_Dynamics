clear all;
close all;
clc
%% x axis Force

D = 2100;   B = 12.2;
C = 1.87;   E = 0.13;
Sh= 0.000851;   Sv= -77;

k_v = -1:0.001:1;
F_x = PacejkaMF(k_v, B, C, D, E, Sh, Sv);
figure(1);
plot(k_v, F_x);
grid on;
xlabel('\kappa');
ylabel('F_x0');
legend('F_x_0', 'Location','southeast'); 
title('longitudinal tire force as function of longitudinal slip \kappa');

% second part

om = 70; R = 0.2; v =13;

k2 = (om*R - v)/v

F_x0_k2 = PacejkaMF(k2, B, C, D, E, Sh, Sv)

% third part Cornering stiffness 


syms x
%f = sin(5*x);
f = D*sin(C*atan(B*x - E* (B*x - atan(B*x))));
deriv = diff(f);

Cfk = (3927*cos((187*atan((5307*Sh)/500 + (13*atan((61*Sh)/5))/100))/100)*(793/(500*((3721*Sh^2)/25 + 1)) + 5307/500))/(((5307*Sh)/500 + (13*atan((61*Sh)/5))/100)^2 + 1)
 

%% ex 2 
k = 0.08; vcx = 15; vcy = -1.3; 
Bxa = 23.308/(sqrt((19.414*k)^2+1));
Cxa = 0.926;
Shxa = -0.001257;
Dxa = 1 / (cos(Cxa*atan(Bxa*Shxa))); 

% find alpha and then Fx 

alpha = -atan(vcy/vcx)
Fx02 = PacejkaMF(k, B, C, D, E, Sh, Sv)
Y1 = PCJMF(Dxa, Cxa, Bxa, alpha, Shxa, Fx02) 

%% alpha range 
alpha_c = [0 2 4 6 8];
alpha_cr = deg2rad(alpha_c);

Fxa0 = PacejkaMF(k_v, B, C, D, E, Sh, Sv);
Ya0 = PCJMF(Dxa, Cxa, k_v, alpha_cr(1), Shxa, Fxa0); 
Ga0 = Ga(Dxa, Cxa, k_v, alpha_cr(1), Shxa); 

Fxa2 = PacejkaMF(k_v, B, C, D, E, Sh, Sv);
Ya2 = PCJMF(Dxa, Cxa, k_v, alpha_cr(2), Shxa, Fxa2); 
Ga2 = Ga(Dxa, Cxa, k_v, alpha_cr(2), Shxa); 

Fxa4 = PacejkaMF(k_v, B, C, D, E, Sh, Sv);
Ya4 = PCJMF(Dxa, Cxa, k_v, alpha_cr(3), Shxa, Fxa4); 
Ga4 = Ga(Dxa, Cxa, k_v, alpha_cr(3), Shxa); 


Fxa6 = PacejkaMF(k_v, B, C, D, E, Sh, Sv);
Ya6 = PCJMF(Dxa, Cxa, k_v, alpha_cr(4), Shxa, Fxa6); 
Ga6 = Ga(Dxa, Cxa, k_v, alpha_cr(4), Shxa); 


Fxa8 = PacejkaMF(k_v, B, C, D, E, Sh, Sv);
Ya8 = PCJMF(Dxa, Cxa, k_v, alpha_cr(5), Shxa, Fxa8); 
Ga8 = Ga(Dxa, Cxa, k_v, alpha_cr(5), Shxa); 



figure(2)
plot(k_v, Ya0, 'r');
hold on ;
grid on; 
plot(k_v, Ya2, 'k');
plot(k_v, Ya4, 'm');
plot(k_v, Ya6, 'b');
plot(k_v, Ya8, 'g');
xlabel('\kappa');
ylabel('F_x');
legend('\alpha = 0 deg','\alpha = 2 deg', '\alpha = 4 deg', '\alpha = 6 deg', '\alpha = 8 deg', 'Location','southeast'); 
title('longitudinal tire force as function of \alpha');

figure(3)
plot(k_v, Ga0, 'r');
hold on ;
grid on; 
plot(k_v, Ga2, 'k');
plot(k_v, Ga4, 'm');
plot(k_v, Ga6, 'b');
plot(k_v, Ga8, 'g');
xlabel('\kappa');
ylabel('G_x_a');
legend('\alpha = 0 deg','\alpha = 2 deg', '\alpha = 4 deg', '\alpha = 6 deg', '\alpha = 8 deg', 'Location','southeast'); 
title('Weighting function G_xa as function of \alpha');










