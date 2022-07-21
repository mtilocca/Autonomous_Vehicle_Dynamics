% data load
clc
clear
load('20130222_01_02_03_grandsport.mat')
%%
%%Plot lateral and longitudinal velocity
longvel=insData.vxCG.value;
latevel=insData.vyCG.value;
figure(1)
subplot(2,1,1) 
plot(longvel,'-')
grid on 
title('Subplot 1: Longitudinal velocity m/s')
subplot(2,1,2)
plot(latevel,'-')
grid on
title('Subplot 2: Lateral velocity m/s')
%%
% section 2
%Evaluate the longitudinal speed using the Hall-effect wheel speed
%sensors and compare the data with the INS data

figure()
plot(tireData.wheelTicksRL.time, tireData.wheelTicksRL.value, 'r')
[value,loc]=findpeaks(-tireData.wheelTicksRL.value);
i=length(loc);
j=1;
T=zeros(i,1);
v=zeros(i,1);


time_peaks = tireData.wheelTicksRL.time(loc);
delta = diff(time_peaks);
v=(params.rollingCircumferenceRL.value)./delta;
           

figure(2)
plot(insData.vxCG.time,longvel,'m') 
hold on
plot(time_peaks(1:end-1),v,'r')
hold on
grid on
ylabel('velocity  [m/s]')
xlabel('time ')
legend('Measured','Hall effect sensor calculated')
%%
%Evaluate the lateral acceleration using the relation with the yaw-rate
%and the longitudinal speed.

yr=insData.yawRate.value; %yr=yaw rate
for i=1:7396
    ay(i)=yr(i)*longvel(i);
    %as ay=omega*u is a good aproximation neglecting road banking
end

figure(3)
plot(ay) 
grid on
hold on
ylabel('lateral acceleration [m/s^2]')
xlabel('time  [s]')
legend('a_y lat acceleration' )

%%
%Comparing the longitudinal acceleration measured by INS with the one
%obtained by derivation of the longitudinal speed measured from the Hall
%sensors. 
figure(4)
plot(insData.vxCG.time,longvel,'k') 
hold on
plot(time_peaks(1:end-1),v,'m')
grid on
ylabel('longitudinal acceleration [m/s^2]')
xlabel('time  [s]')
legend('ax',' ax derived')

%%
%Evaluate the side slip angle.
figure(5)
plot(insData.sideSlip.time,insData.sideSlip.value,"b")
grid on
ylabel('Side Slip angle [º]')
xlabel('time  [s]')
legend('alfa')

