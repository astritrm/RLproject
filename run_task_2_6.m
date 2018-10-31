
clear all
clc
close all

%%

rad2deg = 180/pi;
deg2rad = pi/180;

tstart=0;           % Sim start time
tstop=4500;        % Sim stop time
tsamp=10;           % Sampling time for how often states are stored. (NOT ODE solver time step)
                
p0=[1500 500];      % Initial position (NED)
v0=[6.63 0]';       % Initial velocity (body)
u_r_0 = v0(1);
psi0=50*deg2rad;             % Inital yaw angle
r0=0;               % Inital yaw rate
c=1;                % Current on (1)/off (0)

U_d = v0(1); %desired speed

init_controllers

%% task 2.1


%Loading waypoints
load('WP.mat')

wpt_posx = WP(1,:);
wpt_posy = WP(2,:);
wpt_time = [0 20 40 60 80 100]; 
t = 0:1:max(wpt_time);

% method 1 - cubic Hermite interpolation
x_p = pchip(wpt_time,wpt_posx,t);
y_p = pchip(wpt_time,wpt_posy,t);

% method 2 - spline interpolation
x_s = spline(wpt_time,wpt_posx,t); 
y_s = spline(wpt_time,wpt_posy,t);

n_of_waypoints = length(WP(1, :));
R_bar = zeros(1, n_of_waypoints-2);


%% SIMULATION
sim MSFartoystyring26

%% Path Plot
close all;

dec = 20;
object_tracking = 0;
pathplotter(p(:,1), p(:,2), psi, tsamp, dec, tstart, tstop, object_tracking, WP);

%%Use the manual switch in simulink to compare between sideslip-correction and no
%%correction
chi = beta(1:length(psi))+psi;
betap = beta(1:length(psi));
figure()
plot(t,rad2deg*psi,t,rad2deg*chi,t, rad2deg*chi_d,'LineWidth', 2);
legend('\psi','\chi','\chi_d', 'Interpreter', 'latex');

figure()
plot(t, rad2deg*betap, 'LineWidth', 2);
legend('\beta');

figure()
plot(t, v(:,1), 'LineWidth', 2);
legend('u');



