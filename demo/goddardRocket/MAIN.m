% MAIN.m -- Goddard Rocket
%
% This script runs a trajectory optimization to find the optimal thrust
% trajectory for the rocket to reach the maximum altitude. Physical
% parameters are roughly based on the SpaceX Falcon 9 rocket.
%
% Dynamics include variable mass, inverse-square gravity, speed-dependent
% drag coefficient, height dependent air density.
%
% NOTES:
%   This problem sort of converges, but not very well. I think that there
%   is a singular arc in it that is not being handled correctly. It is
%   still interesting to see as an example of ways in which problems might
%   misbehave.
%

%clc; clear;
addpath ../../

%%%% Assumptions:
% SpaceX Falcon 9 rocket:
% http://www.spacex.com/falcon9
%
mTotal = 505846;   %(kg)  %Total lift-off mass
mFuel = 0.8*mTotal;  %(kg)  %mass of the fuel
mEmpty = mTotal-mFuel;  %(kg)  %mass of the rocket (without fuel)
Tmax = 5885000;    %(N)   %Maximum thrust

[t,x,u] = rocket_codegen(mTotal,mEmpty,Tmax);

figure(120);
subplot(2,2,1);
plot(t,x(1,:)/1000)
xlabel('time (s)')
ylabel('height (km)')
title('Maximal Height Trajectory')
subplot(2,2,2);
plot(t,x(3,:))
xlabel('time (s)')
ylabel('mass (kg)')
title('Goddard Rocket')
subplot(2,2,3);
plot(t,x(2,:))
xlabel('time (s)')
ylabel('velocity (m/s)')
subplot(2,2,4);
plot(t,u/1000)
xlabel('time (s)')
ylabel('thrust (kN)')
