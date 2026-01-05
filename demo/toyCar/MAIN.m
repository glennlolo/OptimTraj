% MAIN.m  --  Toy Car
%
% Dynamics:
%   A simple model of a car, where the state is its position and
%   orientation, and the control is the rate of change in steering.
%
% Objective:
%   Find the best path between two points that avoids driving on steep
%   slopes.
%

clc; clear;

xBnd = [1,5];
yBnd = [1,5];

startPoint = [2.5;1.5];   %Start here
finishPoint = [4.0;4.5];   %Finish here

uMax = 100.0;  %Max steering rate

soln = toyCar_codegen(xBnd,yBnd,startPoint,finishPoint,uMax);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Display the solution                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

figure(1); clf; hold on;

drawHills(xBnd,yBnd);

t = linspace(soln.grid.time(1), soln.grid.time(end), 150);
z = soln.interp.state(t);
x = z(1,:);
y = z(2,:);
th = z(3,:);
u = soln.interp.control(t);

tGrid = soln.grid.time;
xGrid = soln.grid.state(1,:);
yGrid = soln.grid.state(2,:);
thGrid = soln.grid.state(3,:);
uGrid = soln.grid.control;

% Plot the entire trajectory
plot(x,y,'r-','LineWidth',3);

% Plot the grid points:
plot(xGrid, yGrid, 'ko','MarkerSize',5,'LineWidth',3);

% Plot the start and end points:
plot(x([1,end]), y([1,end]),'ks','MarkerSize',12,'LineWidth',3);

% Plot the state and control:
figure(2); clf; 

subplot(2,2,1); hold on;
plot(t,x);
plot(tGrid,xGrid,'ko','MarkerSize',5,'LineWidth',3);
ylabel('x');

subplot(2,2,3); hold on;
plot(t,y);
plot(tGrid,yGrid,'ko','MarkerSize',5,'LineWidth',3);
ylabel('y');

subplot(2,2,2); hold on;
plot(t,th);
plot(tGrid,thGrid,'ko','MarkerSize',5,'LineWidth',3);
ylabel('θ');

subplot(2,2,4); hold on;
plot(tGrid,uGrid,'ko','MarkerSize',5,'LineWidth',3);
plot(t,u);
ylabel('u');
