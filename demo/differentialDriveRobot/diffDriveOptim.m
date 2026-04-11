function soln = diffDriveOptim(robot,xMax,x0,theta0,wp)
%DIFFDRIVEOPTIM Function defining the differential drive robot problem

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Problem Bounds                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
s0 = 0; %Initial robot speed (m/s)
sLow = -5; %Minimal robot Speed
sUpp = 5; %Infinite robot Speed
sF = 0; %Final robot speed is supposed at 0 for static position.

w0 = 0; %Initial robot angular speed (rad/s)
wLow = -2*pi; %Minimal robot angular Speed
wUpp = 2*pi; %Infinite robot angular Speed
wF = 0; %Final robot angular speed is supposed at 0 for static position.

thetaLow = -pi; %Heading of the robot is between -pi/pi
thetaUpp = pi;
thetaF = wp(end,3); %We suppose the final waypoint angle as requested

xLow = [0+robot.L 0+robot.L]; %The robot minimum position (adding the robot semi length)
xF = wp(end,1:2); %Taking the final waypoints as the final position

i0 = 0; %Initial motor current (A)
iLow = -2; %Minimal motor current (A)
iUpp = 2; %Maximal motor current (A)
iF = 0; %Final motor current is supposed to be 0 for static position.

vLow = -12; %Minimal Tension in the motors (V)
vUpp = 12; %Maximal Tension in the motors (V)
v0 = 0; %Initial Tension Guess
vF = 0; %Final Tension Guess

P.bounds.initialTime.low = 0;
P.bounds.initialTime.upp = 0;

P.bounds.finalTime.low = 0;
P.bounds.finalTime.upp = 20;

P.bounds.state.low = [sLow;wLow;thetaLow;xLow';iLow;iLow];
P.bounds.state.upp = [sUpp;wUpp;thetaUpp;xMax'-robot.L;iUpp;iUpp];

P.bounds.initialState.low = [s0;w0;theta0;x0';i0;i0];
P.bounds.initialState.upp = [s0;w0;theta0;x0';i0;i0];

P.bounds.finalState.low = [sF;wF;thetaF;xF'-1e-2;iF;iF];
P.bounds.finalState.upp = [sF;wF;thetaF;xF'+1e-2;iF;iF];

P.bounds.control.low = [vLow;vLow];
P.bounds.control.upp = [vUpp;vUpp];
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Initial Guess                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%Use a simple movement Bang/Bang Law for Guess
a0 = 1; %Maximum Acceleration (m/s^2)
wpFull = [x0;wp(:,1:2)];
xSection = diff(wpFull); %Waypoints Section vectors
xDistances = sqrt(dot(xSection,xSection,2))'; %Waypoint distances
tGuess = sqrt(9/2*xDistances/a0); %Time guess for each waypoint
thetaGuess = atan2(xSection(:,2),xSection(:,1));
wpTime = [cumsum(tGuess(1:end-1)'),wp(1:end-1,:)];

%Construct the state guess
wGuess = diff(thetaGuess)/2; %Guess rotation;
vGuess = 3*xDistances./(2*tGuess); %Guess speed
xGuess = zeros(7,size(wp,1)-1); %Search the guess just for intermediary waypoints
for i = 1:size(xGuess,2)
    xGuess(:,i) = [vGuess(i);wGuess(i);wp(i,3);wp(i,1:2)';0;0];
end
vGuess = robot.Kb*robot.N*xGuess(1:2,:);

P.guess.time = [0,cumsum(tGuess)];  %(s)
P.guess.state = [ [s0;w0;theta0;x0';i0;i0], xGuess,[sF;wF;thetaF;xF';iF;iF] ];
P.guess.control = [[v0;v0], vGuess,[vF;vF]];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Objective and Dynamic functions                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Dynamics function:
[A,B] = dynamicConstantsComputing(robot);
P.func.dynamics = @(t,x,u)( diffDriveDynamics(x,u,robot,A,B) );

% Objective function:
P.func.bndObj = @(t0,x0,tF,xF)( tF-t0+wpTime(1) );  %Minimize final time

% Path Constraint Function
P.func.pathCst = @(t,x,u)( pathConst(t,x,wpTime) );

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Options and Method selection                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
P.options.method = 'trapezoid';
P.options.defaultAccuracy = 'low';

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                              Solve!                                     %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
soln = optimTraj(P);

end

function [A,B] = dynamicConstantsComputing(robot)
%% Creating linear State Matrices (Constants)
a = robot.Kt*robot.N/(robot.R*robot.M);
b = robot.Kt*robot.N/(robot.R*robot.L)/(robot.M*robot.d^2+robot.J);
c = robot.Kb*robot.N/(robot.R*robot.La);
d = robot.Ra/robot.La;
A = [0 0 0 0 0 a a;
     0 0 0 0 0 b -b;
     0 1 0 0 0 0 0;
     0 0 0 0 0 0 0;
     0 0 0 0 0 0 0;
     -c -c*robot.L 0 0 0 -d 0;
     -c +c*robot.L 0 0 0 0 -d];
B = [0 0;
     0 0;
     0 0;
     0 0;
     0 0;
     1/robot.La 0;
     0 1/robot.La];
end