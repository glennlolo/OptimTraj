function [t,x,u] = diffDriveOptim(robot,xMax,x0,theta0,wp)
%DIFFDRIVEOPTIM Function defining the differential drive robot problem

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Problem Bounds                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
phiDot0 = 0; %Initial Wheel speed (rad/s)
phiDotLow = 0; %Minimal Wheel Speed
phiDotUpp = Inf; %Infinite Wheel Speed
phiDotF = 0; %Final Wheel speed is supposed at 0 for static position.

thetaLow = -pi; %Heading of the robot is between -pi/pi
thetaUpp = pi;
thetaF = wp(end,3); %We suppose the final waypoint angle as requested

xLow = [0+robot.L 0+robot.L]; %The robot minimum position (adding the robot semi length)
xF = wp(end,1:2); %Taking the final waypoints as the final position

i0 = 0; %Initial motor current (A)
iLow = -5; %Minimal motor current (A)
iUpp = 5; %Maximal motor current (A)
iF = 0; %Final motor current is supposed to be 0 for static position.

VLow = -24; %Minimal Tension in the motors (V)
VUpp = 24; %Maximal Tension in the motors (V)

P.bounds.initialTime.low = 0;
P.bounds.initialTime.upp = 0;

P.bounds.finalTime.low = 0;
P.bounds.finalTime.upp = 20;

P.bounds.state.low = [phiDotLow;phiDotLow;thetaLow;xLow';iLow;iLow];
P.bounds.state.upp = [phiDotUpp;phiDotUpp;thetaUpp;xMax'-robot.L;iUpp;iUpp];

P.bounds.initialState.low = [phiDot0;phiDot0;theta0;x0';i0;i0];
P.bounds.initialState.upp = [phiDot0;phiDot0;theta0;x0';i0;i0];

P.bounds.finalState.low = [phiDotF;phiDotF;thetaF-deg2rad(5);xF'-1e-2;iF;iF];
P.bounds.finalState.upp = [phiDotF;phiDotF;thetaF+deg2rad(5);xF'+1e-2;iF;iF];

P.bounds.control.low = [VLow;VLow];
P.bounds.control.upp = [VUpp;VUpp];
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                           Initial Guess                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%Use a simple movement Bang/Bang Law for Guess
a0 = 1; %Maximum Acceleration (m/s^2)
wpFull = [x0;wp(:,1:2)];
xSection = sqrt(dot(diff(wpFull),diff(wpFull),2))'; %Waypoint distances
tGuess = sqrt(9/2*xSection/a0); %Time guess for each waypoint
vGuess = 3/2*xSection./tGuess; %Linear Velocity Guess for each waypoint

%Construct the state guess
omegaGuess = pi/4; %Guess rotation;
phiDotGuess = [(vGuess+omegaGuess)/robot.R;(vGuess-omegaGuess)/robot.R]; %Guess wheel speeds (Right, Left)
xGuess = zeros(7,size(wp,1)-1); %Search the guess just for intermediary waypoints
for i = 1:size(xGuess,2)
    xGuess(:,i) = [phiDotGuess(i,:)';wp(i,3);wp(i,1:2)';iUpp/2;iUpp/2];
end

P.guess.time = [0,cumsum(tGuess)];  %(s)
P.guess.state = [ [phiDot0;phiDot0;theta0;x0';i0;i0], xGuess,[phiDotF;phiDotF;thetaF;xF';iF;iF] ];
P.guess.control = [uUpp, uLow];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Objective and Dynamic functions                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Dynamics function:
P.func.dynamics = @(t,x,u)( diffDriveDynamics(x,u) );

% Objective function:
P.func.bndObj = @(t0,x0,tF,xF)( tF );  %Minimize final time

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Options and Method selection                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%



end