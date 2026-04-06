function soln = diffDriveOptim(robot,xMax,x0,theta0,wp)
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

vLow = -24; %Minimal Tension in the motors (V)
vUpp = 24; %Maximal Tension in the motors (V)
v0 = 0; %Initial Tension Guess
vF = 0; %Final Tension Guess

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
vLGuess = 3/2*xDistances./tGuess; %Linear Velocity Guess for each waypoint
thetaGuess = atan2(xSection(:,2),xSection(:,1));
wpTime = [tGuess(1:end-1),wp(1:end-1,1:2)];

%Construct the state guess
omegaGuess = diff(thetaGuess)/2; %Guess rotation;
phiDotGuess = [(vLGuess+omegaGuess)/robot.R;(vLGuess-omegaGuess)/robot.R]; %Guess wheel speeds (Right, Left)
xGuess = zeros(7,size(wp,1)-1); %Search the guess just for intermediary waypoints
for i = 1:size(xGuess,2)
    xGuess(:,i) = [phiDotGuess(:,i);wp(i,3);wp(i,1:2)';iUpp/2;iUpp/2];
end
vGuess = robot.Kb*robot.N*xGuess(1:2,:);

P.guess.time = [0,cumsum(tGuess)];  %(s)
P.guess.state = [ [phiDot0;phiDot0;theta0;x0';i0;i0], xGuess,[phiDotF;phiDotF;thetaF;xF';iF;iF] ];
P.guess.control = [[v0;v0], vGuess,[vF;vF]];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Objective and Dynamic functions                         %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% Dynamics function:
P.func.dynamics = @(t,x,u)( diffDriveDynamics(x,u,robot) );

% Objective function:
P.func.bndObj = @(t0,x0,tF,xF)( tF-t0 );  %Minimize final time

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