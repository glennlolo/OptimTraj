function dz = diffDriveDynamics(z,u,robot)
%DIFFDRIVEDYNAMICS Function defining the differrential drive robot
%dynamics. The state of the robot is its two wheels speed, its orientation,
%its position along the plane, its current on both motors.
%
%INPUTS:
%   z = [7,n] = [phiDotR,phiDotL,theta,x,y,iR,iL] = state
%   u = [2,n] = [vR,vL] = motors tension
%   robot = [1,1] = struct = robot parameters definition
%
%OUTPUTS:
%   dz = dz/dt

%% Computing Dynamics constants
%intermediary constants
ab = robot.R*(robot.M*robot.d^2+robot.J)/(4*robot.L^2);
a = ab+robot.M*robot.R/4;
b = -ab+robot.M*robot.R/4;
cd = robot.M*robot.d*robot.R^2/(4*robot.L^2);
c = cd;
d = cd;
%computing constants
e = 1/(a-b^2/a);
f = -c*b/a;
g = -d*(1-b/a);
h = -b*robot.Kt/(robot.R*a);

%% Creating linear State Matrices (Constants)
A = zeros(7);
B = zeros(7,2);

A(1:2,6:7) = [e*robot.Kt/robot.R e*h; %phiDot to phiDotDot components
              e*h e*robot.Kt/robot.R];
A(3,1:2) = robot.R/2*[1 -1]; %phiDot to omega components
A(6,[1 5]) = [-robot.Kb*robot.N/robot.La -robot.Ra/robot.La]; %phiDot and current to current derivates components
A(7,[2 6]) = [-robot.Kb*robot.N/robot.La -robot.Ra/robot.La]; %phiDot and current to current derivates components

B(6,1) = 1/robot.La;
B(7,2) = B(6,2);

%% Creating nonLinear State Matrices (Variable)
Bv = zeros(7,1);

Bv(1:2) = e*[f*z(1)^2+c*z(2)^2+g*z(1)*z(2);
             f*z(2)^2+c*z(1)^2+g*z(1)*z(2)];
Bv(4:5) = [robot.R*cos(z(3))/2*z(1) + robot.R*cos(z(3))/2*z(2);
            robot.R*sin(z(3))/2*z(1) + robot.R*sin(z(3))/2*z(2)];

%% Calculating Derivates
dz = A*z+B*u+Bv;

end