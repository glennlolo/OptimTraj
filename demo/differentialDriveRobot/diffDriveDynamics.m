function dz = diffDriveDynamics(z,u,robot,A,B)
%DIFFDRIVEDYNAMICS Function defining the differrential drive robot
%dynamics. The state of the robot is its two wheels speed, its orientation,
%its position along the plane, its current on both motors.
%
%INPUTS:
%   z = [7,n] = [V,w,theta,x,y,iR,iL] = state
%   u = [2,n] = [vR,vL] = motors tension
%   robot = [1,1] = struct = robot parameters definition
%
%OUTPUTS:
%   dz = dz/dt

%% Creating nonLinear State Matrices (Variable)
Bv = zeros(7,size(z,2));

Bv(1,:) = robot.d*z(2,:);
Bv(2,:) = -robot.M*robot.d/(robot.M*robot.d^2+robot.J)*z(1,:).*z(2,:);
Bv(4,:) = cos(z(3,:)).*z(1,:);
Bv(5,:) = sin(z(3,:)).*z(1,:);

%% Calculating Derivates
dz = A*z+B*u+Bv;

end