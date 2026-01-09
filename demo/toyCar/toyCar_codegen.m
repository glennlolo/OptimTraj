function soln = toyCar_codegen(xBnd,yBnd,startPoint,finishPoint,uMax)

waypoint = [3;3.5];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Set up function handles                            %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.func.dynamics = @(t,x,u)( dynamics(x,u) );
problem.func.pathObj = @(t,x,u)( objective(x,u) );
problem.func.pathCst = @(t,x,u)( pathConst(t,x,waypoint) );

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Set up bounds on state and control                      %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = 0.1;
problem.bounds.finalTime.upp = 100;

problem.bounds.state.low = [xBnd(1); yBnd(1); -2*pi];
problem.bounds.state.upp = [xBnd(2); yBnd(2);  2*pi];

problem.bounds.initialState.low = [startPoint; -2*pi];
problem.bounds.initialState.upp = [startPoint; 2*pi];

problem.bounds.finalState.low = [finishPoint; -2*pi];
problem.bounds.finalState.upp = [finishPoint; 2*pi];

problem.bounds.control.low = -uMax;
problem.bounds.control.upp = uMax;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                 Initialize trajectory with guess                        %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% Car travels at a speed of one, and drives in a straight line from start
% to finish point.

del = finishPoint - waypoint;  % vector from start to finish
mid = waypoint - startPoint;
angle = atan2(del(1),del(2));
angleMid = atan2(mid(1),mid(2));

problem.guess.time = [0, norm(del)+norm(mid)];
problem.guess.state = [[startPoint; angleMid],[finishPoint; angle]];
problem.guess.control = [0,0];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                      Options for Transcription                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.options.method = 'trapezoid';
problem.options.trapezoid.nGrid = 15;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve!                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

soln = optimTraj(problem);
end