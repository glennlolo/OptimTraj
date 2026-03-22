function defaultSoln = defaultSolution(problem,nState)
%DEFAULTSOLUTION This function creates a default solution structure
%according to the method and the problem definition
arguments (Input)
    problem struct
    nState (1,1) double
end
coder.varsize("defaultName",[1 10],[0 1])
coder.varsize("defaultState",[nState Inf], [0 1])
coder.varsize("defaultTime",[1 Inf],[0 1])

defaultName = '';
defaultState = zeros(3,1);
defaultTime = 0;
defaultInfo = struct('iterations',0,'funcCount',0,'algorithm',defaultName,...
    'constrviolation',0,'stepsize',0,'lssteplength',0,'firstorderopt',0,...
    'nlpTime',0,'exitFlag',0,'objVal',0,'error',defaultState,'maxError',0);
defaultGrid = struct('time',defaultTime,'state',defaultState,'control',defaultTime);
%defaultInterp = struct('control',@() 0,'state',@() 0,'collCst',@() 0);
defaultInterp = [];
problemComp = problem;
problemComp.func.weights = defaultTime';
problemComp.func.defectCst = @() 0;

defaultSoln = struct('grid',defaultGrid,'info',defaultInfo,'problem',problemComp,'interp',defaultInterp);
end