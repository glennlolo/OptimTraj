function problemOut = getDefaultOptions(problemIn)
% problemOut = getDefaultOptions(problemIn)
%
% This function fills in any blank entries in the problem.options struct.
% It is designed to be called from inside of optimTraj.m, and not by the
% user.
%

problemOut.func = problemIn.func;
problemOut.bounds = problemIn.bounds;
problemOut.guess = problemIn.guess;

% Figure out basic problem size:
nState = size(problemIn.guess.state,1);
nControl = size(problemIn.guess.control,1);

%%%% Top-level default options:
OPT.method = 'trapezoid';
OPT.verbose = 2;
OPT.defaultAccuracy = 'medium';
OPT.nlpOpt = optimset(...
                'Display','iter',...
                'TolFun',1e-6,...
                'MaxIter',400,...
                'MaxFunEvals',5e4*(nState+nControl));
OPT.trapezoid = defaults_trapezoid('medium');
OPT.hermiteSimpson = defaults_hermiteSimpson('medium');
OPT.chebyshev = defaults_chebyshev('medium');
OPT.multiCheb = defaults_multiCheb('medium');
OPT.rungeKutta = defaults_rungeKutta('medium');
OPT.gpops = defaults_gpops('medium');


%%%% Basic setup

% ensure that options is not empty
if ~isfield(problemIn,'options')
    problemOut.options.method = OPT.method;
end
problemOut.options = repmat(OPT,1,length(problemIn.options));

% Loop over opt and fill in nlpOpt struct:
for i=1:length(problemIn.options)
    if ~isfield(problemIn.options(i),'verbose') || isempty(problemIn.options(i).verbose)
        verbose = OPT.verbose;
    else
        verbose = problemIn.options(i).verbose;
    end
    problemOut.options(i).verbose = verbose;
    if ~isfield(problemIn.options(i),'defaultAccuracy') || isempty(problemIn.options(i).defaultAccuracy)
        defaultAccuracy = OPT.defaultAccuracy;
    else
        defaultAccuracy = problemIn.options(i).defaultAccuracy;
    end
    problemOut.options(i).defaultAccuracy = defaultAccuracy;
    switch verbose
        case 0
            NLP_display = 'notify';
        case 1
            NLP_display = 'final-detailed';
        case 2
            NLP_display = 'iter';
        case 3
            NLP_display = 'iter-detailed';
        otherwise
            error('Invalid value for options.verbose');
    end
    switch defaultAccuracy
        case 'low'
            problemOut.options(i).nlpOpt = optimset(...
                'Display',NLP_display,...
                'TolFun',1e-4,...
                'MaxIter',200,...
                'MaxFunEvals',1e4*(nState+nControl));
        case 'medium'
            problemOut.options(i).nlpOpt = optimset(...
                'Display',NLP_display,...
                'TolFun',1e-6,...
                'MaxIter',400,...
                'MaxFunEvals',5e4*(nState+nControl));
        case 'high'
            problemOut.options(i).nlpOpt = optimset(...
                'Display',NLP_display,...
                'TolFun',1e-8,...
                'MaxIter',800,...
                'MaxFunEvals',1e5*(nState+nControl));
        otherwise
            error('Invalid value for options.defaultAccuracy')
    end
    if isfield(problemIn.options(i),'nlpOpt')
        if isstruct(problemIn.options(i).nlpOpt) && ~isempty(problemIn.options(i).nlpOpt)
            names = fieldnames(problemIn.options(i).nlpOpt);
            for j=1:length(names)
                if ~isfield(OPT.nlpOpt,names{j})
                    disp(['WARNING: options.nlpOpt.' names{j} ' is not a valid option']);
                else
                    problemOut.options(i).nlpOpt.(names{j}) = problemIn.options(i).nlpOpt.(names{j});
                end
            end
        end
    end
end

% Check ChebFun dependency:
missingChebFun = false;
for i=1:length(problemIn.options(i))
    if strcmp(problemIn.options(i).method,'chebyshev')
        if ~isfile("chebpts.m")
            missingChebFun = true;
            problemOut.options(i).method = 'trapezoid';  %Force default method
        end
    end
end
if missingChebFun
   warning('''chebyshev'' method requires the Chebfun toolbox');
   disp('   --> Install Chebfun toolbox:  (http://www.chebfun.org/)');
   disp('   --> Running with default method instead (''trapezoid'')');
end

% Fill in method-specific paramters:
for i=1:length(problemIn.options(i))
    if ~isfield(problemIn.options(i),'method') || isempty(problemIn.options(i).method)
        method = OPT.method;
    else
        method = problemIn.options(i).method;
    end
    if ~isfield(problemIn.options(i),'defaultAccuracy') || isempty(problemIn.options(i).defaultAccuracy)
        defaultAccuracy = OPT.defaultAccuracy;
    else
        defaultAccuracy = problemIn.options(i).defaultAccuracy;
    end
    problemOut.options(i).method = method;
    switch method
        case 'trapezoid'
            problemOut.options(i).trapezoid = defaults_trapezoid(defaultAccuracy);
        case 'hermiteSimpson'
            problemOut.options(i).hermiteSimpson = defaults_hermiteSimpson(defaultAccuracy);
        case 'chebyshev'
            problemOut.options(i).chebyshev = defaults_chebyshev(defaultAccuracy);
        case 'multiCheb'
            problemOut.options(i).multiCheb = defaults_multiCheb(defaultAccuracy);
        case 'rungeKutta'
            problemOut.options(i).rungeKutta = defaults_rungeKutta(defaultAccuracy);
        case 'gpops'
            problemOut.options(i).gpops = defaults_gpops(defaultAccuracy);
        otherwise
            error('Invalid value for options.method');
    end
    if isfield(problemIn.options(i),method)
        fields = fieldnames(problemIn.options(i));
        for j=1:length(fields)
            if isstruct(problemIn.options(i).(fields{j})) && ~isempty(problemIn.options(i).(fields{j}))
                names = fieldnames(problemIn.options(i).(fields{j}));
                for k=1:length(names)
                    if ~isfield(problemIn.options(i).(fields{j}),names{k})
                        disp(['WARNING: options.' fields{j} '.' names{k} ' is not a valid option']);
                    else
                        problemOut.options(i).(fields{j}).(names{k}) = problemIn.options(i).(fields{j}).(names{k});
                    end
                end
            elseif ~isempty(problemIn.options(i).(fields{j}))
                problemOut.options(i).(fields{j}) = problemIn.options(i).(fields{j});
            end
        end
    end
end
end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Method-specific parameters                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%



function OPT_trapezoid = defaults_trapezoid(accuracy)

switch accuracy
    case 'low'
        OPT_trapezoid.nGrid = 12;
    case 'medium'
        OPT_trapezoid.nGrid = 30;
    case 'high'
        OPT_trapezoid.nGrid = 60;
    otherwise
        error('Invalid value for options.defaultAccuracy')
end

OPT_trapezoid.adaptiveDerivativeCheck = 'off';

end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%



function OPT_hermiteSimpson = defaults_hermiteSimpson(accuracy)

switch accuracy
    case 'low'
        OPT_hermiteSimpson.nSegment = 10;
    case 'medium'
        OPT_hermiteSimpson.nSegment = 20;
    case 'high'
        OPT_hermiteSimpson.nSegment = 40;
    otherwise
        error('Invalid value for options.defaultAccuracy')
end

OPT_hermiteSimpson.adaptiveDerivativeCheck = 'off';

end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%



function OPT_chebyshev = defaults_chebyshev(accuracy)

switch accuracy
    case 'low'
        OPT_chebyshev.nColPts = 9;
    case 'medium'
        OPT_chebyshev.nColPts = 13;
    case 'high'
        OPT_chebyshev.nColPts = 23;
    otherwise
        error('Invalid value for options.defaultAccuracy')
end

end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


function OPT_multiCheb = defaults_multiCheb(accuracy)

switch accuracy
    case 'low'
        OPT_multiCheb.nColPts = 6;
        OPT_multiCheb.nSegment = 3;
    case 'medium'
        OPT_multiCheb.nColPts = 8;
        OPT_multiCheb.nSegment = 6;
    case 'high'
        OPT_multiCheb.nColPts = 8;
        OPT_multiCheb.nSegment = 12;
    otherwise
        error('Invalid value for options.defaultAccuracy')
end

end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%



function OPT_rungeKutta = defaults_rungeKutta(accuracy)

switch accuracy
    case 'low'
        OPT_rungeKutta.nSegment = 10;
        OPT_rungeKutta.nSubStep = 2;
    case 'medium'
        OPT_rungeKutta.nSegment = 20;
        OPT_rungeKutta.nSubStep = 2;
    case 'high'
        OPT_rungeKutta.nSegment = 20;
        OPT_rungeKutta.nSubStep = 4;
    otherwise
        error('Invalid value for options.defaultAccuracy')
end

OPT_rungeKutta.adaptiveDerivativeCheck = 'off';

end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%


function OPT_gpops = defaults_gpops(accuracy)

OPT_gpops.bounds.phase.integral.lower = -inf;
OPT_gpops.bounds.phase.integral.upper = inf;
OPT_gpops.guess.phase.integral = 0;

OPT_gpops.name = 'OptimTraj_GPOPS';
OPT_gpops.auxdata = [];
OPT_gpops.nlp.solver = 'ipopt'; % {'ipopt','snopt'}
OPT_gpops.derivatives.dependencies = 'full';  %�full�, �sparse� or �sparseNaN�
OPT_gpops.derivatives.supplier = 'sparseCD'; %'sparseCD';  %'adigator'
OPT_gpops.derivatives.derivativelevel = 'first'; %'second';
OPT_gpops.mesh.method = 'hp-PattersonRao';
OPT_gpops.method = 'RPM-Integration';
OPT_gpops.mesh.phase.colpoints = 10*ones(1,10);
OPT_gpops.mesh.phase.fraction = ones(1,10)/10;
OPT_gpops.scales.method = 'none'; % { 'none' , automatic-hybridUpdate' , 'automatic-bounds';

switch accuracy
    case 'low'
        OPT_gpops.mesh.tolerance = 1e-2;
        OPT_gpops.mesh.maxiterations = 0;
    case 'medium'
        OPT_gpops.mesh.tolerance = 1e-3;
        OPT_gpops.mesh.maxiterations = 1;        
    case 'high'
        OPT_gpops.mesh.tolerance = 1e-4;
        OPT_gpops.mesh.maxiterations = 3;
    otherwise
        error('Invalid value for options.defaultAccuracy')
end

end
