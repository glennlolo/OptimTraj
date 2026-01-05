function problemOut = inputValidation(problemIn)
%
% This function runs through the problem struct and sets any missing fields
% to the default value. If a mandatory field is missing, then it throws an
% error.
%
% INPUTS:
%   problemIn = a partially completed problem struct
%
% OUTPUTS:
%   problemOut = a complete problem struct, with validated fields
%

problemOut = problemIn;

%%%% Check the function handles:

if ~isfield(problemIn,'func')
    error('Field ''func'' cannot be ommitted from ''problem''');
else
    if ~isfield(problemIn.func,'dynamics')
        error('Field ''dynamics'' cannot be ommitted from ''problem.func'''); end
    if ~isfield(problemIn.func,'pathObj'), problemOut.func.pathObj = []; end
    if ~isfield(problemIn.func,'bndObj'), problemOut.func.bndObj = []; end
    if ~isfield(problemIn.func,'pathCst'), problemOut.func.pathCst = []; end
    if ~isfield(problemIn.func,'bndCst'), problemOut.func.bndCst = []; end
end

%%%% Check the initial guess (also compute nState and nControl):
if ~isfield(problemIn, 'guess')
    error('Field ''guess'' cannot be ommitted from ''problem''');
else
    if ~isfield(problemIn.guess,'time')
        error('Field ''time'' cannot be ommitted from ''problem.guess'''); end
    if ~isfield(problemIn.guess, 'state')
        error('Field ''state'' cannot be ommitted from ''problem.guess'''); end
    if ~isfield(problemIn.guess, 'control')
        error('Field ''control'' cannot be ommitted from ''problem.guess'''); end
    
    % Compute the size of the time, state, and control based on guess
    [checkOne, nTime] = size(problemIn.guess.time);
    [nState, checkTimeState] = size(problemIn.guess.state);
    [nControl, checkTimeControl] = size(problemIn.guess.control);
    
    if nTime < 2 || checkOne ~= 1
        error('guess.time must have dimensions of [1, nTime], where nTime > 1');
    end
    
    if checkTimeState ~= nTime
        error('guess.state must have dimensions of [nState, nTime]');
    end
    if checkTimeControl ~= nTime
        error('guess.control must have dimensions of [nControl, nTime]');
    end
    
end

%%%% Check the problem bounds:
if ~isfield(problemIn,'bounds')
    problemOut.bounds.initialTime = [];
    problemOut.bounds.finalTime = [];
    problemOut.bounds.state = [];
    problemOut.bounds.initialState = [];
    problemOut.bounds.finalState = [];
    problemOut.bounds.control = [];
else
    
    if ~isfield(problemIn.bounds,'initialTime')
        problemOut.bounds.initialTime = []; end
    problemOut.bounds.initialTime = ...
        checkLowUpp(problemOut.bounds.initialTime,1,1,'initialTime');
    
    if ~isfield(problemIn.bounds,'finalTime')
        problemOut.bounds.finalTime = []; end
    problemOut.bounds.finalTime = ...
        checkLowUpp(problemOut.bounds.finalTime,1,1,'finalTime');
    
    if ~isfield(problemIn.bounds,'state')
        problemOut.bounds.state = []; end
    problemOut.bounds.state = ...
        checkLowUpp(problemOut.bounds.state,nState,1,'state');
    
    if ~isfield(problemIn.bounds,'initialState')
        problemOut.bounds.initialState = []; end
    problemOut.bounds.initialState = ...
        checkLowUpp(problemOut.bounds.initialState,nState,1,'initialState');
    
    if ~isfield(problemIn.bounds,'finalState')
        problemOut.bounds.finalState = []; end
    problemOut.bounds.finalState = ...
        checkLowUpp(problemOut.bounds.finalState,nState,1,'finalState');
    
    if ~isfield(problemIn.bounds,'control')
        problemOut.bounds.control = []; end
    problemOut.bounds.control = ...
        checkLowUpp(problemOut.bounds.control,nControl,1,'control');
    
end

end


function input = checkLowUpp(input,nRow,nCol,name)
%
% This function checks that input has the following is true:
%   size(input.low) == [nRow, nCol]
%   size(input.upp) == [nRow, nCol]

if ~isfield(input,'low')
    input.low = -inf(nRow,nCol);
end

if ~isfield(input,'upp')
    input.upp = inf(nRow,nCol);
end

[lowRow, lowCol] = size(input.low);
if lowRow ~= nRow || lowCol ~= nCol
    error(['problem.bounds.' name ...
        '.low must have size = [' num2str(nRow) ', ' num2str(nCol) ']']);
end

[uppRow, uppCol] = size(input.upp);
if uppRow ~= nRow || uppCol ~= nCol
    error(['problem.bounds.' name ...
        '.upp must have size = [' num2str(nRow) ', ' num2str(nCol) ']']);
end

if sum(sum(input.upp-input.low < 0))
    error(...
        ['problem.bounds.' name '.upp must be >= problem.bounds.' name '.low!']);
end

end