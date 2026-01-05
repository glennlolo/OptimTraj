function [Cpa,Cpb] = pathConst(t,z)
%PATHCONST Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    t
    z
end

arguments (Output)
    Cpa
    Cpb
end

x = z(1,:);  
y = z(2,:);

Cpa = [];

[~,idx]=min(abs(t-2));
Cpb = zeros([2 length(t)]);
if ~isempty(idx)
    Cpb(1,idx) = x(idx)-4;
    Cpb(2,idx) = y(idx)-1;
end

end