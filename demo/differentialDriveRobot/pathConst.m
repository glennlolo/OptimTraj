function [Cpa,Cpb] = pathConst(t,z,waypoint)
%PATHCONST Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    t
    z
    waypoint
end

arguments (Output)
    Cpa
    Cpb
end

x = z(4,:);  
y = z(5,:);

Cpa = [];

[~,idx]=min(abs(t-waypoint(:,1)),[],2);
Cpb = zeros([2 length(t)]);
if ~isempty(idx)
    for i = 1:length(idx)
        Cpb(1,idx(i)) = x(idx(i))-waypoint(i,2);
        Cpb(2,idx(i)) = y(idx(i))-waypoint(i,3);
    end
end

end