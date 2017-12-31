%% M10
% Input parameters:
% tx ... column vector with the duration of each segment
% ax ... column vector with the acceleration used for each segment
% t_ipo ... interpolation clock
% Output parameters:
% t ... vector containing time
% a ... vector containing the acceleration profile of the traj.
% v ... vector containing the velocity profile of the trajectory
% s ... interpolation vector for the trajectory
function [t,a,v,s] = create_lin_intvec(tx,ax,tIpo)
i = 0;
s0 = 0;
v0 = 0;
startTime = 0;
% go through a, c and d
for j = 1 : size(tx,1)
        startTime = sumUp(j,tx);
    for timeStep = 0:tIpo:tx(j)
        i = i + 1;
        t(i,1) = startTime + timeStep;
        a(i,1) = ax(j);
        v(i,1) = v0+a(i,1)*timeStep;
        s(i,1) = s0+v0*timeStep+a(i,1)/2*timeStep^2;
    end
    %i = i -1;
    v0 = v(i,1);
    s0 = s(i,1);
    startTime = t(i,1);
    i = i -1;
end
end

function f = sumUp(k,tx)
temp = 0;
if (k ~= 1)
    temp = temp + tx(k-1,1) + sumUp(k-1,tx);
    %temp =  tx(k-1,1) + sumUp(k-1,tx);
end
f = temp;
end