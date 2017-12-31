%% M11
% Input parameters:
% e1 ... vector with euler coordinates for starting position
% e2 ... vector with euler coordinates for end position
% s ... interpolation vector for the trajectory
% Output parameters:
% ec ... cell array containing the vectors of euler coordinates for
% the interpolated path
function ec = create_lin_path(e1,e2,s)
e1 = [transpose(e1(1:3)) transpose(e1(4:6))];
e2 = [transpose(e2(1:3)) transpose(e2(4:6))];
sTot =  norm(e1(:,1) - e2(:,1));
for i = 1 : 6
    ec{i} = e1(i)+(e2(i)-e1(i))*s/sTot;
end
end