%%%%%%%%%%%%%%%%%%%%%%%%%%
% Rotate around the z axis
% Author: Roman Passler
% Version 1
% 27.02.2017
%%%%%%%%%%%%%%%%%%%%%%%%%%

function T = rotZ(a)
    %T = [[cosd(a) sind(a) 0 0]' [-sind(a) cosd(a) 0 0]' [0 0 1 0]' [0 0 0 1]'];
    T = [cosd(a) -sind(a) 0 0; sind(a) cosd(a) 0 0; 0 0 1 0; 0 0 0 1];
end