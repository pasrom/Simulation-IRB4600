%%%%%%%%%%%%%%%%%%%%%%%%%%
% Rotate around the y axis
% Author: Roman Passler
% Version 1
% 27.02.2017
%%%%%%%%%%%%%%%%%%%%%%%%%%

function T = rotY(a)
    %T = [[cosd(a) 0 -sind(a) 0]' [0 1 0 0]' [sind(a) 0 cosd(a) 0]' [0 0 0 1]'];
    T=[cosd(a) 0 sind(a) 0; 0 1 0 0; -sind(a) 0 cosd(a) 0; 0 0 0 1];
end