%%%%%%%%%%%%%%%%%%%%%%%%%%
% Rotate around the x axis
% Author: Roman Passler
% Version 1
% 27.02.2017
%%%%%%%%%%%%%%%%%%%%%%%%%%

function T = rotX(a)    
   % T = [[1 0 0 0]' [0 cosd(a) sind(a) 0]' [0 -sind(a) cosd(a) 0]' [0 0 0 1]'];
   T=[1 0 0 0; 0 cosd(a) -sind(a) 0; 0 sind(a) cosd(a) 0; 0 0 0 1]; % Rotationsmatrix um x
end