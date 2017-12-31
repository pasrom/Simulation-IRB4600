%%%%%%%%%%%%%%%%%%%%%%%%%%
% Translate a point 
% Author: Roman Passler
% Version 1
% 27.02.2017
%%%%%%%%%%%%%%%%%%%%%%%%%%

function T = trans(x, y, z)    
    T = [[1 0 0 0]' [0 1 0 0]' [0 0 1 0]' [x y z 1]'];
end