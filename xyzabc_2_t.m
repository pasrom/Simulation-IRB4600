%%%%%%%%%%%%%%%%%%%%%%%%%%
% m3
% Author: Roman Passler
% Version 1
% 27.02.2017
%%%%%%%%%%%%%%%%%%%%%%%%%%

function T = xyzabc_2_t(x,y,z,a,b,c)  
    T = rotZ(a) * rotY(b) * rotX(c);
    T(1:3,4) = [x;y;z];
    T(4,4)=1;
end