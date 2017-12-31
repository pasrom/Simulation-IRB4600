%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% Author: Roman Passler
% Version 1
% 27.02.2017
%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x, y, z, a, b, c] = t_2_xyzabc(T, pose)
    %Koordinaten auslesen
    x = T(1,4);
    y = T(2,4);
    z = T(3,4);
    r11 = T(1,1);
    r21 = T(2,1);
    r31 = T(3,1);
    r12 = T(1,2);
    r22 = T(2,2);
    r32 = T(3,2);
    r13 = T(1,3);
    r23 = T(2,2);
    r33 = T(3,3);

    %Winkel auslesen
    if pose == 1
        b = atan2d(-r31,sqrt(r11^2+r21^2));
    elseif pose == 2
        b = atan2d(-r31,-sqrt(r11^2+r21^2));
    end

    if b >= 89.999 && b <= 90.001
        a = 0;
        b = 90;
        c = atan2d(r12,r22);
    elseif b <= -89.999 && b >= -90.001
        a = 0;
        b = -90;
        c = -atan2d(r12,r22);
    else
        a = atan2d(r21/cosd(b),r11/cosd(b));
        c = atan2d(r32/cosd(b),r33/cosd(b));
    end
end
