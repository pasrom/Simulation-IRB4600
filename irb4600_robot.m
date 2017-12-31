% robot.dhp DH parameters, [n 6] Array
% colums: type sign alpha a d theta
% (1/2) (1/-1)
% 1...rotational axis
% 2...translational axis
%
% robot.eff: tool frame (in flange coordinates)
% [4 4] Array
%
% robot.bas: base frame (in world coordinates)
% [4 4] Array
%
% ALL ANGLES IN DEG!!

function robot = irb4600_robot()
    
    robot.dhp = [   1, 1, 0, 0, 495, 0
                    1, 1, -90, 175, 0, 0
                    1, 1, 0, 900, 0, -90
                    1, 1, 90, 175, -960, 0
                    1, 1, -90, 0, 0, 0
                    1, 1, -90, 0, 135, 180];
    % robot.eff tool
             
    x1 = 0;
    y1 = 0;
    z1 = 0;
    a1 = 0;
    b1 = 0;
    c1 = 0;
             
    robot.eff = xyzabc_2_t(x1,y1,z1,a1,b1,c1);
    
    % robot.bas basis
    
    x2 = 0;
    y2 = 0;
    z2 = 0;
    a2 = 0;
    b2 = 0;
    c2 = 0;
    
    robot.bas = xyzabc_2_t(x2,y2,z2,a2,b2,c2);
end