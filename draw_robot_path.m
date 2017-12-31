function draw_robot_path (q,t_ipo,robot,ks_length,erase)
%% m12
% Input parameters:
% q ... cell array of column vectors of all interpolated joint variables for the whole trajectory
% t_ipo ... interpolation clock
% robot: robot parameters
% ks_length: length for drawing frame axes
% erase: flag to clear screen for each interpolation
for ii = 1:length(q)
    if erase == 1
        clf;
    end
    [koor] = coortraf_craig(q{ii},robot);
    axis([-2000 2000 -2000 2000 -0.2 2000]);
    view([102,20]);
    grid on; xlabel('X'); ylabel('Y'); zlabel('Z');
    draw_kin(koor,ks_length)
    pause(t_ipo);
end
end