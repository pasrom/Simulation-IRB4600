% M7) Write a Matlab function to calculate the forward kinematics T of a robot which is given by the structure of robot parameters (robot) and the column vector with the joint variables (q).

function T = fk_craig(q,robot)

T = robot.bas;

for i = 1:length(q)
    % 1= Rotationsachse
    if robot.dhp(i,1) == 1
        alpha = robot.dhp(i,3);
        a = robot.dhp(i,4);
        d = robot.dhp(i,5);
        % Ausgangswinkel + Vorzeichen * Drehwinkel
        theta = robot.dhp(i,6)+(robot.dhp(i,2)*q(i,1));
        T = T * dh_trafo_craig(alpha,a,d,theta);
        % Translationsachse
    elseif dhp(i,1) == 2
            alpha = robot.dhp(i,3);  
            a = robot.dhp(i,4);
             % Verschiebung um d
            d = robot.dhp(i,5)+(robot.dhp(i,2)*q(i,1));
            theta = robot.dhp(i,6);
            T = T * dh_trafo_craig(alpha,a,d,theta);
        else     
            error('WRONG AXIS-TYPE DEFINED');
        end
    end
    T = T * robot.eff;
end