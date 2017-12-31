% M8) Write a Matlab function to calculate the 
% backward Kinematics of the IRB 4600?60. The function 
% v shall return the column vector of the joint variables (q)
% for a given pose of the robot if the end?
% effector reaches a goal TG. 

function q = irb4600_rk(tg,bas,eff,pose)
%IRB4600	Backwards Kinematics of the ABB IRB4600-60 Manipulator
%
%	Q = IRB4600_RK(TG,BAS,EFF,POSE)  calculates the joint angles Q of the Backward Kinematic Solution
%	of the ABB IRB4600-60 manipulator for a given goal transformation T and
%	desired configuration POSE
%   Limits of axis angles are currently not taken into account
%	
%	tg: goal frame
%   bas: base frame
%   eff: tool frame
%   pose: is the desired robot configuration
% 
%   pose    q1      Elbow   wrist
%    1      s1      s1      s1
%    2      s1      s1      s2
%    3      s1      s2      s1
%    4      s1      s2      s2
%    5      s2      s2      s1
%    6      s2      s2      s2
%    7      s2      s1      s1
%    8      s2      s1      s2
%
%
%   ALL ANGLES IN DEGREES !!
%
%	Copyright (c) 2010, 2016 by Robert Merz


narginchk(4,4);

[m,n] = size(bas);
if (n ~= 4)|(m ~= 4)
	error('Base Frame must be 4 by 4');
end

[m,n] = size(eff);
if (n ~= 4)|(m ~= 4)
	error('Tool Frame must be 4 by 4');
end

[m,n] = size(tg);
if (n ~= 4)|(m ~= 4)
	error('Goal Frame must be 4 by 4');
end

if (pose<1)|(pose>8)
	error('Configuration must be between 1 and 8');
end

% DH-parameters for ABB IRB4600-60
d1=495;
a1=175;
a2=900;
a3=175;
d4=960;
d6=135;

%Calculate the coordinate frame of the flange {6} in {0}
%	tg: goal frame
%   bas: base frame
%   eff: tool frame
%   T.0F = base frame^-1 * goal frame * tool frame^-1

%t06Old=inv(bas)*tg*inv(eff);   % bas=eff when tool is exactly at the same position as the wrist frame!

t06=(bas)\tg/(eff);
%Calculate the origin of Frame {4} in {0}
%   P.4F = [0; 0; -d6; 1]
p04=t06*[0,0,-d6,1]'; 

  
%Calculate theta1 (two solutions)
if pose<5
    %   solution 1 for theta1
    q(1)=atan2d(p04(2,1),p04(1,1)); 
else
    %   solution 2 for theta1
	q(1)=atan2d(-p04(2,1),-p04(1,1));
end 

%Calculate theta2
r=p04(1,1)*cosd(q(1))+p04(2,1)*sind(q(1));
k1=r-a1;
k2=d1-p04(3,1);
k3=((r-a1)^2+(d1-p04(3,1))^2+a2^2-a3^2-d4^2)/(2*a2);

%check if a solution is possible

if  (k1^2+k2^2-k3^2) < 0 % no solution possible !!!!!!!!!!!!!!!!!!!!!!
    disp('no solution for theta2 and theta3');
    for j=1:6
       q(j)=NaN;
    end
else
%there are two solutions for theta2   
   	if (pose==1)|(pose==2)|(pose==5)|(pose==6)
            %   solution 1 for theta2
			q(2)=atan2d(k2,k1)+atan2d(sqrt(k1^2+k2^2-k3^2),k3);
    else
            %   solution 2 for theta2
			q(2)=atan2d(k2,k1)-atan2d(sqrt(k1^2+k2^2-k3^2),k3); 
    end
    
%check if -180 < theta2 < 180
if q(2)>180 q(2)=q(2)-360;
elseif q(2)<-180 q(2)=q(2)+360;
end

% Calculate theta3
k4=-a1-a2*cosd(q(2))+r;
k5=-a2*sind(q(2))+d1-p04(3,1);

q(3)=-q(2)+atan2d(d4*k5+a3*k4,d4*k4-a3*k5); % ***** there is only one solution for theta3

%check if -180 < theta3 < 180
if q(3)<-180
    q(3)=360+q(3);
elseif q(3)>180
   	q(3)=q(3)-360;
end
	
%Calculate forward kinematics t03

% DH-Parameter
%   type  sign alpha a  d   theta
%     1     1   0   0   d1  0; 
%     1     1   -90 a1  0   0; 
%     1     1   0   a2  0   -90; 

% T = dh_trafo_craig(alpha,a,d,theta)
t01=dh_trafo_craig(0,0,d1,q(1));
t12=dh_trafo_craig(-90,a1,0,q(2));
t23=dh_trafo_craig(0,a2,0,q(3)-90);

t03= t01*t12*t23;

%Calculate t36
r06=t06(1:3,1:3);
r03=t03(1:3,1:3);

t36=(r03)\r06;
      
%Calculate theta4-6 - they are the zyz-Euler Angles of t36
if abs(t36(2,3))>0.999
	disp('sherical wrist in singularity');
	q(6)=0;
	if t36(2,3)>0
   		q(5)=0;
   		q(4)=atan2d(-t36(3,1),t36(3,2));
    else
        q(5)=180;
      	q(4)=atan2d(-t36(3,1),t36(1,1));
    end   
    
    
else % Calculate theta5, theta4, theta6 when sperhical wrist not in singularity
   	if mod(pose,2)==1
        %   solution 1 for theta 5
      	q(5)=atan2d(+sqrt(t36(2,1)^2+t36(2,2)^2),t36(2,3)); 
    else
        %   solution 2 for theta 5
   		q(5)=atan2d(-sqrt(t36(2,1)^2+t36(2,2)^2),t36(2,3)); 
    end
    %   solution for theta 4
   	q(4)=atan2d(-t36(3,3)/sind(q(5)),-t36(1,3)/sind(q(5)));
    %   solution for theta 6
   	q(6)=atan2d(t36(2,2)/sind(q(5)),-t36(2,1)/sind(q(5)));
	end

end