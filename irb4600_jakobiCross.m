function J = irb4600_jakobiCross(q, bas, eff)

robot=irb4600_robot();
robot.bas=bas;
robot.eff=eff;
T1_0=fk_craig(q(1:1),robot);
T2_0=fk_craig(q(1:2),robot);
T3_0=fk_craig(q(1:3),robot);
T4_0=fk_craig(q(1:4),robot);
T5_0=fk_craig(q(1:5),robot);
T6_0=fk_craig(q(1:6),robot);

%Initalise
J=eye(6); 

%Kreuzprodukt Jacobi füllen
J(1:3,1)=cross(T1_0(1:3,3),(T6_0(1:3,4)-T1_0(1:3,4)));
J(1:3,2)=cross(T2_0(1:3,3),(T6_0(1:3,4)-T2_0(1:3,4)));
J(1:3,3)=cross(T3_0(1:3,3),(T6_0(1:3,4)-T3_0(1:3,4)));
J(1:3,4)=cross(T4_0(1:3,3),(T6_0(1:3,4)-T4_0(1:3,4)));
J(1:3,5)=cross(T5_0(1:3,3),(T6_0(1:3,4)-T5_0(1:3,4)));
J(1:3,6)=cross(T6_0(1:3,3),(T6_0(1:3,4)-T6_0(1:3,4)));

%Jacobi füllen
J(4:6,1)=T1_0(1:3,3);
J(4:6,2)=T2_0(1:3,3);
J(4:6,3)=T3_0(1:3,3);
J(4:6,4)=T4_0(1:3,3);
J(4:6,5)=T5_0(1:3,3);
J(4:6,6)=T6_0(1:3,3);

end
