%%
% Test Functions
% Author: Roman Passler
% Version 1
% 27.02.2017
clear all;
close all;
% don't show the plots in window, save only to file
set(0,'DefaultFigureVisible','off')
% everything in latex style :)
set(groot,'defaulttextinterpreter','latex'); %alles latex :)
%set(groot,'defaultLineLineWidth',1);
set(groot,'defaultFigureColor','w');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(0,'defaultTextInterpreter','latex'); %trying to set the default
clearvars
%%
strcat(pwd, '/matlab_files')
addpath(strcat(pwd, '/matlab_files'))

A = [-6 -3; 1 2; -2 2]
c = (transpose(A)*A)^(-1) * transpose(A)
b = [8 -11 8]'

%% M1 Rotation

a = 90;
Tx = rotX(a);
Bx = rotx(a);
Ty = rotY(a);
By = roty(a);
Tz = rotZ(a);
Bz = rotz(a);

%% M2 translation
x = 10;
y = 0;
z = 5;
Tt = trans(x,y,z);

%% M3
a = 30;
vy = 20;
vz = 10;
x = -14;
y = 16;
z = 15.9;

T = xyzabc_2_t(x,y,z,a,vy,vz);

%% M4
pose =2;
[x,y,z,a,vy,vz] = t_2_xyzabc(T, pose)

%% M5
alpha = 90;
a = 20;
d = 3;
theta = 0;
T = dh_trafo_craig(alpha,a,d,theta);

%% M6

robot = irb4600_robot();
q = [0;-90;90;0;0;0];

[koor] = coortraf_craig(q,robot);
axis([-2000 2000 -2000 2000 -0.2 2000]);
view([102,20]);
grid on; xlabel('X'); ylabel('Y'); zlabel('Z');
ks_length = 100;
draw_kin(koor,ks_length)
figure(1)
xlim([-2000 2000])
ylim([-2000 2000])
zlim([-1 2000])
view([60 30])

%% M7

T = fk_craig(q,robot)

%% m8 test

robot = irb4600_robot();
tg=xyzabc_2_t(1000,0,1000,0,0,0)
bas=trans(-400.1, -146.6, 299.4);
eff=trans(0,0,200);
robot.bas=bas;
robot.eff=eff;
qback=irb4600_rk(tg, bas, eff, 1)';
[ x,y,z,a,b,c ]=t_2_xyzabc(fk_craig(qback,robot), 1)


%% M8


robot = irb4600_robot();
tg=xyzabc_2_t(1000,500,0,90,-90,0)
bas=trans(0, 0, 0);
eff=trans(0,0, 0);
robot.bas = bas;
robot.eff = eff;
qback=irb4600_rk(tg, robot.bas, robot.eff, 8)';
%qback=kr15_rk(tg, robot.bas, robot.eff, 2);
T = fk_craig(qback,robot)

%% m8 neu

 robot2 = irb4600_robot();
tgB = [0 0 1 1000; 0 1 0 500; -1 0 0 400; 0 0 0 1]
bas2 = [1 0 0 -400.1; 0 1 0 -146.6; 0 0 1 299.4; 0 0 0 1];
eff2 = [1 0 0 0; 0 1 0 0; 0 0 1 200; 0 0 0 1];
pose = 1;
robot2.bas = bas2;
robot2.eff = eff2;
qback2 = kr15_rk(tgB,robot2.bas,robot2.eff,pose);
T = fk_craig(qback2,robot2)

%% m9 - m11
e1 = [-500, 45
    500, 15
    250,  -45];
e2 = [100, 0
    200, -45
    0, 10];
vC = 250;
aMax = 150;
tIpo = 0.25;
tIpoSmall = 0.0025;

[tx,ax] = create_lin_seg_list(e1,e2,vC,aMax,tIpo);
[t,a,v,s] = create_lin_intvec(tx,ax,tIpo);
[txSmall,axSmall] = create_lin_seg_list(e1,e2,vC,aMax,tIpoSmall);
[tSmall,aSmall,vSmall,sSmall] = create_lin_intvec(txSmall,axSmall,tIpoSmall);
ec = create_lin_path(e1,e2,s);

%% m12

t_ipo=0.1;
amax=1500;
vc=1000;
pose=4;
e{1}=[1000, 0, 1000, 0, 95, 35,vc];
e{2}=[0, 1000, 500, -45, 175, 0,vc];
%e{1}=[1000,-1000,1000,0,45,0,0];
%e{2}=[500,0,200,0,90,0,vc];
%e{3}=[-1000,1000,1000,0,135,0,vc];
% e{4}=[1000,0,1500,0,135,0,vc];
% e{5}=[-1000,-1000,1000,0,135,0,vc];
% e{6}=[1000,-1000,1000,0,45,0,vc];


%% own tests

  qTest = [0 0 0 90 90 90]'
  abc_t_w(qTest, 3,4,3);

%% m14

q=[35;-17;120;45;-3;28];
bas=trans(0,0,0);
eff=trans(0,0,0);
j=irb4600_jakobi(q, bas, eff)
 
%%
for ii = 1:length(e)-1
    %% Pfad generieren
    e1 = e{ii}(1:6)';
    e2 = e{ii+1}(1:6)';
    
    e1 = [transpose(e{ii}(1:3)) transpose(e{ii}(4:6))];
    e2 = [transpose(e{ii+1}(1:3)) transpose(e{ii+1}(4:6))];
    
    [tx,ax] = create_lin_seg_list(e1,e2,e{ii+1}(7),amax,t_ipo);
    [t,a,v,s] = create_lin_intvec(tx,ax,t_ipo);
    ec = create_lin_path(e1,e2,s);
    
    %% Rueckwaertskinematik anwenden

    for kk = 1:length(ec{1})
        tg = xyzabc_2_t(ec{1}(kk),ec{2}(kk),ec{3}(kk),ec{4}(kk),ec{5}(kk),ec{6}(kk));
        %bas = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
        %eff = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
        bas=trans(0, 0, 0);
        eff=trans(0,0,0);
        qq{kk} = irb4600_rk(tg,bas,eff,pose)';
        if kk == 1
            v0{kk}(:) = [0 0 0 0 0 0]'; 
        else
            vx = (qq{kk}(1,1) - qq{kk-1}(1,1))/t_ipo;
            vy = (qq{kk}(2,1) - qq{kk-1}(2,1))/t_ipo;
            vz = (qq{kk}(3,1) - qq{kk-1}(3,1))/t_ipo;
            alphaP = (qq{kk}(4,1) - qq{kk-1}(4,1))/t_ipo;
            betaP = (qq{kk}(5,1) - qq{kk-1}(5,1))/t_ipo;
            gammaP = (qq{kk}(6,1) - qq{kk-1}(6,1))/t_ipo;
            omega = abc_t_w(qq{kk}, alphaP, betaP, gammaP);
            v0{kk}(:,1) = [vx; vy; vz; omega];
        end
        qDot{kk}(:,1) = irb4600_jakobi(qq{kk}, bas, eff)^(-1)*v0{kk}(:);
        qNeu{kk}(:,1) = qq{kk} + qDot{kk}(:,1) * t_ipo;
        euklAbstand{kk}(:,1) = norm(qq{kk}(1:3,1)) - norm(qNeu{kk}(1:3,1));
    end
    
    %% Roboter erzeugen
    robot = irb4600_robot();
    robot.bas = bas;
    robot.eff = eff;
    ks_length = 100;
    erase = 1;
    %%
    draw_robot_path(qq,t_ipo,robot,ks_length,erase);
    clear qq
end