%% hausübung 2

%% init
clear all
clear variables
close all
clc

owner = 1;
%% plot latex style
set(groot,'defaulttextinterpreter','latex'); %alles latex :)
set(groot,'defaultLineLineWidth',1);
set(groot,'defaultFigureColor','w');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

%% begin the HÜ
t_ipo=[0.1 0.01 0.001]';
amax=250;
vc=250;
pose=1;
ks_length = 100;
erase = 0;
t_2_xyzabcPose = 2;
e{1}=[1000, 0, 1000, 0, 95, 35,0];
e{2}=[0, 1000, 500, -45, 175, 0,vc];
e{1}=[700, -750, 500, 0, 160, -85,0];
e{2}=[1000, 250, 1000, 0, 100, -25,vc];
% e{1}=[1000, 250, 1000, 0, 100, -25,0];
% e{2}=[700, -750, 500, 0, 160, -85,vc];
% e{1}=[1000, 250, 1000, 0, 100, -25,0];
% e{2}=[-700, 750, 1500, 0, 160, -85,vc];
% e{1}=[1500, 250, 1000, 0, 90, -2,0];
% e{2}=[-700, 1750, 1500, -30, 140, -85,vc]
e{1}=[1000, 0, 1000, 0, 175, 0,0];
e{2}=[0, 1000, 500, 0, 175, 0,vc];
nameA = 'eulerwinkel2';
nameB = 'achswinkel2';
nameC = 'euklAbstand2';
nameD = 'robot-irb4600-path2';

sTot = norm(e{1}(1:3) - e{2}(1:3));
%copyRight = '\textcopyright $\,$ Roman Passler';
colourCopy = [0.902 0.9020 0.902] ;

robot = irb4600_robot();
robot.bas = trans(-400.1,-146.6,299.4);
robot.eff = trans(0,0,200);
erease = 0;

descr = {
    char(strcat('Pose',{' '}, num2str(pose)));
    strcat('Baseframe= ', num2str(robot.bas(1:3,4)'));
    strcat('Effectorframe= ', num2str(robot.eff(1:3,4)'));
    strcat('Startpoint $\;$(',num2str(e{1}(1)), ',',num2str(e{1}(2)),',',num2str(e{1}(3)) ,',',num2str(e{1}(4)),',',num2str(e{1}(5)),',',num2str(e{1}(6)),')');
    strcat('Endpoint $\;$(',num2str(e{2}(1)), ',',num2str(e{2}(2)),',',num2str(e{2}(3)) ,',',num2str(e{2}(4)),',',num2str(e{2}(5)),',',num2str(e{2}(6)),')');
    strcat('$sTot= \;$ ', num2str(sTot) , ' $mm$');
    strcat('$v_{c}= \;$ ', num2str(vc), ' $mm/s$');
    strcat('$a_{max}= \;$ ', num2str(amax) , ' $mm/s^{2}$');
    };

for ctrTipo=1:length(t_ipo)
    for ii = 1:length(e)-1
        %% Pfad generieren
        [tx,ax] = create_lin_seg_list(e{ii}(1:6),e{ii+1}(1:6),e{ii+1}(7),amax,t_ipo(ctrTipo));
        [t,a,v,s] = create_lin_intvec(tx,ax,t_ipo(ctrTipo));
        ec = create_lin_path(e{ii}(1:6),e{ii+1}(1:6),s);
        % init variables -> should be faster...
        actualEC =  cell(1,length(ec{1}));
        analyticalQ = actualEC;
        realQGG = actualEC;
        qDot = actualEC;
        trajektSoll = actualEC;
        trajektIst = actualEC;
        euklAbstand = zeros(length(ec{1}),1);
        v0 = cell(1,length(ec{1})+1);
        difQ = v0;
        %% Rueckwaertskinematik anwenden
        tic
        for kk = 1:length(ec{1})
            tg = xyzabc_2_t(ec{1}(kk),ec{2}(kk),ec{3}(kk),ec{4}(kk),ec{5}(kk),ec{6}(kk));
            analyticalQ{kk} = irb4600_rk(tg,robot.bas,robot.eff,pose)';
            coor_wRe=fk_craig(analyticalQ{kk},robot);
            [ec_diffRe{1}(kk),ec_diffRe{2}(kk),ec_diffRe{3}(kk),ec_diffRe{4}(kk),ec_diffRe{5}(kk),ec_diffRe{6}(kk)] = t_2_xyzabc(coor_wRe, t_2_xyzabcPose);
            if kk == 1
                v0{kk}(:,1) = [0 0 0 0 0 0]';
                vNext = [0 0 0 0 0 0];
                % Die erste Position faken, da keine Wegmessung integriert ist
                difQ{kk} = analyticalQ{kk};
            end
            coor_w=fk_craig(difQ{kk},robot);
            [ec_diff{1}(kk),ec_diff{2}(kk),ec_diff{3}(kk),ec_diff{4}(kk),ec_diff{5}(kk),ec_diff{6}(kk)] = t_2_xyzabc(coor_w, t_2_xyzabcPose);
            trajektSoll{kk}  = [ec{1}(kk) ec{2}(kk) ec{3}(kk) ec{4}(kk) ec{5}(kk) ec{6}(kk)]';
            trajektIst{kk}  = [ec_diff{1}(kk) ec_diff{2}(kk) ec_diff{3}(kk) ec_diff{4}(kk) ec_diff{5}(kk) ec_diff{6}(kk)]';
            % dieses Delta rechnen ist zwar in der Simulation sinnlos,
            % aber bei einer realen Wegmessung sollte es so gemacht
            % werden.
            delta = trajektSoll{kk} - trajektIst{kk};
            actualEC{kk} = trajektIst{kk} + delta;
            if kk > 1
                vNext = (actualEC{kk}-actualEC{kk-1})/t_ipo(ctrTipo);
            end
            % output is degree, but we ned rad!
            omega = deg2rad(abc_t_w(actualEC{kk}, vNext(4), vNext(5), vNext(6)));
            v0{kk}(:,1) = [vNext(1); vNext(2); vNext(3); omega];
            qDot{kk}(:,1) = rad2deg((irb4600_jakobiCross(difQ{kk}, robot.bas, robot.eff))\v0{kk}(:));
            difQ{kk+1}(:,1) = difQ{kk} + qDot{kk}(:,1) * t_ipo(ctrTipo);
            euklAbstand(kk,1)=sqrt((ec{1}(kk)-ec_diff{1}(kk))^2+(ec{2}(kk)-ec_diff{2}(kk))^2+(ec{3}(kk)-ec_diff{3}(kk))^2);
        end
    end
    toc
    %%
    tic
    euklAbstandList{ctrTipo} = euklAbstand;
    ec_diffReList{ctrTipo} = ec_diffRe;
    ec_diffEcList{ctrTipo} = ec;
    ec_diffList{ctrTipo} = ec_diff;
    realQList{ctrTipo} = analyticalQ;
    qqList{ctrTipo} = difQ;
    tList{ctrTipo} = t;
    toc
    %%
    if ctrTipo > 0
        %break;
    end
end
%% draw robot two times
fig11 = figure(10+ctrTipo);
screensize = get( groot, 'Screensize' );
x0=2;
y0=1;
set(gcf,'units','points','position',[screensize(3)/3*x0,y0*screensize(4)/2,screensize(3)/3,screensize(4)/2.5])
draw_robot_pathNew(realQList{1},qqList{1},t_ipo(1),robot,ks_length,erase);
print(fig11,nameD, '-depsc');

if exist('owner')
    spalten = 1;
    faktor = 1.2;
else
    spalten = 2;
    faktor = 1.05;
end

%% plots eulerwinkel
legendA = [string('x'), string('y'), string('z'), string('$\alpha$'), string('$\beta$'), string('$\gamma$')];
labelYa = string('Weg $s$ in $mm$');
labelYb = string('Winkel in Grad');
labelY = [labelYa,labelYa,labelYa,labelYb,labelYb,labelYb];
for j = 1:length(ec_diffReList{1});
    fig1 = figure(1);
    screensize = get( groot, 'Screensize' );
    x0=0;
    y0=0;
    set(gcf,'units','points','position',[screensize(3)/3*x0,y0,screensize(3)/3,screensize(4)])
    
    subplot(length(analyticalQ{1})+1,spalten,j);
    p2(1) = plot(tList{1},ec_diffReList{1}{j},'DisplayName','analytical');
    hold on
    for jj = 1:length(ec_diffReList)
        p2(jj+1) = plot(tList{jj},ec_diffList{jj}{j},'DisplayName',char(char(strcat('@ $t_{ipo}$',{' '},num2str(t_ipo(jj))))));
    end
    if exist('owner')
        p2(jj+2) = plot(tList{1},ec_diffEcList{1}{j},'DisplayName','soll');
    end
    ylabel(labelY(j));
    xlabel('Zeit $t$ in $s$');
    title(legendA(j));
    xlim([0 tList{1}(end)]);
    grid on;
    if j == length(ec_diffReList{1});
        hL = subplot(length(analyticalQ{1})+1,spalten,7);
        poshL = get(hL,'position');     % Getting its position
        lgd =  legend(p2);
        lgdPos = get(lgd,'position');
        lgdPos = [poshL(1),poshL(2)*faktor,lgdPos(3),lgdPos(4)];
        set(lgd,'position',lgdPos);      % Adjusting legend's position
        axis(hL,'off');                 % Turning its axis off
        text(200,30,descr,'Interpreter','latex','Units','Pixels');
        set(gcf,'NextPlot','add');
        axes;
        h = title('Eulerwinkel - Verlauf \"uber die Zeit','Interpreter','latex');
        P = get(h,'Position');
        set(h,'Position',[P(1) P(2)+0.015 P(3)]);
        set(gca,'Visible','off');
        set(h,'Visible','on');
        if exist('copyRight')
            text(20,20,copyRight,'Interpreter','latex','Units','Pixels', 'FontSize', 50,'Color',colourCopy,'rotation', 45);
            text(20,320,copyRight,'Interpreter','latex','Units','Pixels', 'FontSize', 50,'Color',colourCopy,'rotation', 45);
            text(20,620,copyRight,'Interpreter','latex','Units','Pixels', 'FontSize', 50,'Color',colourCopy,'rotation', 45);
            
        end
    end
    % hold off
end
hold off;
%align_Ylabels(gcf);
print(fig1,nameA, '-depsc');
print(fig1,nameA, '-djpeg');
%% plots achs winkel
legendB = [string('1'), string('2'), string('3'), string('4'), string('5'), string('6')];

for j = 1:length(analyticalQ{1})
    fig2 = figure(2);
    x0=1;
    y0=0;
    set(gcf,'units','points','position',[screensize(3)/3*x0,y0,screensize(3)/3,screensize(4)])
    subplot(length(analyticalQ{1})+1,spalten,j);
    tb = [tList{1}'];
    for jj = 1: length(realQList{1});
        tempA(jj) = realQList{1}{jj}(j);
    end
    p1(1) = plot(tb,tempA,'DisplayName',strcat('@ analytical'));
    hold on;
    
    for jjj = 1:length(ec_diffReList)
        tNew = [tList{jjj}' tList{jjj}(end) + t_ipo(ctrTipo)];
        clear tempB;
        for jj = 1: length(qqList{jjj});
            tempB(jj) = qqList{jjj}{jj}(j);
        end
        p1(jjj+1) = plot(tNew,tempB,'DisplayName',char(strcat('@ $t_{ipo}$',{' '},num2str(t_ipo(jjj)))));
    end
    
    ylabel('Winkel in $^{\circ}$');
    xlabel('Zeit $t$ in $s$');
    title(strcat(legendB(j)));
    xlim([0 tNew(end)]);
    grid on;
    if j == length(analyticalQ{1})
        hL = subplot(length(analyticalQ{1})+1,spalten,7);
        poshL = get(hL,'position');     % Getting its position
        lgd = legend(p1);
        lgdPos = get(lgd,'position');
        lgdPos = [poshL(1),poshL(2)*faktor,lgdPos(3),lgdPos(4)];
        set(lgd,'position',lgdPos);      % Adjusting legend's position
        axis(hL,'off');                 % Turning its axis off
        text(200,30,descr,'Interpreter','latex','Units','Pixels');
        set(gcf,'NextPlot','add');
        axes;
        h = title('Achswinkel - Verlauf \"uber die Zeit','Interpreter','latex');
        P = get(h,'Position');
        set(h,'Position',[P(1) P(2)+0.015 P(3)]);
        set(gca,'Visible','off');
        set(h,'Visible','on');
        if exist('copyRight')
            text(20,20,copyRight,'Interpreter','latex','Units','Pixels', 'FontSize', 50,'Color',colourCopy,'rotation', 45);
            text(20,320,copyRight,'Interpreter','latex','Units','Pixels', 'FontSize', 50,'Color',colourCopy,'rotation', 45);
            text(20,620,copyRight,'Interpreter','latex','Units','Pixels', 'FontSize', 50,'Color',colourCopy,'rotation', 45);
            
        end
    end
end
hold off;
if exist('owner')
    align_Ylabels(gcf) ;
end
print(fig2,nameB, '-depsc');
print(fig2,nameB, '-djpeg');
%% plot euklAbstand

fig3 = figure(5);
x0=2;
y0=0;
set(gcf,'units','points','position',[screensize(3)/3*x0,y0,screensize(3)/3,screensize(4)/2.5]);
hold on;
for jj = 1 : length (euklAbstandList)
    p(jj) = plot(tList{jj},euklAbstandList{jj},'DisplayName',char(strcat('@ $t_{ipo}$',{' '},num2str(t_ipo(jj)))));
end
xlim([0 tList{jj}(end)]);
legend(p,'location','southoutside','Orientation','horizontal');
title('euklidischer Abstand','Interpreter','latex');
ylabel('Weg $s$ in $mm$');
xlabel('Zeit $t$ in $s$');
text(210,80,descr,'Interpreter','latex','Units','Pixels');
if exist('copyRight')
    text(20,20,copyRight,'Interpreter','latex','Units','Pixels', 'FontSize', 50,'Color',colourCopy,'rotation', 45);
end
hold off
print(fig3,nameC,'-depsc');
print(fig3,nameC, '-djpeg');
