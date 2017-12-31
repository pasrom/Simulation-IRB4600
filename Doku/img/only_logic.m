
t_ipo=[0.1 0.01 0.001]';
amax=250;
vc=250;
pose=3;
t_2_xyzabcPose = 2;

e{1}=[700, -750, 500, 0, 160, -85,0];
e{2}=[1000, 250, 1000, 0, 100, -25,vc];

robot = irb4600_robot();
robot.bas = trans(-400.1,-146.6,299.4);
robot.eff = trans(0,0,200);
erease = 0;

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