%% M9
% Input parameters:
% e1 ... vector with euler coordinates of the start position
% e2 ... vector with euler coordinates of the end position
% vc ... speed in the linear segment of the trajectory
% amax ... maximum acceleration to be used
% t_ipo ... interpolation clock
% Output parameters:
% tx ... column vector with the duration of each segment
% ax ... column vector with the acceleration used for each segment

function [tx,ax] = create_lin_seg_list(e1,e2,vC,aMax,tIpo)
e1 = [transpose(e1(1:3)) transpose(e1(4:6))];
e2 = [transpose(e2(1:3)) transpose(e2(4:6))];
sTot = norm(e1(:,1) - e2(:,1));
% von Hand:
%temp = endpunkt(:,1) - startpunkt(:,1)
%temp = sqrt(temp(1)^2 + temp(2)^2 + temp(3)^2)

ta = vC/aMax;
sCrit  = vC^2/aMax;

if sCrit < sTot
    disp('Trapez');
    tc = sTot/vC - ta;
else
    disp('Dreieck');
    ta = sqrt(sTot/aMax);
end

% aufrunden auf ein vielfaches von tIpo
rest = mod (ta, tIpo) ;
if rest == 0
    taIpo = ta - rest;
else
    taIpo = ta - rest + tIpo;
end
ceil(ta/tIpo)*tIpo

if sCrit < sTot
    tcIpo = tc -  mod (tc, tIpo) + tIpo;
else
    tcIpo = 0;
end
sa = vC^2/(2 * aMax);
sc = sTot - 2 * sa;

aNew = sTot / ( taIpo^2 + taIpo * tcIpo);
vCNew = aNew * taIpo;
saNew = vCNew^2/(2 * aNew);
scNew = sTot - 2 * saNew;

if sCrit >= sTot
    tc = 0;
    scNew = 0;
end

tx = [taIpo;tcIpo;taIpo]; %Ausgabe
ax = [aNew;0;-aNew];
end