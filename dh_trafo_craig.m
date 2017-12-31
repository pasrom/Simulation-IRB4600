%% M5) Write a Matlab function returning the 4x4 transformation matrix T for given DH parameters according to Craig?s interpretation).
function T = dh_trafo_craig(alpha,a,d,theta)      
     T = rotX(alpha) * trans(a,0,0) * rotZ(theta) * trans(0,0,d);
end