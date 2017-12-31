%% alpha beta gamma Winkelgeschwidnigkeit zu omega

function w = abc_t_w(q,alphaP,betaP,gammaP)
    rotZalpha = rotZ(q(4,1));
    rotYbeta = rotY(q(5,1));
    w = [0 0 alphaP]' + rotZalpha(1:3,1:3) * [0 betaP 0]' + rotZalpha(1:3,1:3) * rotYbeta(1:3,1:3) * [gammaP 0 0]';
end