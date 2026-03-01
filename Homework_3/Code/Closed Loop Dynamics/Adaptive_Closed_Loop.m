function EtaDot = Adaptive_Closed_Loop(Y,Yf,uf,tau)
    G = Gains();
    beta = G.beta;

    % Yf
    Yfdot = -beta*Yf + beta*Y;

    % Ydf
    Ydfdot = -beta*Ydf + beta*Yd;

    % uf
    ufdot = -beta*uf + beta*tau;

    EtaDot = [Yfdot(:);Ydfdot(:);ufdot];
end