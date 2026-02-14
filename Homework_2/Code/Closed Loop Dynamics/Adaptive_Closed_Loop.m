function EtaDot = Adaptive_Closed_Loop(Y,Yf,uf,tau,Gamma,Theta_Tilde)
    G = Gains();
    beta = G.beta;
    kc   = G.kc;
    v    = G.v;
    Gamma_UppBound = G.Gamma_UppBound;

    % Yf
    Yfdot = -beta*Yf + beta*Y;

    % uf
    ufdot = -beta*uf + beta*tau;

    % Gamma 
    Gamma_Norm = norm(Gamma,'fro');
    if(Gamma_Norm < Gamma_UppBound)
        Gammadot = beta*Gamma - kc * Gamma * ((Yf')*Yf) / (1 + v*(norm(Yf)')*norm(Yf)) * Theta_Tilde;
    else
        Gammadot = zeros(9,9);
    end

    EtaDot = [Yfdot(:);ufdot;Gammadot(:)];
end