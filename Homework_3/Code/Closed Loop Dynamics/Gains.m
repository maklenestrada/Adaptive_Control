function G = Gains()
%kr Matrix
kr1 = 3;
kr2 = 1.5;

kr = [kr1 0; 0 kr2];

%ke Matrix 
ke1 = 1.5;
ke2 = 0.75;

ke = [ke1 0; 0 ke2]; 

%Gamma Matrix 9x9
Gamma = 5e-1*eye(9);

%Alpha
alpha = 1.5;

%% Torque-Filtering Gains
%Beta 
beta = 10;

%% Least-Squares Gains
%kepsilon
kc = 1;

keps = 0.05;

%nu
v = 1;

%Gamma Upper Bound
Gamma_UppBound = 1;

G.kr = kr;
G.ke = ke;
G.Gamma = Gamma;
G.alpha = alpha;
G.beta = beta;
G.kc = kc;
G.v = v;
G.keps = keps;
G.Gamma_UppBound = Gamma_UppBound;
end