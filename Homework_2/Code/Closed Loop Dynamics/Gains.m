function G = Gains()
%kr Matrix
kr1 = 1;
kr2 = 0.5;

kr = [kr1 0; 0 kr2];

%ke Matrix 
ke1 = 1;
ke2 = 0.5;

ke = [ke1 0; 0 ke2]; 

%Gamma Matrix 9x9
Gamma = 1*eye(9);

%Alpha
alpha = 1;

%% Torque-Filtering Gains
%Beta 
beta = 1;

%% Least-Squares Gains
%kepsilon
kc = 1;

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
G.Gamma_UppBound = Gamma_UppBound;
end