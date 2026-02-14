function zdot = Closed_Loop_Composite(t,z)
%% Extract State Vector 
r = [z(1);z(2)];
e = [z(3);z(4)];
Theta_Tilde = [z(5);z(6);z(7);z(8);z(9);z(10);z(11);z(12);z(13)];

%% Extract gains
G = Gains();
kr = G.kr;
ke = G.ke;
alpha = G.alpha;
kc = G.kc;
v = G.v;
%% Desired Trajectories qd,qd_dot,qd_ddot
qd = DesiredTraj_qd(t);
qd_dot = DesiredTraj_qd_dot(t);
qd_ddot = DesiredTraj_qd_ddot(t);

%% Solving for q,qdot
q = qd - e;
q_dot = qd_dot - r + alpha*e;

%% Solving for Theta_Hat
Theta = Theta_Values();
Theta_Hat = Theta - Theta_Tilde;

%% Dynamic Matricies
D = Dynamics(q,q_dot,Theta);
Minv = D.Minv;
Vm = D.Vm;

%% Compute regressor matrix
Y = Ymatrix(q, q_dot, qd, qd_dot, qd_ddot, alpha);

%% Controller
tau = Y*Theta_Hat + kr*r + ke*e;

% Apply torque saturation
tauMax = [250; 30];
tau = max(min(tau, tauMax), -tauMax);
%% Eta Dot
% EtaDot = [Yfdot
%           ufdot
%       GammaDot]

% Yf
Yf = reshape(z(14:31),2,9);

% uf
uf = [z(32);z(33)];

% Gamma
Gamma = reshape(z(34:114),9,9);

Etadot = Adaptive_Closed_Loop(Y,Yf,uf,tau,Gamma,Theta_Tilde)

%% Test Code
uf - Yf*Theta

%% Closed Loop Dynamics 
rdot = Minv*(Y*Theta - tau - Vm*r);
edot = r - alpha*e;
ThetaTildedot = -Gamma*Y'*r - (kc*Gamma*(Yf')*Yf*Theta_Tilde)/(1 + v*norm(Yf)'*norm(Yf));

zdot = [rdot;edot;ThetaTildedot;Etadot];
end




