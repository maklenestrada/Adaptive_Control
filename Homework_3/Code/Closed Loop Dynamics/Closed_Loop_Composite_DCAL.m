function zdot = Closed_Loop_Composite_DCAL(t,z)
%% Extract State Vector 
r = [z(1);z(2)];
e = [z(3);z(4)];
Theta_Tilde = [z(5);z(6);z(7);z(8);z(9);z(10);z(11);z(12);z(13)];

%% Extract gains
G = Gains();
kr = G.kr;
ke = G.ke;
keps = G.keps;
Gamma = G.Gamma;
alpha = G.alpha;
beta = G.beta;

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
M = D.M;
Minv = D.Minv;
Mdot = D.Mdot;
Vm = D.Vm;

%% Compute regressor matrix
Y = Ymatrix(q, q_dot, qd, qd_dot, qd_ddot, alpha);
Yd = Ymatrix(qd, qd_dot, qd, qd_dot, qd_ddot, alpha);
%% Controller
tau = Yd*Theta_Hat + kr*r + ke*e;

% Apply torque saturation
tauMax = [250; 30];
tau = max(min(tau, tauMax), -tauMax);
%% Yf
Yf = reshape(z(14:31),2,9);

Yfdot = -beta*Yf + beta*Y;

%% Y1f 
Y1fTilde = reshape(z(34:51),2,9);

%% Y1Tilde & Y2Tilde
Y1d = Yd;
Y2d = Yd;
Y1 = Y;
Y2 = Y;

Y1Tilde = Y1d - Y1;
Y2Tilde = Y2d - Y2;

YfTilde = Y1fTilde + Y2Tilde;

%% Ydf
v = reshape(z(52:69),2,9);
vdot = -beta*v + beta*Y1d;
Ydf = v + Y2d;

%% uf
uf = [z(32);z(33)];

ufdot = -beta*uf + beta*tau;
%% Test Code
uf - Yf*Theta

%% Closed Loop Dynamics 
rdot = Minv*(Y*Theta - tau - Vm*r);
edot = r - alpha*e;
ThetaTildedot = -Gamma*Yd'*r - Gamma*(Ydf')*Ydf*Theta_Tilde + keps*Gamma*(Ydf')*YfTilde*Theta;
Y1fTildedot = - beta*Y1fTilde + beta*Y1Tilde;

zdot = [rdot;edot;ThetaTildedot;Yfdot(:);ufdot;Y1fTildedot(:);vdot(:)];
end