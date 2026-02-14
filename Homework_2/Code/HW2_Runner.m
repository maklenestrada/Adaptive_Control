clear all;clc;close all
%% Initial Condition
r1 = 0.1;
r2 = 0.1;
r = [r1;r2];

e1 = 0;
e2 = 0;
e = [e1;e2];

Theta_Tilde = zeros(9,1);

Yf = zeros(2,9);

uf = zeros(2,1);

Gamma = 1e-9*eye(9);

z0 = [r;e;Theta_Tilde];
z0_TF = [r;e;Theta_Tilde;Yf(:);uf];
z0_LS = [r;e;Theta_Tilde;Yf(:);uf; Gamma(:)];

%% Solving for q,qdot
% Desired Trajectories qd,qd_dot,qd_ddot
t0 = 0;
qd = DesiredTraj_qd(t0);
qd_dot = DesiredTraj_qd_dot(t0);

%Value for alpha
G = Gains();
alpha = G.alpha;

q0 = qd - e;
q_dot0 = qd_dot - r + alpha*e;


%% Run Sim
tspan = [0 30];
[tSim,zSim] = ode45(@(t,z) Closed_Loop_Traditional(t,z), tspan,z0);
[tSim_TF,zSim_TF] = ode45(@(t,z) Closed_Loop_Composite(t,z), tspan,z0_TF);
[tSim_LS,zSim_LS] = ode45(@(t,z) Closed_Loop_LeastSquares(t,z), tspan,z0_LS);

%% Extract errors
% Traditional Adaptive Controller
e1_trad = zSim(:,3);
e2_trad = zSim(:,4);

% Torque-Filtered / Composite Adaptive Controller
e1_TF = zSim_TF(:,3);
e2_TF = zSim_TF(:,4);

% Least-Squares Adaptive Controller 
e1_LS = zSim_LS(:,3);
e2_LS = zSim_LS(:,4);

% Plot Tracking Errors in Subplots
figure;
subplot(2,1,1)
plot(tSim, e1_trad, 'k', 'LineWidth', 2); hold on;
plot(tSim_TF, e1_TF, 'r', 'LineWidth', 2);
plot(tSim_LS, e1_LS, 'b', 'LineWidth', 2);
grid on;
ylabel('e_1');
title('Tracking Error e_1');
legend('Traditional','Torque-Filtering','Least-Squares');

subplot(2,1,2)
plot(tSim, e2_trad, 'k', 'LineWidth', 2); hold on;
plot(tSim_TF, e2_TF, 'r', 'LineWidth', 2);
plot(tSim_LS, e2_LS, 'b', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('e_2');
title('Tracking Error e_2');
legend('Traditional','Torque-Filtering','Least-Squares');

%% Extract Theta_Tilde blocks
Theta_Tilde_Trad = zSim(:,5:13);
Theta_Tilde_TF   = zSim_TF(:,5:13);
Theta_Tilde_LS   = zSim_LS(:,5:13);

% Compute 2-norm of Theta_Tilde at each time
norm_Trad = vecnorm(Theta_Tilde_Trad,2,2);
norm_TF   = vecnorm(Theta_Tilde_TF,2,2);
norm_LS   = vecnorm(Theta_Tilde_LS,2,2);

figure
plot(tSim, norm_Trad,'k','LineWidth',2); 
hold on
plot(tSim_TF, norm_TF,'r', 'LineWidth',2);
plot(tSim_LS, norm_LS,'b', 'LineWidth',2);

xlabel('Time (s)')
ylabel('$\|\tilde{\theta}\|$', 'Interpreter','latex')
title('Parameter Estimation')
legend('Traditional','Torque-Filtering','Least-Squares');
grid on


