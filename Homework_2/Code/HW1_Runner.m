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

z0_TF = [r;e;Theta_Tilde;Yf(:);uf];
z0 = [r;e;Theta_Tilde];

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
tspan = [0 100];
[tSim_TF,zSim_TF] = ode45(@(t,z) Closed_Loop_Composite(t,z), tspan,z0_TF);
[tSim,zSim] = ode45(@(t,z) Closed_Loop_Traditional(t,z), tspan,z0);

%% Extract errors
% Traditional Adaptive Controller
e1_trad = zSim(:,3);
e2_trad = zSim(:,4);

% Torque-Filtered / Composite Adaptive Controller
e1_TF = zSim_TF(:,3);
e2_TF = zSim_TF(:,4);

% Plot Tracking Errors in Subplots
figure;
subplot(2,1,1)
plot(tSim, e1_trad, 'k', 'LineWidth', 2); hold on;
plot(tSim_TF, e1_TF, 'b', 'LineWidth', 2);
grid on;
ylabel('e_1');
title('Tracking Error e_1: Traditional vs Torque-Filtered');
legend('Traditional','Torque-Filtered');

subplot(2,1,2)
plot(tSim, e2_trad, 'k', 'LineWidth', 2); hold on;
plot(tSim_TF, e2_TF, 'b', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('e_2');
title('Tracking Error e_2: Traditional vs Torque-Filtered');
legend('Traditional','Torque-Filtered');



