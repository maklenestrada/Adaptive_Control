clear all;clc;close all
%% Initial Condition
r1 = 0.1;
r2 = 0.1;
r = [r1;r2];

e1 = 0;
e2 = 0;
e = [e1;e2];

Theta_Tilde = zeros(9,1);

G = Gains();
Gamma = G.Gamma;

Y1fTilde = zeros(2,9);
Yf = zeros(2,9);
Ydf = zeros(2,9);
uf = zeros(2,1);

z0 = [r;e;Theta_Tilde];
z0_TF = [r;e;Theta_Tilde;Yf(:);uf];
z0_CompDCAL = [r;e;Theta_Tilde;Y1fTilde(:);Yf(:);Ydf(:);uf];

%% Run Sim
dt = 0.1;
tspan = 0:dt:60;
[tSim,zSim] = ode45(@(t,z) Closed_Loop_Traditional(t,z), tspan,z0);
[tSim_Comp,zSim_Comp] = ode45(@(t,z) Closed_Loop_Composite(t,z), tspan,z0_TF);
[tSim_DCAL,zSim_DCAL] = ode45(@(t,z) Closed_Loop_Traditional_DCAL(t,z), tspan,z0);
[tSim_CompDCAL,zSim_CompDCAL] = ode45(@(t,z) Closed_Loop_Composite_DCAL(t,z), tspan,z0_CompDCAL);

%% Extract errors
% Traditional Adaptive Controller
e1_trad = zSim(:,3);
e2_trad = zSim(:,4);

% Torque-Filtered / Composite Adaptive Controller
e1_TF = zSim_Comp(:,3);
e2_TF = zSim_Comp(:,4);

% Traditional Adaptive Controller DCAL
e1_DCAL = zSim_DCAL(:,3);
e2_DCAL = zSim_DCAL(:,4);

% Composite Adaptive Controller DCAL
e1_CompDCAL = zSim_CompDCAL(:,3);
e2_CompDCAL = zSim_CompDCAL(:,4);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Traditional Adaptive Controller
e1_trad = log10(abs(e1_trad));
e2_trad = log10(abs(e2_trad));

% Torque-Filtered / Composite Adaptive Controller
e1_TF = log10(abs(e1_TF));
e2_TF = log10(abs(e2_TF));

% Traditional Adaptive Controller DCAL
e1_DCAL = log10(abs(e1_DCAL));
e2_DCAL = log10(abs(e2_DCAL));

% Composite Adaptive Controller DCAL
e1_CompDCAL = log10(abs(e1_CompDCAL));
e2_CompDCAL = log10(abs(e2_CompDCAL));

% Plot Tracking Errors in Subplots
figure;
subplot(2,1,1)
plot(tSim, e1_trad, 'r', 'LineWidth', 2); 
hold on;
plot(tSim_DCAL, e1_DCAL, 'm', 'LineWidth', 2);
plot(tSim_Comp, e1_TF, 'b', 'LineWidth', 2);
plot(tSim_CompDCAL, e1_CompDCAL, 'c', 'LineWidth', 2);
grid on;
ylabel('e_1');
title('Tracking Error e_1 — Trajectory 2 (High Excitation)');
legend('Traditional','Traditional with DCAL','Composite','Composite with DCAL');

subplot(2,1,2)
plot(tSim, e2_trad, 'r', 'LineWidth', 2); 
hold on;
plot(tSim_DCAL, e2_DCAL, 'm', 'LineWidth', 2);
plot(tSim_Comp, e2_TF, 'b', 'LineWidth', 2);
plot(tSim_CompDCAL, e1_CompDCAL, 'c', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('e_2');
title('Tracking Error e_2 — Trajectory 2 (High Excitation)');
legend('Traditional','Traditional with DCAL','Composite','Composite with DCAL');

% Plot Tracking Errors in Subplots
figure;
subplot(2,1,1)
plot(tSim, e1_trad, 'k', 'LineWidth', 2); 
hold on;
plot(tSim_DCAL, e1_DCAL, 'b', 'LineWidth', 2);
grid on;
ylabel('e_1');
title('Tracking Error e_1 — Trajectory 2 (High Excitation)');
legend('Traditional','Traditional with DCAL');

subplot(2,1,2)
plot(tSim, e2_trad, 'k', 'LineWidth', 2); 
hold on;
plot(tSim_DCAL, e2_DCAL, 'b', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('e_2');
title('Tracking Error e_2 — Trajectory 2 (High Excitation) ');
legend('Traditional','Traditional with DCAL');

figure;
subplot(2,1,1)
plot(tSim_Comp, e1_TF, 'k', 'LineWidth', 2);
hold on;
plot(tSim_CompDCAL, e1_CompDCAL, 'b', 'LineWidth', 2);
grid on;
ylabel('e_1');
title('Tracking Error e_1 — Trajectory 2 (High Excitation)');
legend('Composite','Composite with DCAL');

subplot(2,1,2)
plot(tSim_Comp, e2_TF, 'k', 'LineWidth', 2);
hold on;
plot(tSim_CompDCAL, e1_CompDCAL, 'b', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('e_2');
title('Tracking Error e_2 — Trajectory 2 (High Excitation)');
legend('Composite','Composite with DCAL');

%% Extract Theta_Tilde blocks
Theta_Tilde_Trad = zSim(:,5:13);
Theta_Tilde_TF   = zSim_Comp(:,5:13);
Theta_Tilde_DCAL = zSim_DCAL(:,5:13);
Theta_Tilde_CompDCAL = zSim_CompDCAL(:,5:13);

% Compute 2-norm of Theta_Tilde at each time
norm_Trad = vecnorm(Theta_Tilde_Trad,2,2);
norm_TF   = vecnorm(Theta_Tilde_TF,2,2);
norm_DCAL = vecnorm(Theta_Tilde_DCAL,2,2);
norm_CompDCAL = vecnorm(Theta_Tilde_CompDCAL,2,2);

% eps_val = 0;
% 
% norm_Trad = log10(norm_Trad + eps_val);
% norm_TF   = log10(norm_TF   + eps_val);
% norm_LS   = log10(norm_LS   + eps_val);

figure
plot(tSim, norm_Trad,'r','LineWidth',2); 
hold on
plot(tSim_DCAL,norm_DCAL,'m','LineWidth',2);
plot(tSim_Comp, norm_TF,'b', 'LineWidth',2);
plot(tSim_CompDCAL,norm_CompDCAL,'c','LineWidth',2);
xlabel('Time (s)')
ylabel('$\|\tilde{\theta}\|$', 'Interpreter','latex')
title('Parameter Estimation — Trajectory 2 (High Excitation)')
legend('Traditional','Traditional with DCAL','Composite','Composite with DCAL');
grid on
xl = xlim;
xlim([1 xl(2)])


%% Q1 Part 2
T = 5;

%Length of time for sims 
N = length(tSim);
N_Comp = length(tSim_Comp);
N_DCAL = length(tSim_DCAL);
N_CompDCAL = length(tSim_CompDCAL);

%Traditional 
[lambda, Q] = ComputeEig(zSim,tSim,N,T,dt);

%Composite 
[lambda_Comp,Q_Comp] = ComputeEig(zSim_Comp,tSim_Comp,N_Comp,T,dt);

%Traditional DCAL 
[lambda_DCAL,Q_DCAL] = ComputeEig_DCAL(tSim_DCAL,N_DCAL,T,dt);

%Composite DCAL
[lambda_CompDCAL, Q_CompDCAL] = ComputeEig_DCAL(tSim_CompDCAL,N_CompDCAL,T,dt);


figure;
plot(tSim,          lambda,     'r',  'LineWidth', 2); 
hold on;
plot(tSim_DCAL,     lambda_DCAL,     'm',  'LineWidth', 2);
plot(tSim_Comp,     lambda_Comp,     'b',  'LineWidth', 2);
plot(tSim_CompDCAL, lambda_CompDCAL, 'c',  'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('\lambda_{min}(P)');
title('Minimum Eigenvalue with Trajectory 2 (High Excitation)');
legend('Traditional','Traditional with DCAL','Composite','Composite with DCAL');
grid on;
xl = xlim;
xlim([5 xl(2)])

function [lambda, Q] = ComputeEig(zSim,tSim,N,T,dt)
    G = Gains();
    alpha = G.alpha;
    Q = zeros(2,9,N);
    for i = 1:N
        r = zSim(i,1:2)';
        e = zSim(i,3:4)';

        qd = DesiredTraj_qd(tSim(i)); 
        qd_dot = DesiredTraj_qd_dot(tSim(i)); 
        qd_ddot = DesiredTraj_qd_ddot(tSim(i));
        
        q = qd - e;
        qdot = qd_dot - r + alpha*e;

        Q(:,:,i) = Ymatrix(q, qdot, qd, qd_dot, qd_ddot, alpha);
    end
    lambda = ComputeLambda(T,Q,N,dt);
end

function [lambda, Q] = ComputeEig_DCAL(tSim,N,T,dt)
    G = Gains();
    alpha = G.alpha;
    Q = zeros(2,9,N);
    for i = 1:N
        qd = DesiredTraj_qd(tSim(i)); 
        qd_dot = DesiredTraj_qd_dot(tSim(i)); 
        qd_ddot = DesiredTraj_qd_ddot(tSim(i));

        Q(:,:,i) = Ymatrix(qd, qd_dot, qd, qd_dot, qd_ddot, alpha);
    end
    lambda = ComputeLambda(T,Q,N,dt);
end

function lambda = ComputeLambda(T,Q,N,dt)
    lambda = zeros(N,1);
    NT = round(T/dt);
    for i = NT:N
        P = zeros(9);
        for j = (i - NT + 1):i
            Q_i = Q(:,:,j);
            P = P + (Q_i')*Q_i * dt;
        end
        lambda(i) = min(eig(P));
    end
end