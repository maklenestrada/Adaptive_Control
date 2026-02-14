function D = Dynamics(q,q_dot,Theta)
q1 = q(1);
q2 = q(2);
q1Dot = q_dot(1);
q2Dot = q_dot(2);

p1 =  Theta(1);
p2 =  Theta(2);
p3 =  Theta(3);
fd1 = Theta(4);
fd2 = Theta(5);
fk1 = Theta(6);
fk2 = Theta(7);
fs1 = Theta(8);
fs2 = Theta(9);

c2 = cos(q2);
s2 = sin(q2);

% M(q)
M11 = p1 + 2*p3*c2;
M12 = p2 + p3*c2;
M21 = p2 + p3*c2;
M22 = p2;
M = [M11 M12; M21 M22];

% Analytical inverse of the inertia matrix
Minv = (1/(p2^2 - p1*p2 + p3^2*cos(q(2))^2))*... 
    [-p2               p2+p3*cos(q(2))  ;...
        p2+p3*cos(q(2)) -p1-2*p3*cos(q(2))];

% Time derivative of M(q)
dM11 = -2 * p3 * sin(q2) * q2Dot;
dM12 = -1 * p3 * sin(q2) * q2Dot;
dM21 = -1 * p3 * sin(q2) * q2Dot;
dM22 = 0;

Mdot = [dM11 dM12; dM21 dM22];

% Vm(q,qdot)
Vm11 = -p3*s2*q2Dot;
Vm12 = -p3*s2*(q1Dot + q2Dot);
Vm21 = p3*s2*q1Dot;
Vm22 = 0;
Vm = [Vm11 Vm12; Vm21 Vm22];

% Fd(qdot)
Fd = [fd1*q1Dot; fd2*q2Dot];

% Fs(qdot)
Fs = [fs1*tanh(q1Dot); fs2*tanh(q2Dot)];

% Fk(q) 
Fk = [fk1*q1; fk2*q2];

D.M = M;
D.Minv = Minv;
D.Mdot = Mdot;
D.Vm = Vm;
D.Fd = Fd;
D.Fs = Fs;
D.Fk = Fk;
end