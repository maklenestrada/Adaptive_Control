function qd = DesiredTraj_qd(t)
    qd1 = 0.25*cos(0.5*t) + 0.25*sin(pi/2*t) + 0.25*cos(exp(1)/2*t) + 0.25*cos(1/sqrt(2)*t);
    qd2 = 0.5*cos(t) + 0.5*sin(sqrt(2)*t) + 0.5*sin(exp(1)*t) + 0.5*cos(pi*t);
    qd = [qd1; qd2];
end