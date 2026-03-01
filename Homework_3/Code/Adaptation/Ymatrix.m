function Y = Ymatrix(q,qDot,qd,qdDot,qdDDot,alpha)
q1 = q(1);
q2 = q(2);

q1Dot = qDot(1);
q2Dot = qDot(2);

qd1 = qd(1);
qd2 = qd(2);

qd1Dot = qdDot(1);
qd2Dot = qdDot(2);

qd1DDot = qdDDot(1);
qd2DDot = qdDDot(2);

c2 = cos(q2);
s2 = sin(q2);

Y11 = -alpha*q1Dot + alpha*qd1Dot + qd1DDot;
Y12 = -alpha*q2Dot + alpha*qd2Dot + qd2DDot;
Y13 = alpha*q1Dot*q2*s2 - alpha*q1Dot*qd2Dot*s2 ...
      + alpha*q2Dot*q1*s2 + alpha*q2Dot*q2*s2 ...
      - alpha*q2Dot*qd1Dot*s2 - alpha*q2Dot*qd2Dot*s2 ...
      - 2*alpha*c2*q1Dot - alpha*c2*q2Dot ...
      + 2*alpha*c2*qd1Dot + alpha*c2*qd2Dot ...
      - q1Dot*qd2Dot*s2 - q2Dot*qd1Dot*s2 - q2Dot*qd2Dot*s2 ...
      + 2*c2*qd1DDot + c2*qd2DDot;
Y14 = q1Dot;
Y15 = 0;
Y16 = q1;
Y17 = 0;
Y18 = tanh(q1Dot);
Y19 = 0;

% Row 2
Y21 = 0;
Y22 = -alpha*q1Dot - alpha*q2Dot + alpha*qd1Dot + alpha*qd2Dot + qd1DDot + qd2DDot;
Y23 = -alpha*q1Dot*q1*s2 + alpha*q1Dot*qd1Dot*s2 ...
      - alpha*c2*q1Dot + alpha*c2*qd1Dot ...
      + q1Dot*qd1Dot*s2 + c2*qd1DDot;
Y24 = 0;
Y25 = q2Dot;
Y26 = 0;
Y27 = q2;
Y28 = 0;
Y29 = tanh(q2Dot);

% Regressor matrix
Y = [Y11 Y12 Y13 Y14 Y15 Y16 Y17 Y18 Y19;
     Y21 Y22 Y23 Y24 Y25 Y26 Y27 Y28 Y29];
end