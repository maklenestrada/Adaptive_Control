function Theta = Theta_Values()
% Robot parameters 
p1 = 3.473;
p2 = 0.196;
p3 = 0.242;
fd1 = 5.3;
fd2 = 1.1;
fk1 = 5;
fk2 = 0.5;
fs1 = 8.45;
fs2 = 2.35;

Theta     = [p1;p2;p3;...
             fd1;fd2;...
             fk1;fk2;...
             fs1;fs2];
end