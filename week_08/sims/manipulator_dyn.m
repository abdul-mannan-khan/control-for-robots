function [M, C, G] = manipulator_dyn(q, dq, p)
% 2-link planar revolute manipulator dynamics (standard form)
% M(q) ddq + C(q,dq) dq + G(q) = tau
% p: struct with m1, m2, l1, l2, lc1, lc2, I1, I2, g

m1 = p.m1; m2 = p.m2; l1 = p.l1; l2 = p.l2;
lc1 = p.lc1; lc2 = p.lc2; I1 = p.I1; I2 = p.I2; g = p.g;
q1 = q(1); q2 = q(2); dq1 = dq(1); dq2 = dq(2);

a = I1 + I2 + m1*lc1^2 + m2*(l1^2 + lc2^2);
b = m2*l1*lc2;
d = I2 + m2*lc2^2;

M = [a + 2*b*cos(q2),  d + b*cos(q2);
     d + b*cos(q2),    d];

C = [-b*sin(q2)*dq2, -b*sin(q2)*(dq1+dq2);
      b*sin(q2)*dq1,  0];

G = [(m1*lc1 + m2*l1)*g*cos(q1) + m2*lc2*g*cos(q1+q2);
      m2*lc2*g*cos(q1+q2)];
end
