function dx = quadrotor_dyn(x, u, p)
% Planar quadrotor: x = [px; pz; phi; vx; vz; dphi]
% u = [T; tau]  (thrust, torque). p: struct with m, J, g, l
m = p.m; J = p.J; g = p.g;
phi = x(3); vx = x(4); vz = x(5); dphi = x(6);
T = u(1); tau = u(2);
dx = [ vx;
       vz;
       dphi;
      -T*sin(phi)/m;
       T*cos(phi)/m - g;
       tau/J ];
end
