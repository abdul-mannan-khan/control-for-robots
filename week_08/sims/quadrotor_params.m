function p = quadrotor_params()
p.m = 0.5;
p.J = 0.01;
p.g = 9.81;
p.l = 0.15;
p.T_hover = p.m * p.g;
end
