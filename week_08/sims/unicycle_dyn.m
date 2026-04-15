function dx = unicycle_dyn(x, u)
% Unicycle mobile robot: x = [px; py; theta], u = [v; omega]
dx = [ u(1)*cos(x(3));
       u(1)*sin(x(3));
       u(2) ];
end
