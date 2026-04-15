function results = sim_robust(saveFig)
% ROBUST CONTROL for robotic systems.
%
% Robotic uncertainty sources modelled:
%   Manipulator : unknown payload mass  m2 in [0.7, 1.3]  (30% variation)
%   Mobile      : wheel / actuator scale a in [0.8, 1.25]  (25% variation)
%   Quadrotor   : unknown takeoff mass  m in [0.4, 0.8]   (60% variation)
%
% Controller structure for each robot (see Section 7.2 of the lecture):
%
%   Manipulator (robust inverse-dynamics PD+):
%       tau = -Kp*e - Kd*e_dot + G_nominal(q)  -  rho * tanh(s/eps)
%       where s = e_dot + Lambda*e  (Slotine-Li sliding variable)
%       Gravity compensation uses the NOMINAL plant parameters only.
%
%   Mobile (Kanayama + robust velocity term):
%       v     = v_ref cos(e_theta) + k_x * e_x                  (body frame)
%       omega = omega_ref + v_ref ( k_y e_y + k_theta sin(e_theta) )
%       Gains (k_x, k_y, k_theta) are tuned min-max over scale uncertainty.
%
%   Quadrotor (nested PD, nominal-mass feedforward):
%       T    = m_nom * (g - Kp_z * e_z - Kd_z * e_zdot)
%       tau  = J_nom * (-Kp_phi * phi - Kd_phi * phi_dot)
%       Gains (Kp_z, Kd_z, Kp_phi, Kd_phi) tuned min-max over m.
%
% Optimization:  theta* = argmin_theta  max_{p in P}  J(theta, p)
%   - J_i = integral tracking error^2 + beta * integral input^2
%   - min-max over a finite plant set (5 or 4 points); solver: fminsearch
%
% This is an operational proxy for an H-inf / mu-synthesis formulation.

if nargin < 1, saveFig = true; end
results = struct();
fprintf('\n=== ROBUST CONTROL OPTIMIZATION (for robotic systems) ===\n');

%% ===== MANIPULATOR =====
fprintf('[ROB] Manipulator (robust inverse-dynamics PD+):\n');
p_nom = manipulator_params();       % NOMINAL plant used by controller
q_des = [pi/3; -pi/4]; T = 3; dt = 0.002;

% Uncertain plant set — payload mass varied +-30%
mass_mult  = [0.7 0.85 1.0 1.15 1.3];
plant_set = cell(1, numel(mass_mult));
for i = 1:numel(mass_mult)
    pi_ = p_nom; pi_.m2 = mass_mult(i) * p_nom.m2;
    plant_set{i} = pi_;
end

% Decision variables: [Kp, Kd, rho, Lambda]
cost_m = @(z) robust_manip_cost(z, p_nom, plant_set, q_des, T, dt);
z0     = [40, 10, 2, 5];
zOpt   = fminsearch(cost_m, z0, optimset('Display','off','MaxIter',120));
[t1, Q1, U1] = run_robust_manip(z0,   p_nom, plant_set{end}, q_des, T, dt);
[t2, Q2, U2] = run_robust_manip(zOpt, p_nom, plant_set{end}, q_des, T, dt);
results.manipulator.z0 = z0; results.manipulator.zOpt = zOpt;
results.manipulator.J0 = cost_m(z0); results.manipulator.JOpt = cost_m(zOpt);
fprintf('  [Kp Kd rho Lambda]: [%.1f %.1f %.1f %.1f] -> [%.1f %.1f %.1f %.1f]\n',z0,zOpt);
fprintf('  worst-case J: %.4f -> %.4f\n', results.manipulator.J0, results.manipulator.JOpt);

%% ===== MOBILE =====
fprintf('[ROB] Mobile robot (Kanayama tracker, robust to actuator scale):\n');
scales = [0.8 0.9 1.0 1.1 1.25];  T = 8; dt = 0.02;
cost_mob = @(z) robust_mobile_cost(z, scales, T, dt);
z0m = [1, 2, 2];
zOm = fminsearch(cost_mob, z0m, optimset('Display','off','MaxIter',80));
[tm1, Xm1, Um1] = run_robust_mobile(z0m, 1.25, T, dt);
[tm2, Xm2, Um2] = run_robust_mobile(zOm,  1.25, T, dt);
results.mobile.z0 = z0m; results.mobile.zOpt = zOm;
results.mobile.J0 = cost_mob(z0m); results.mobile.JOpt = cost_mob(zOm);
fprintf('  [kx ky ktheta]: [%.2f %.2f %.2f] -> [%.2f %.2f %.2f]\n',z0m,zOm);
fprintf('  worst-case J: %.4f -> %.4f\n', results.mobile.J0, results.mobile.JOpt);

%% ===== QUADROTOR =====
fprintf('[ROB] Quadrotor (nested PD, robust to mass uncertainty):\n');
pq_nom = quadrotor_params();
masses = [0.4 0.5 0.65 0.8];  T = 4; dt = 0.005;
cost_q = @(z) robust_quad_cost(z, pq_nom, masses, T, dt);
z0q = [6, 4, 10, 4];
zOq = fminsearch(cost_q, z0q, optimset('Display','off','MaxIter',120));
[tq1, Xq1, Uq1] = run_robust_quad(z0q, pq_nom, 0.8, T, dt);
[tq2, Xq2, Uq2] = run_robust_quad(zOq, pq_nom, 0.8, T, dt);
results.quad.z0 = z0q; results.quad.zOpt = zOq;
results.quad.J0 = cost_q(z0q); results.quad.JOpt = cost_q(zOq);
fprintf('  [Kpz Kdz Kpphi Kdphi]: [%.1f %.1f %.1f %.1f] -> [%.1f %.1f %.1f %.1f]\n',z0q,zOq);
fprintf('  worst-case J: %.4f -> %.4f\n', results.quad.J0, results.quad.JOpt);

%% Plot
f = figure('Visible','off','Position',[1 1 1500 950]);
subplot(3,3,1); plot(t1,Q1(:,1),'r--','LineWidth',1.4,'DisplayName',sprintf('baseline (K_p=%.0f,K_d=%.0f,\\rho=%.1f,\\Lambda=%.1f)',z0(1),z0(2),z0(3),z0(4))); hold on;
plot(t2,Q2(:,1),'b-','LineWidth',1.4,'DisplayName',sprintf('min-max opt (K_p=%.1f,K_d=%.1f,\\rho=%.1f,\\Lambda=%.1f)',zOpt(1),zOpt(2),zOpt(3),zOpt(4)));
yline(q_des(1),'k:','LineWidth',1.2,'DisplayName','q_{1,des}');
grid on; title('Manipulator q_1 (worst-case payload m_2=1.3)'); ylabel('q_1 [rad]'); xlabel('t [s]');
legend('Location','southeast','FontSize',7);

subplot(3,3,2); plot(t1,Q1(:,2),'r--','LineWidth',1.4,'DisplayName','baseline'); hold on;
plot(t2,Q2(:,2),'b-','LineWidth',1.4,'DisplayName','optimized');
yline(q_des(2),'k:','LineWidth',1.2,'DisplayName','q_{2,des}');
grid on; title('Manipulator q_2'); ylabel('q_2 [rad]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,3); plot(t1,U1(:,1),'r--','LineWidth',1.2,'DisplayName','baseline \tau_1'); hold on;
plot(t2,U2(:,1),'b-','LineWidth',1.2,'DisplayName','optimized \tau_1');
grid on; title('Manipulator torque \tau_1'); ylabel('\tau_1 [N\cdotm]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,4); tref=linspace(0,2*pi,200);
plot(cos(tref),sin(tref),'k:','LineWidth',1.3,'DisplayName','reference circle'); hold on;
plot(Xm1(:,1),Xm1(:,2),'r--','LineWidth',1.4,'DisplayName','baseline robust');
plot(Xm2(:,1),Xm2(:,2),'b-','LineWidth',1.4,'DisplayName','min-max opt');
axis equal; grid on; title('Mobile XY (worst-case wheel scale 1.25)');
xlabel('x [m]'); ylabel('y [m]'); legend('Location','best','FontSize',7);

subplot(3,3,5); plot(tm1,Um1(:,1),'r--','LineWidth',1.2,'DisplayName','baseline v'); hold on;
plot(tm2,Um2(:,1),'b-','LineWidth',1.2,'DisplayName','optimized v');
grid on; title('Mobile v command'); ylabel('v [m/s]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,6); plot(tm1,Um1(:,2),'r--','LineWidth',1.2,'DisplayName','baseline \omega'); hold on;
plot(tm2,Um2(:,2),'b-','LineWidth',1.2,'DisplayName','optimized \omega');
grid on; title('Mobile \omega command'); ylabel('\omega [rad/s]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,7); plot(tq1,Xq1(:,2),'r--','LineWidth',1.4,'DisplayName','baseline'); hold on;
plot(tq2,Xq2(:,2),'b-','LineWidth',1.4,'DisplayName','optimized');
yline(1.0,'k:','LineWidth',1.2,'DisplayName','z_{des}=1m');
grid on; title('Quadrotor altitude (worst-case m=0.8)'); ylabel('z [m]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,8); plot(tq1,Xq1(:,3),'r--','LineWidth',1.4,'DisplayName','baseline'); hold on;
plot(tq2,Xq2(:,3),'b-','LineWidth',1.4,'DisplayName','optimized');
yline(0,'k:','LineWidth',1.0,'DisplayName','\phi_{des}=0');
grid on; title('Quadrotor roll \phi'); ylabel('\phi [rad]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,9); plot(tq1,Uq1(:,1),'r--','LineWidth',1.2,'DisplayName','baseline thrust'); hold on;
plot(tq2,Uq2(:,1),'b-','LineWidth',1.2,'DisplayName','optimized thrust');
yline(0.8*pq_nom.g,'k:','LineWidth',1.0,'DisplayName','hover mg (worst case)');
grid on; title('Quadrotor thrust T'); ylabel('T [N]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

sgtitle({'ROBUST CONTROL FOR ROBOTIC SYSTEMS  —  red dashed = baseline gains,  blue solid = min-max optimized gains,  black dotted = reference',...
    sprintf('Worst-case cost reduction: manipulator %.1f%%, mobile %.1f%%, quadrotor %.1f%%',...
    100*(results.manipulator.J0-results.manipulator.JOpt)/results.manipulator.J0,...
    100*(results.mobile.J0-results.mobile.JOpt)/results.mobile.J0,...
    100*(results.quad.J0-results.quad.JOpt)/results.quad.J0)},...
    'FontSize',10,'FontWeight','bold');
if saveFig
    saveas(f, fullfile(fileparts(mfilename('fullpath')),'figures','robust_results.png'));
end
close(f);
end

% ======================================================================
% Manipulator: robust inverse-dynamics PD+ with sliding-surface-based
% discontinuous compensation.  Feedforward uses NOMINAL parameters ONLY.
% ======================================================================
function [t, Q, U] = run_robust_manip(z, p_nom, p_true, q_des, T, dt)
Kp = z(1); Kd = z(2); rho = z(3); Lambda = z(4);
eps_bl = 0.02;                   % boundary-layer width for tanh
N = round(T/dt); t = (0:N-1)'*dt;
q = [0;0]; dq = [0;0];
Q = zeros(N,2); U = zeros(N,2);
for k = 1:N
    e  = q - q_des;
    de = dq;                     % desired velocity = 0
    s  = de + Lambda*e;          % Slotine-Li sliding variable

    % ----- controller uses NOMINAL plant only -----
    [~, ~, G_nom] = manipulator_dyn(q, dq, p_nom);
    tau_nominal = -Kp*e - Kd*de + G_nom;       % model-based feedback + gravity
    tau_robust  = -rho * tanh(s/eps_bl);       % robust switching (boundary-layered)
    u = tau_nominal + tau_robust;

    % ----- TRUE (uncertain) plant integrated -----
    [M, C, G] = manipulator_dyn(q, dq, p_true);
    ddq = M \ (u - C*dq - G);
    dq = dq + dt*ddq;  q = q + dt*dq;
    Q(k,:) = q';  U(k,:) = u';
end
end
function J = robust_manip_cost(z, p_nom, plant_set, q_des, T, dt)
if any(z<=0) || z(1)>200 || z(2)>50 || z(3)>20 || z(4)>30, J=1e6; return; end
Js = zeros(1,numel(plant_set));
for i = 1:numel(plant_set)
    [~, Q, U] = run_robust_manip(z, p_nom, plant_set{i}, q_des, T, dt);
    e = Q - q_des';
    Js(i) = sum(sum(e.^2))*dt + 0.001*sum(sum(U.^2))*dt;
end
J = max(Js);            % WORST-CASE over the plant set
end

% ======================================================================
% Mobile: Kanayama tracker (Kanayama et al. IEEE ICRA 1990).
% Robust in the sense that (kx, ky, k_theta) are tuned over a range of
% actuator scale factors a.  Plant applies v_true = a * v_command.
% ======================================================================
function [t, X, U] = run_robust_mobile(z, a_true, T, dt)
kx = z(1); ky = z(2); ktheta = z(3);
N = round(T/dt); t = (0:N-1)'*dt;
x = [0;0;0]; X = zeros(N,3); U = zeros(N,2);
for k = 1:N
    tk = t(k);
    % reference pose and velocity on unit circle
    xr   = [cos(0.3*tk); sin(0.3*tk); 0.3*tk + pi/2];
    dxr  = [-0.3*sin(0.3*tk); 0.3*cos(0.3*tk); 0.3];
    vref = norm(dxr(1:2));
    % body-frame error
    R    = [cos(x(3)) sin(x(3)); -sin(x(3)) cos(x(3))];
    eb   = R * (xr(1:2) - x(1:2));
    etheta = wrap(xr(3) - x(3));
    % Kanayama law
    v     = vref*cos(etheta) + kx*eb(1);
    omega = dxr(3) + vref*(ky*eb(2) + ktheta*sin(etheta));
    % true plant: v actually executed = a_true * v_command
    u_plant = [a_true*v; omega];
    x = x + dt * unicycle_dyn(x, u_plant);  x(3) = wrap(x(3));
    X(k,:) = x';  U(k,:) = [v omega];
end
end
function J = robust_mobile_cost(z, scales, T, dt)
if any(z<=0) || any(z>15), J=1e6; return; end
Js = zeros(1, numel(scales));
for i = 1:numel(scales)
    [t, X, U] = run_robust_mobile(z, scales(i), T, dt);
    xr = [cos(0.3*t) sin(0.3*t)];
    e  = X(:,1:2) - xr;
    Js(i) = sum(sum(e.^2))*dt + 0.01*sum(sum(U.^2))*dt;
end
J = max(Js);           % WORST-CASE over actuator scales
end

% ======================================================================
% Quadrotor: nested PD.  Feedforward mass uses NOMINAL value; true plant
% mass varies.  Gains (Kp_z, Kd_z, Kp_phi, Kd_phi) optimized min-max.
% ======================================================================
function [t, X, U] = run_robust_quad(z, p_nom, m_true, T, dt)
Kpz = z(1); Kdz = z(2); Kpphi = z(3); Kdphi = z(4);
p_true = p_nom;  p_true.m = m_true;
N = round(T/dt); t = (0:N-1)'*dt;
x = zeros(6,1); X = zeros(N,6); U = zeros(N,2);
for k = 1:N
    ez  = x(2) - 1.0;   dez  = x(5);
    phi = x(3);         dphi = x(6);
    % Controller uses NOMINAL mass/inertia in feedforward
    T_cmd = p_nom.m * (p_nom.g - Kpz*ez - Kdz*dez);
    T_cmd = max(0.05, T_cmd);                     % thrust >= 0
    tau   = p_nom.J * (-Kpphi*phi - Kdphi*dphi);
    u = [T_cmd; tau];
    % True plant
    x = x + dt * quadrotor_dyn(x, u, p_true);
    X(k,:) = x';  U(k,:) = u';
end
end
function J = robust_quad_cost(z, p_nom, masses, T, dt)
if any(z<=0) || any(z>50), J=1e6; return; end
Js = zeros(1, numel(masses));
for i = 1:numel(masses)
    [~, X, U] = run_robust_quad(z, p_nom, masses(i), T, dt);
    ez = X(:,2) - 1.0;
    Js(i) = sum(ez.^2)*dt + 0.5*sum(X(:,3).^2)*dt + 0.001*sum(sum(U.^2))*dt;
end
J = max(Js);           % WORST-CASE over mass range
end

function y = wrap(a), y = mod(a+pi, 2*pi) - pi; end
