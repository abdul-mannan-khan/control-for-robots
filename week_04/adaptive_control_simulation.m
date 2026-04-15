%% Adaptive Control Simulation for Robotic Systems
%  Week 3: Adaptive Control - M.Sc. Control for Robots
%  Advanced Robotics Programme, AY 2025-26
%
%  This file contains complete simulations for:
%    Section 1: 2-DOF Manipulator with Adaptive Control
%    Section 2: Differential-Drive Mobile Robot with Adaptive Control
%    Section 3: Quadrotor UAV with Adaptive Control
%    Section 4: Comparison - Computed Torque vs Adaptive Control
%
%  Each section: defines parameters, builds regressor, simulates with ode45,
%  tracks reference trajectories, and plots results.

clear all; close all; clc;

%% ========================================================================
%  SECTION 1: 2-DOF MANIPULATOR - ADAPTIVE CONTROL
%  ========================================================================
fprintf('=== Section 1: 2-DOF Manipulator Adaptive Control ===\n');

% True physical parameters
m1 = 1.0;   % Link 1 mass [kg]
m2 = 2.0;   % Link 2 mass [kg]  <-- THIS IS UNKNOWN
l1 = 1.0;   % Link 1 length [m]
l2 = 1.0;   % Link 2 length [m]
g  = 9.81;  % Gravity [m/s^2]

% True composite parameter vector (6 parameters)
theta_true = [m1*l1^2;           % theta1
              m2*l1^2;           % theta2
              m2*l2^2;           % theta3
              m2*l1*l2;          % theta4
              (m1+m2)*g*l1;      % theta5
              m2*g*l2];          % theta6

% Initial parameter estimates (using m2_hat = 1.0 instead of 2.0)
m2_hat0 = 1.0;
theta_hat0 = [m1*l1^2;
              m2_hat0*l1^2;
              m2_hat0*l2^2;
              m2_hat0*l1*l2;
              (m1+m2_hat0)*g*l1;
              m2_hat0*g*l2];

fprintf('True parameters:    [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', theta_true);
fprintf('Initial estimates:  [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', theta_hat0);

% Adaptive control design parameters
Lambda_manip = 10 * eye(2);    % Sliding surface slope
K_D_manip    = 50 * eye(2);    % Damping gain on sliding variable
Gamma_inv_manip = 5 * eye(6);  % Inverse adaptation gain (Gamma^{-1})

% Desired sinusoidal trajectory
qd_fun   = @(t) [0.5*sin(t); 0.5*cos(t)];
dqd_fun  = @(t) [0.5*cos(t); -0.5*sin(t)];
ddqd_fun = @(t) [-0.5*sin(t); -0.5*cos(t)];

% Initial conditions: [q1; q2; dq1; dq2; theta_hat(1:6)]
x0_manip = [0; 0; 0; 0; theta_hat0];
tspan_manip = 0:0.005:20;

% Simulate
opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
[t_m, X_m] = ode45(@(t,x) manipulator_adaptive_ode(t, x, ...
    m1, m2, l1, l2, g, Lambda_manip, K_D_manip, Gamma_inv_manip, ...
    qd_fun, dqd_fun, ddqd_fun), tspan_manip, x0_manip, opts);

% Extract results
q_m      = X_m(:, 1:2);
dq_m     = X_m(:, 3:4);
theta_hat_m = X_m(:, 5:10);

% Compute desired trajectory for plotting
qd_plot = zeros(length(t_m), 2);
for i = 1:length(t_m)
    qd_plot(i,:) = qd_fun(t_m(i))';
end
e_m = qd_plot - q_m;

% --- Plot manipulator results ---
figure('Name', 'Manipulator Adaptive Control', 'Position', [50, 50, 1400, 900]);

% Joint tracking
subplot(3,3,1);
plot(t_m, qd_plot(:,1), 'b--', 'LineWidth', 1.5); hold on;
plot(t_m, q_m(:,1), 'r-', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('q_1 [rad]');
title('Joint 1 Tracking'); legend('Desired', 'Actual');
grid on;

subplot(3,3,2);
plot(t_m, qd_plot(:,2), 'b--', 'LineWidth', 1.5); hold on;
plot(t_m, q_m(:,2), 'r-', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('q_2 [rad]');
title('Joint 2 Tracking'); legend('Desired', 'Actual');
grid on;

% Tracking errors
subplot(3,3,3);
plot(t_m, e_m(:,1), 'r-', 'LineWidth', 1.2); hold on;
plot(t_m, e_m(:,2), 'b-', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Error [rad]');
title('Tracking Errors'); legend('e_1', 'e_2');
grid on;

% Parameter convergence (6 subplots)
param_names = {'\theta_1 = m_1 l_1^2', '\theta_2 = m_2 l_1^2', ...
               '\theta_3 = m_2 l_2^2', '\theta_4 = m_2 l_1 l_2', ...
               '\theta_5 = (m_1+m_2)gl_1', '\theta_6 = m_2 g l_2'};
for i = 1:6
    subplot(3,3,3+i);
    plot(t_m, theta_hat_m(:,i), 'r-', 'LineWidth', 1.5); hold on;
    yline(theta_true(i), 'b--', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel(['\theta_{' num2str(i) '}']);
    title(['Parameter ' num2str(i) ': ' param_names{i}]);
    legend('Estimate', 'True');
    grid on;
end
sgtitle('Section 1: 2-DOF Manipulator - Adaptive Control', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('Manipulator simulation complete.\n');
fprintf('Final tracking error norm: %.6f rad\n', norm(e_m(end,:)));
fprintf('Final parameter error norm: %.4f\n\n', norm(theta_hat_m(end,:)' - theta_true));

%% ========================================================================
%  SECTION 2: MOBILE ROBOT - ADAPTIVE CONTROL
%  ========================================================================
fprintf('=== Section 2: Mobile Robot Adaptive Control ===\n');

% True parameters
r_true = 0.05;     % Wheel radius [m]
b_true = 0.30;     % Track width [m]
theta_mob_true = [r_true; r_true/b_true];

% Initial estimates (20% error on radius, 20% error on track width)
r_hat0  = 0.04;    % 20% underestimate
b_hat0  = 0.36;    % 20% overestimate
theta_mob_hat0 = [r_hat0; r_hat0/b_hat0];

fprintf('True parameters:    r = %.4f, r/b = %.4f\n', theta_mob_true);
fprintf('Initial estimates:  r = %.4f, r/b = %.4f\n', theta_mob_hat0);

% Tracking controller gains
k1_mob = 3.0;
k2_mob = 5.0;
k3_mob = 3.0;
Gamma_mob = diag([0.01, 0.05]);  % Adaptation gain

% Circular reference trajectory
R_circ    = 2.0;     % Circle radius [m]
w_circ    = 0.5;     % Angular frequency [rad/s]
xd_mob    = @(t) R_circ * cos(w_circ * t);
yd_mob    = @(t) R_circ * sin(w_circ * t);
dxd_mob   = @(t) -R_circ * w_circ * sin(w_circ * t);
dyd_mob   = @(t)  R_circ * w_circ * cos(w_circ * t);
phid_mob  = @(t) w_circ * t + pi/2;
vd_mob    = @(t) R_circ * w_circ;
wd_mob    = @(t) w_circ;

% Initial state: [x; y; phi; theta_hat(1); theta_hat(2)]
x0_mob = [R_circ; 0; pi/2; theta_mob_hat0];
tspan_mob = 0:0.01:30;

[t_mob, X_mob] = ode45(@(t,x) mobile_adaptive_ode(t, x, ...
    r_true, b_true, k1_mob, k2_mob, k3_mob, Gamma_mob, ...
    xd_mob, yd_mob, dxd_mob, dyd_mob, phid_mob, vd_mob, wd_mob), ...
    tspan_mob, x0_mob, opts);

% Extract results
px_mob = X_mob(:,1);
py_mob = X_mob(:,2);
phi_mob = X_mob(:,3);
theta_hat_mob = X_mob(:,4:5);

% Desired trajectory for plotting
xd_plot = zeros(length(t_mob),1);
yd_plot = zeros(length(t_mob),1);
for i = 1:length(t_mob)
    xd_plot(i) = xd_mob(t_mob(i));
    yd_plot(i) = yd_mob(t_mob(i));
end
pos_err_mob = sqrt((xd_plot - px_mob).^2 + (yd_plot - py_mob).^2);

% --- Plot mobile robot results ---
figure('Name', 'Mobile Robot Adaptive Control', 'Position', [100, 100, 1200, 800]);

subplot(2,3,1);
plot(xd_plot, yd_plot, 'b--', 'LineWidth', 1.5); hold on;
plot(px_mob, py_mob, 'r-', 'LineWidth', 1.2);
plot(px_mob(1), py_mob(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
xlabel('x [m]'); ylabel('y [m]');
title('Path Tracking'); legend('Desired', 'Actual', 'Start');
axis equal; grid on;

subplot(2,3,2);
plot(t_mob, xd_plot, 'b--', 'LineWidth', 1.5); hold on;
plot(t_mob, px_mob, 'r-', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('x [m]');
title('X Position'); legend('Desired', 'Actual');
grid on;

subplot(2,3,3);
plot(t_mob, yd_plot, 'b--', 'LineWidth', 1.5); hold on;
plot(t_mob, py_mob, 'r-', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('y [m]');
title('Y Position'); legend('Desired', 'Actual');
grid on;

subplot(2,3,4);
plot(t_mob, pos_err_mob, 'r-', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Position Error [m]');
title('Position Error Norm'); grid on;

subplot(2,3,5);
plot(t_mob, theta_hat_mob(:,1), 'r-', 'LineWidth', 1.5); hold on;
yline(theta_mob_true(1), 'b--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('r [m]');
title('Parameter: Wheel Radius r'); legend('Estimate', 'True');
grid on;

subplot(2,3,6);
plot(t_mob, theta_hat_mob(:,2), 'r-', 'LineWidth', 1.5); hold on;
yline(theta_mob_true(2), 'b--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('r/b');
title('Parameter: r/b Ratio'); legend('Estimate', 'True');
grid on;

sgtitle('Section 2: Mobile Robot - Adaptive Control', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('Mobile robot simulation complete.\n');
fprintf('Final position error: %.6f m\n', pos_err_mob(end));
fprintf('Final parameter error norm: %.6f\n\n', norm(theta_hat_mob(end,:)' - theta_mob_true));

%% ========================================================================
%  SECTION 3: QUADROTOR UAV - ADAPTIVE CONTROL
%  ========================================================================
fprintf('=== Section 3: Quadrotor Adaptive Control ===\n');

% True parameters
m_true  = 1.5;     % Mass [kg]
g_val   = 9.81;    % Gravity [m/s^2]
Ixx_true = 0.03;   % Roll inertia [kg.m^2]
Iyy_true = 0.03;   % Pitch inertia [kg.m^2]
Izz_true = 0.06;   % Yaw inertia [kg.m^2]

theta_p_true = [m_true; m_true * g_val];           % Position parameters
theta_a_true = [Ixx_true; Iyy_true; Izz_true];     % Attitude parameters

% Initial estimates (mass underestimated by 33%, inertias overestimated)
m_hat0  = 1.0;
theta_p_hat0 = [m_hat0; m_hat0 * g_val];
theta_a_hat0 = [0.05; 0.05; 0.10];

fprintf('True position params:  m=%.2f, mg=%.2f\n', theta_p_true);
fprintf('Initial pos estimates: m=%.2f, mg=%.2f\n', theta_p_hat0);
fprintf('True attitude params:  [%.3f, %.3f, %.3f]\n', theta_a_true);
fprintf('Initial att estimates: [%.3f, %.3f, %.3f]\n', theta_a_hat0);

% Adaptive control gains
Lambda_p_quad = 2.0 * eye(3);    % Position sliding surface
K_Dp_quad     = 15.0 * eye(3);   % Position damping
Gamma_p_inv   = 0.5 * eye(2);    % Position adaptation

Lambda_a_quad = 5.0 * eye(3);    % Attitude sliding surface
K_Da_quad     = 8.0 * eye(3);    % Attitude damping
Gamma_a_inv   = 0.1 * eye(3);    % Attitude adaptation

% Helical reference trajectory
pd_fun   = @(t) [2*cos(0.5*t); 2*sin(0.5*t); 0.2*t];
dpd_fun  = @(t) [-sin(0.5*t); cos(0.5*t); 0.2];
ddpd_fun = @(t) [-0.5*cos(0.5*t); -0.5*sin(0.5*t); 0];
psid_fun   = @(t) 0;
dpsid_fun  = @(t) 0;
ddpsid_fun = @(t) 0;

% State vector (17 states):
%   x(1:3)   = position [x, y, z]
%   x(4:6)   = velocity [dx, dy, dz]
%   x(7:9)   = Euler angles [phi, theta, psi]
%   x(10:12) = angular rates [dphi, dtheta, dpsi]
%   x(13:14) = theta_p_hat [m_hat, mg_hat]
%   x(15:17) = theta_a_hat [Ixx_hat, Iyy_hat, Izz_hat]
x0_quad = zeros(17, 1);
% Start at the reference trajectory (avoids a large initial transient
% that would demand extreme attitudes and violate linearisation assumptions)
x0_quad(1:3) = pd_fun(0);       % position = [2; 0; 0]
x0_quad(4:6) = dpd_fun(0);      % velocity = [0; 1; 0.2]
x0_quad(13:14) = theta_p_hat0;
x0_quad(15:17) = theta_a_hat0;
tspan_quad = 0:0.005:30;

[t_q, X_q] = ode45(@(t,x) quadrotor_adaptive_ode(t, x, ...
    m_true, g_val, Ixx_true, Iyy_true, Izz_true, ...
    Lambda_p_quad, K_Dp_quad, Gamma_p_inv, ...
    Lambda_a_quad, K_Da_quad, Gamma_a_inv, ...
    pd_fun, dpd_fun, ddpd_fun, psid_fun, dpsid_fun, ddpsid_fun), ...
    tspan_quad, x0_quad, opts);

% Extract results
pos_q     = X_q(:, 1:3);
vel_q     = X_q(:, 4:6);
euler_q   = X_q(:, 7:9);
omega_q   = X_q(:, 10:12);
theta_p_hat_q = X_q(:, 13:14);
theta_a_hat_q = X_q(:, 15:17);

% Desired trajectory for plotting
pd_plot = zeros(length(t_q), 3);
for i = 1:length(t_q)
    pd_plot(i,:) = pd_fun(t_q(i))';
end
pos_err_q = pos_q - pd_plot;

% --- Plot quadrotor results ---
figure('Name', 'Quadrotor Adaptive Control', 'Position', [150, 50, 1400, 1000]);

% 3D trajectory
subplot(2,3,1);
plot3(pd_plot(:,1), pd_plot(:,2), pd_plot(:,3), 'b--', 'LineWidth', 1.5); hold on;
plot3(pos_q(:,1), pos_q(:,2), pos_q(:,3), 'r-', 'LineWidth', 1.2);
plot3(pos_q(1,1), pos_q(1,2), pos_q(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('3D Trajectory'); legend('Desired', 'Actual', 'Start');
grid on; view(30, 25);

% Position tracking
subplot(2,3,2);
plot(t_q, pd_plot(:,1), 'b--', t_q, pos_q(:,1), 'r-', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('x [m]');
title('X Position'); legend('Desired', 'Actual'); grid on;

subplot(2,3,3);
plot(t_q, pd_plot(:,3), 'b--', t_q, pos_q(:,3), 'r-', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('z [m]');
title('Z Position (Altitude)'); legend('Desired', 'Actual'); grid on;

% Position errors
subplot(2,3,4);
plot(t_q, pos_err_q(:,1), 'r-', t_q, pos_err_q(:,2), 'g-', ...
     t_q, pos_err_q(:,3), 'b-', 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('Error [m]');
title('Position Errors'); legend('e_x', 'e_y', 'e_z'); grid on;

% Position parameter convergence
subplot(2,3,5);
plot(t_q, theta_p_hat_q(:,1), 'r-', 'LineWidth', 1.5); hold on;
yline(theta_p_true(1), 'b--', 'LineWidth', 1.5);
plot(t_q, theta_p_hat_q(:,2)/g_val, 'm-', 'LineWidth', 1.5);
yline(theta_p_true(1), 'c--', 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('Mass [kg]');
title('Mass Estimate Convergence');
legend('\hat{m}', 'm_{true}', '\hat{m}g / g', 'Location', 'best');
grid on;

% Attitude parameter convergence
subplot(2,3,6);
plot(t_q, theta_a_hat_q(:,1), 'r-', 'LineWidth', 1.5); hold on;
plot(t_q, theta_a_hat_q(:,2), 'g-', 'LineWidth', 1.5);
plot(t_q, theta_a_hat_q(:,3), 'b-', 'LineWidth', 1.5);
yline(Ixx_true, 'r--', 'LineWidth', 1.0);
yline(Iyy_true, 'g--', 'LineWidth', 1.0);
yline(Izz_true, 'b--', 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('Inertia [kg.m^2]');
title('Inertia Estimates');
legend('\hat{I}_{xx}', '\hat{I}_{yy}', '\hat{I}_{zz}', ...
       'I_{xx,true}', 'I_{yy,true}', 'I_{zz,true}');
grid on;

sgtitle('Section 3: Quadrotor - Adaptive Control (Helical Trajectory)', ...
    'FontSize', 14, 'FontWeight', 'bold');

fprintf('Quadrotor simulation complete.\n');
fprintf('Final position error norm: %.6f m\n', norm(pos_err_q(end,:)));
fprintf('Final mass estimate: %.4f (true: %.4f)\n', theta_p_hat_q(end,1), m_true);
fprintf('Final inertia estimates: [%.4f, %.4f, %.4f]\n', theta_a_hat_q(end,:));
fprintf('True inertias:          [%.4f, %.4f, %.4f]\n\n', Ixx_true, Iyy_true, Izz_true);

%% ========================================================================
%  SECTION 4: COMPARISON - COMPUTED TORQUE vs ADAPTIVE CONTROL
%  ========================================================================
fprintf('=== Section 4: Computed Torque vs Adaptive Control Comparison ===\n');

% --- Computed Torque with WRONG parameters ---
% State: [q1; q2; dq1; dq2] (no parameter adaptation)
x0_ct = [0; 0; 0; 0];
Kp_ct = 100 * eye(2);
Kv_ct = 20 * eye(2);

[t_ct, X_ct] = ode45(@(t,x) manipulator_ct_wrong_ode(t, x, ...
    m1, m2, l1, l2, g, m2_hat0, Kp_ct, Kv_ct, ...
    qd_fun, dqd_fun, ddqd_fun), tspan_manip, x0_ct, opts);

q_ct = X_ct(:, 1:2);
e_ct = qd_plot - q_ct;

% --- Adaptive Control (already computed in Section 1) ---
% q_m and e_m from Section 1

% --- Plot comparison ---
figure('Name', 'Computed Torque vs Adaptive', 'Position', [200, 100, 1200, 600]);

subplot(2,3,1);
plot(t_ct, qd_plot(:,1), 'b--', 'LineWidth', 1.5); hold on;
plot(t_ct, q_ct(:,1), 'r-', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('q_1 [rad]');
title('CT: Joint 1 Tracking'); legend('Desired', 'CT (wrong params)');
grid on;

subplot(2,3,2);
plot(t_m, qd_plot(:,1), 'b--', 'LineWidth', 1.5); hold on;
plot(t_m, q_m(:,1), 'r-', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('q_1 [rad]');
title('Adaptive: Joint 1 Tracking'); legend('Desired', 'Adaptive');
grid on;

subplot(2,3,3);
plot(t_ct, sqrt(e_ct(:,1).^2 + e_ct(:,2).^2), 'r-', 'LineWidth', 1.2); hold on;
plot(t_m, sqrt(e_m(:,1).^2 + e_m(:,2).^2), 'b-', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('||e|| [rad]');
title('Error Norm Comparison');
legend('CT (wrong params)', 'Adaptive');
grid on;

subplot(2,3,4);
plot(t_ct, e_ct(:,1), 'r-', t_ct, e_ct(:,2), 'b-', 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('Error [rad]');
title('CT Tracking Errors'); legend('e_1', 'e_2'); grid on;

subplot(2,3,5);
plot(t_m, e_m(:,1), 'r-', t_m, e_m(:,2), 'b-', 'LineWidth', 1.0);
xlabel('Time [s]'); ylabel('Error [rad]');
title('Adaptive Tracking Errors'); legend('e_1', 'e_2'); grid on;

subplot(2,3,6);
% Final steady-state error comparison (last 5 seconds)
idx_ss_ct = t_ct >= 15;
idx_ss_ad = t_m >= 15;
ss_err_ct = mean(sqrt(e_ct(idx_ss_ct,1).^2 + e_ct(idx_ss_ct,2).^2));
ss_err_ad = mean(sqrt(e_m(idx_ss_ad,1).^2 + e_m(idx_ss_ad,2).^2));
bar([ss_err_ct, ss_err_ad]);
set(gca, 'XTickLabel', {'CT (wrong)', 'Adaptive'});
ylabel('Mean SS Error [rad]');
title('Steady-State Error Comparison');
grid on;
text(1, ss_err_ct*1.1, sprintf('%.4f', ss_err_ct), 'HorizontalAlignment', 'center');
text(2, ss_err_ad*1.1, sprintf('%.6f', ss_err_ad), 'HorizontalAlignment', 'center');

sgtitle('Section 4: Computed Torque (Wrong Params) vs Adaptive Control', ...
    'FontSize', 14, 'FontWeight', 'bold');

fprintf('Computed torque steady-state error: %.6f rad\n', ss_err_ct);
fprintf('Adaptive control steady-state error: %.6f rad\n', ss_err_ad);
fprintf('Improvement factor: %.1fx\n\n', ss_err_ct / max(ss_err_ad, 1e-12));

fprintf('=== All simulations complete. ===\n');

%% ========================================================================
%  ODE FUNCTIONS
%  ========================================================================

%% --- 2-DOF Manipulator: Adaptive Control ODE ---
function dx = manipulator_adaptive_ode(t, x, m1, m2, l1, l2, g, ...
                                        Lambda, K_D, Gamma_inv, ...
                                        qd_fun, dqd_fun, ddqd_fun)
    % State extraction
    q  = x(1:2);
    dq = x(3:4);
    theta_hat = x(5:10);

    % Desired trajectory
    qd   = qd_fun(t);
    dqd  = dqd_fun(t);
    ddqd = ddqd_fun(t);

    % Errors
    e   = qd - q;
    de  = dqd - dq;

    % Reference signals
    dqr  = dqd + Lambda * e;
    ddqr = ddqd + Lambda * de;

    % Sliding variable
    s = de + Lambda * e;  % equivalently s = dqr - dq

    % Build regressor Y(q, dq, dqr, ddqr) [2x6]
    c2 = cos(q(2));
    s2 = sin(q(2));
    s1 = sin(q(1));
    s12 = sin(q(1) + q(2));

    Y = zeros(2, 6);
    % Row 1 (Joint 1 equation)
    Y(1,1) = ddqr(1);
    Y(1,2) = ddqr(1);
    Y(1,3) = ddqr(1) + ddqr(2);
    Y(1,4) = 2*c2*ddqr(1) + c2*ddqr(2) ...
             - s2*dq(2)*dqr(1) - s2*(dq(1)+dq(2))*dqr(2);
    Y(1,5) = s1;
    Y(1,6) = s12;

    % Row 2 (Joint 2 equation)
    Y(2,1) = 0;
    Y(2,2) = 0;
    Y(2,3) = ddqr(1) + ddqr(2);
    Y(2,4) = c2*ddqr(1) + s2*dq(1)*dqr(1);
    Y(2,5) = 0;
    Y(2,6) = s12;

    % Adaptive control torque
    tau = Y * theta_hat + K_D * s;

    % Parameter update law: d(theta_hat)/dt = Gamma^{-1} Y^T s
    dtheta_hat = Gamma_inv * (Y' * s);

    % True dynamics matrices (for plant simulation)
    M = [m1*l1^2 + m2*(l1^2 + l2^2 + 2*l1*l2*c2), m2*(l2^2 + l1*l2*c2);
         m2*(l2^2 + l1*l2*c2),                      m2*l2^2];

    h = m2*l1*l2*s2;
    C = [-h*dq(2),  -h*(dq(1)+dq(2));
          h*dq(1),   0];

    G = [(m1+m2)*g*l1*s1 + m2*g*l2*s12;
          m2*g*l2*s12];

    % Equations of motion: M*ddq = tau - C*dq - G
    ddq = M \ (tau - C*dq - G);

    dx = [dq; ddq; dtheta_hat];
end

%% --- 2-DOF Manipulator: Computed Torque with WRONG parameters ODE ---
function dx = manipulator_ct_wrong_ode(t, x, m1, m2, l1, l2, g, ...
                                        m2_hat, Kp, Kv, ...
                                        qd_fun, dqd_fun, ddqd_fun)
    % State extraction
    q  = x(1:2);
    dq = x(3:4);

    % Desired trajectory
    qd   = qd_fun(t);
    dqd  = dqd_fun(t);
    ddqd = ddqd_fun(t);

    % Errors
    e  = qd - q;
    de = dqd - dq;

    c2 = cos(q(2));
    s2 = sin(q(2));
    s1 = sin(q(1));
    s12 = sin(q(1) + q(2));

    % Estimated dynamics (using WRONG m2_hat)
    M_hat = [m1*l1^2 + m2_hat*(l1^2+l2^2+2*l1*l2*c2), m2_hat*(l2^2+l1*l2*c2);
             m2_hat*(l2^2+l1*l2*c2),                    m2_hat*l2^2];

    h_hat = m2_hat*l1*l2*s2;
    C_hat = [-h_hat*dq(2),  -h_hat*(dq(1)+dq(2));
              h_hat*dq(1),   0];

    G_hat = [(m1+m2_hat)*g*l1*s1 + m2_hat*g*l2*s12;
              m2_hat*g*l2*s12];

    % Computed torque control law with wrong parameters
    v = ddqd + Kv*de + Kp*e;
    tau = M_hat * v + C_hat * dq + G_hat;

    % True dynamics
    M = [m1*l1^2 + m2*(l1^2+l2^2+2*l1*l2*c2), m2*(l2^2+l1*l2*c2);
         m2*(l2^2+l1*l2*c2),                    m2*l2^2];

    h = m2*l1*l2*s2;
    C = [-h*dq(2),  -h*(dq(1)+dq(2));
          h*dq(1),   0];

    G = [(m1+m2)*g*l1*s1 + m2*g*l2*s12;
          m2*g*l2*s12];

    ddq = M \ (tau - C*dq - G);

    dx = [dq; ddq];
end

%% --- Mobile Robot: Adaptive Control ODE ---
function dx = mobile_adaptive_ode(t, x, r_true, b_true, ...
                                   k1, k2, k3, Gamma_m, ...
                                   xd_fun, yd_fun, dxd_fun, dyd_fun, ...
                                   phid_fun, vd_fun, wd_fun)
    % State extraction
    px  = x(1);
    py  = x(2);
    phi = x(3);
    theta_hat = x(4:5);  % [r_hat; (r/b)_hat]

    % Desired values
    xd_t   = xd_fun(t);
    yd_t   = yd_fun(t);
    phid_t = phid_fun(t);
    vd_t   = vd_fun(t);
    wd_t   = wd_fun(t);

    % Body-frame position errors
    cphi = cos(phi);
    sphi = sin(phi);
    ex =  cphi * (xd_t - px) + sphi * (yd_t - py);
    ey = -sphi * (xd_t - px) + cphi * (yd_t - py);
    ephi = phid_t - phi;
    ephi = atan2(sin(ephi), cos(ephi));  % wrap to [-pi, pi]

    % Tracking control law (velocity commands)
    v_c = vd_t * cos(ephi) + k1 * ex;
    w_c = wd_t + vd_t * (k2 * ey + k3 * sin(ephi));

    % Inverse kinematics using estimated parameters
    r_hat  = max(theta_hat(1), 0.001);   % prevent division by zero
    rb_hat = max(theta_hat(2), 0.001);   % r/b estimate

    omega_R = v_c / r_hat + w_c / (2 * rb_hat);
    omega_L = v_c / r_hat - w_c / (2 * rb_hat);

    % True velocities (actual plant)
    v_actual = (r_true / 2) * (omega_R + omega_L);
    w_actual = (r_true / b_true) * (omega_R - omega_L);

    % Kinematics (true plant)
    dpx  = v_actual * cos(phi);
    dpy  = v_actual * sin(phi);
    dphi = w_actual;

    % Regressor for adaptation
    Y_m = [0.5*(omega_R + omega_L), 0;
           0,                        omega_R - omega_L];

    % Adaptation error signal
    e_adapt = [ex; ephi];

    % Parameter update law
    dtheta = Gamma_m * (Y_m' * e_adapt);

    dx = [dpx; dpy; dphi; dtheta];
end

%% --- Quadrotor: Adaptive Control ODE ---
function dx = quadrotor_adaptive_ode(t, x, m_true, g_val, ...
                                      Ixx_true, Iyy_true, Izz_true, ...
                                      Lambda_p, K_Dp, Gamma_p_inv, ...
                                      Lambda_a, K_Da, Gamma_a_inv, ...
                                      pd_fun, dpd_fun, ddpd_fun, ...
                                      psid_fun, dpsid_fun, ddpsid_fun)
    % State extraction
    pos   = x(1:3);     % [x; y; z]
    vel   = x(4:6);     % [dx; dy; dz]
    euler = x(7:9);     % [phi; theta; psi]
    omega = x(10:12);   % [dphi; dtheta; dpsi]
    theta_p_hat = x(13:14);  % [m_hat; mg_hat]
    theta_a_hat = x(15:17);  % [Ixx_hat; Iyy_hat; Izz_hat]

    phi_val   = euler(1);
    theta_val = euler(2);
    psi_val   = euler(3);

    % Desired trajectory
    pd   = pd_fun(t);
    dpd  = dpd_fun(t);
    ddpd = ddpd_fun(t);
    psid   = psid_fun(t);
    dpsid  = dpsid_fun(t);
    ddpsid = ddpsid_fun(t);

    % ---- POSITION LOOP (Adaptive) ----
    % Position error
    e_p  = pd - pos;
    de_p = dpd - vel;

    % Reference acceleration and sliding variable
    ddp_r = ddpd + Lambda_p * de_p;
    s_p   = de_p + Lambda_p * e_p;

    % Position regressor: F_des = Y_p * theta_p_hat + K_Dp * s_p
    % theta_p_hat = [m_hat; mg_hat]
    Y_p = [ddp_r(1), 0;
           ddp_r(2), 0;
           ddp_r(3), 1];

    % Desired force vector
    F_des = Y_p * theta_p_hat + K_Dp * s_p;

    % Total thrust magnitude
    T = norm(F_des);
    T = max(T, 0.1);  % prevent zero thrust

    % Extract desired roll and pitch from desired force
    % F_des = R * [0; 0; T], so we need to find R such that R*e3 = F_des/T
    F_unit = F_des / T;

    % Desired angles (small angle extraction)
    phi_d   = (F_des(1)*sin(psid) - F_des(2)*cos(psid)) / T;
    theta_d = (F_des(1)*cos(psid) + F_des(2)*sin(psid)) / T;

    % Clamp desired angles for safety
    phi_d   = max(min(phi_d, 0.5), -0.5);
    theta_d = max(min(theta_d, 0.5), -0.5);

    % Position parameter update
    dtheta_p = Gamma_p_inv * (Y_p' * s_p);

    % ---- ATTITUDE LOOP (Adaptive) ----
    % Desired Euler angles
    eta_d = [phi_d; theta_d; psid];

    % For desired angular velocities and accelerations, use numerical
    % approximation (desired angles are computed from force, so we use
    % simple proportional approximation)
    deta_d  = [0; 0; dpsid];
    ddeta_d = [0; 0; ddpsid];

    % Attitude error
    e_a  = eta_d - euler;
    de_a = deta_d - omega;

    % Wrap heading error
    e_a(3) = atan2(sin(e_a(3)), cos(e_a(3)));

    % Reference angular acceleration and sliding variable
    ddeta_r = ddeta_d + Lambda_a * de_a;
    s_a     = de_a + Lambda_a * e_a;

    % Attitude regressor: tau_a = Y_a * theta_a_hat + K_Da * s_a
    Y_a = diag(ddeta_r);

    % Adaptive torques
    tau_a = Y_a * theta_a_hat + K_Da * s_a;

    % Attitude parameter update
    dtheta_a = Gamma_a_inv * (Y_a' * s_a);

    % ---- TRUE PLANT DYNAMICS ----
    % Rotation matrix (ZYX convention)
    cphi = cos(phi_val); sphi = sin(phi_val);
    cth  = cos(theta_val); sth = sin(theta_val);
    cpsi = cos(psi_val); spsi = sin(psi_val);

    R = [cpsi*cth,  cpsi*sth*sphi - spsi*cphi,  cpsi*sth*cphi + spsi*sphi;
         spsi*cth,  spsi*sth*sphi + cpsi*cphi,  spsi*sth*cphi - cpsi*sphi;
         -sth,      cth*sphi,                    cth*cphi];

    % Translational dynamics: m*ddp = [0;0;-mg] + R*[0;0;T]
    gravity_force = [0; 0; -m_true * g_val];
    thrust_body   = [0; 0; T];
    thrust_world  = R * thrust_body;

    ddpos = (gravity_force + thrust_world) / m_true;

    % Rotational dynamics (simplified, decoupled)
    ddphi   = tau_a(1) / Ixx_true;
    ddtheta = tau_a(2) / Iyy_true;
    ddpsi   = tau_a(3) / Izz_true;

    ddeuler = [ddphi; ddtheta; ddpsi];

    % State derivatives
    dx = zeros(17, 1);
    dx(1:3)   = vel;
    dx(4:6)   = ddpos;
    dx(7:9)   = omega;
    dx(10:12) = ddeuler;
    dx(13:14) = dtheta_p;
    dx(15:17) = dtheta_a;
end
