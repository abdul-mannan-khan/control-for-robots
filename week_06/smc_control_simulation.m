%% ========================================================================
%  SMC Control Simulation — Week 6: Sliding Mode Control
%  M.Sc. Control for Robots, Advanced Robotics Programme, AY 2025-26
%  ========================================================================
%  This script implements Sliding Mode Control (SMC) for three robotic
%  platforms:
%    1. 2-DOF Robot Manipulator
%    2. Differential-Drive Mobile Robot (Unicycle)
%    3. Quadrotor Drone
%
%  Features:
%    - Standard SMC with sign(s)
%    - Boundary layer with sat(s/phi)
%    - Super-twisting algorithm
%    - Disturbance injection
%    - Comparison plots across controllers
%
%  Note: Fixed-step RK4 integration is used instead of ode45 because
%        variable-step solvers struggle with the sign() discontinuity
%        in SMC, causing extremely slow execution. Fixed-step RK4 is
%        the standard approach for SMC simulation.
%  ========================================================================
clear; clc; close all;

fprintf('========================================\n');
fprintf(' Week 6: Sliding Mode Control\n');
fprintf(' Control for Robots — Advanced Robotics Programme 2025-26\n');
fprintf('========================================\n\n');

%% ========================================================================
%  PART 1: 2-DOF ROBOT MANIPULATOR SMC
%  ========================================================================
fprintf('--- PART 1: 2-DOF Manipulator SMC ---\n');

% Physical parameters
m1 = 1.0;  m2 = 1.0;   % link masses (kg)
l1 = 1.0;  l2 = 1.0;   % link lengths (m)
g  = 9.81;              % gravity (m/s^2)

% SMC parameters
Lambda_manip = diag([10, 10]);    % sliding surface gain
K_sign       = diag([20, 20]);    % switching gain (sign mode)
K_sat        = diag([20, 20]);    % switching gain (sat mode)
k_reach      = diag([5, 5]);     % proportional reaching gain

% Boundary layer widths to test
phi_values = [0.01, 0.05, 0.1, 1.0];

% Super-twisting gains
k1_st = diag([15, 15]);   % |s|^{1/2} gain
k2_st = diag([10, 10]);   % integral gain

% Simulation parameters (fixed-step RK4)
dt_manip = 0.001;
t_end_manip = 10;

% Initial condition: [q1, q2, dq1, dq2]
x0_manip = [0.5; -0.5; 0; 0];

% Desired trajectory functions (joint space)
qd_fun   = @(t) [sin(t); cos(t)];
qd_d_fun = @(t) [cos(t); -sin(t)];
qd_dd_fun= @(t) [-sin(t); -cos(t)];

% Disturbance function (zero for now; will be added in Activity 2)
dist_fun_zero = @(t) [0; 0];
dist_fun_large = @(t) 10*sin(5*t)*[1; 1];

% --- Run 1: SMC with sign(s) ---
fprintf('  Running SMC with sign(s)...\n');
[T_sign, X_sign, tau_sign, s_sign] = run_manipulator_smc( ...
    dt_manip, t_end_manip, x0_manip, m1, m2, l1, l2, g, ...
    Lambda_manip, K_sign, k_reach, 0, 'sign', [], [], ...
    qd_fun, qd_d_fun, qd_dd_fun, dist_fun_zero);
fprintf('    Done. Final error: %.6f rad\n', norm(X_sign(end,1:2) - qd_fun(T_sign(end))'));

% --- Run 2: SMC with sat(s/phi) for various phi ---
results_sat = cell(length(phi_values), 1);
for idx = 1:length(phi_values)
    phi_val = phi_values(idx);
    fprintf('  Running SMC with sat(s/phi), phi = %.2f...\n', phi_val);
    [T_sat, X_sat, tau_sat, s_sat] = run_manipulator_smc( ...
        dt_manip, t_end_manip, x0_manip, m1, m2, l1, l2, g, ...
        Lambda_manip, K_sat, k_reach, phi_val, 'sat', [], [], ...
        qd_fun, qd_d_fun, qd_dd_fun, dist_fun_zero);
    results_sat{idx} = struct('T', T_sat, 'X', X_sat, 'tau', tau_sat, ...
        's', s_sat, 'phi', phi_val);
    fprintf('    Done. Final error: %.6f rad\n', norm(X_sat(end,1:2) - qd_fun(T_sat(end))'));
end

% --- Run 3: Super-twisting SMC ---
fprintf('  Running Super-Twisting SMC...\n');
x0_st = [x0_manip; 0; 0];  % extra states for integral term
[T_st, X_st, tau_st, s_st] = run_manipulator_smc( ...
    dt_manip, t_end_manip, x0_st, m1, m2, l1, l2, g, ...
    Lambda_manip, K_sign, k_reach, 0, 'super_twisting', k1_st, k2_st, ...
    qd_fun, qd_d_fun, qd_dd_fun, dist_fun_zero);
fprintf('    Done. Final error: %.6f rad\n', norm(X_st(end,1:2) - qd_fun(T_st(end))'));

% ---- PLOTTING: Manipulator Results ----
figure('Name', 'Manipulator SMC — Joint Tracking', 'Position', [50 500 1200 600]);

% Joint 1 tracking
subplot(2,2,1);
plot(T_sign, X_sign(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(T_sign, sin(T_sign), 'r--', 'LineWidth', 1.5);
for idx = 1:length(phi_values)
    R = results_sat{idx};
    plot(R.T, R.X(:,1), 'LineWidth', 1);
end
xlabel('Time (s)'); ylabel('q_1 (rad)');
title('Joint 1 Tracking');
legend('sign(s)', 'Desired', ...
    ['\phi=' num2str(phi_values(1))], ...
    ['\phi=' num2str(phi_values(2))], ...
    ['\phi=' num2str(phi_values(3))], ...
    ['\phi=' num2str(phi_values(4))], ...
    'Location', 'best');
grid on;

% Joint 2 tracking
subplot(2,2,2);
plot(T_sign, X_sign(:,2), 'b-', 'LineWidth', 1.5); hold on;
plot(T_sign, cos(T_sign), 'r--', 'LineWidth', 1.5);
for idx = 1:length(phi_values)
    R = results_sat{idx};
    plot(R.T, R.X(:,2), 'LineWidth', 1);
end
xlabel('Time (s)'); ylabel('q_2 (rad)');
title('Joint 2 Tracking');
grid on;

% Tracking error
subplot(2,2,3);
e_sign = X_sign(:,1:2) - [sin(T_sign), cos(T_sign)];
plot(T_sign, vecnorm(e_sign, 2, 2), 'b-', 'LineWidth', 1.5); hold on;
for idx = 1:length(phi_values)
    R = results_sat{idx};
    e_sat_i = R.X(:,1:2) - [sin(R.T), cos(R.T)];
    plot(R.T, vecnorm(e_sat_i, 2, 2), 'LineWidth', 1);
end
e_st = X_st(:,1:2) - [sin(T_st), cos(T_st)];
plot(T_st, vecnorm(e_st, 2, 2), 'k-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('||e|| (rad)');
title('Tracking Error Norm');
legend('sign(s)', ...
    ['\phi=' num2str(phi_values(1))], ...
    ['\phi=' num2str(phi_values(2))], ...
    ['\phi=' num2str(phi_values(3))], ...
    ['\phi=' num2str(phi_values(4))], ...
    'Super-Twist', 'Location', 'best');
grid on;

% Control torque (joint 1)
subplot(2,2,4);
plot(T_sign, tau_sign(:,1), 'b-', 'LineWidth', 0.8); hold on;
R = results_sat{2};  % phi = 0.05
plot(R.T, R.tau(:,1), 'g-', 'LineWidth', 0.8);
plot(T_st, tau_st(:,1), 'k-', 'LineWidth', 0.8);
xlabel('Time (s)'); ylabel('\tau_1 (Nm)');
title('Control Torque — Joint 1');
legend('sign(s)', 'sat(\phi=0.05)', 'Super-Twist', 'Location', 'best');
grid on;

sgtitle('2-DOF Manipulator — Sliding Mode Control Comparison');

% Sliding surface plot
figure('Name', 'Manipulator SMC — Sliding Surfaces', 'Position', [50 50 1200 400]);
subplot(1,3,1);
plot(T_sign, s_sign(:,1), 'b-', T_sign, s_sign(:,2), 'r-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('s'); title('Sliding Surface — sign(s)');
legend('s_1','s_2'); grid on;

subplot(1,3,2);
R = results_sat{2};
plot(R.T, R.s(:,1), 'b-', R.T, R.s(:,2), 'r-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('s'); title('Sliding Surface — sat(s/0.05)');
legend('s_1','s_2'); grid on;

subplot(1,3,3);
plot(T_st, s_st(:,1), 'b-', T_st, s_st(:,2), 'r-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('s'); title('Sliding Surface — Super-Twisting');
legend('s_1','s_2'); grid on;

sgtitle('Sliding Surface Evolution');

%% ========================================================================
%  PART 2: MOBILE ROBOT SMC
%  ========================================================================
fprintf('\n--- PART 2: Mobile Robot SMC ---\n');

% SMC gains
lambda_v = 1.0;  lambda_w = 2.0;
eta_v = 0.5;     eta_w = 1.0;
phi_v = 0.05;    phi_w = 0.05;

% Desired circular trajectory
R_circle = 2.0;  omega_circle = 0.3;
xd_mob   = @(t) R_circle * cos(omega_circle * t);
yd_mob   = @(t) R_circle * sin(omega_circle * t);
xd_d_mob = @(t) -R_circle * omega_circle * sin(omega_circle * t);
yd_d_mob = @(t)  R_circle * omega_circle * cos(omega_circle * t);

% Initial condition: [x, y, theta]
x0_mob = [0; 0; pi/4];

% Simulation parameters
dt_mob = 0.005;
t_end_mob = 40;

fprintf('  Running SMC with sat(s/phi)...\n');
[T_mob, X_mob, ctrl_mob] = run_mobile_smc(dt_mob, t_end_mob, x0_mob, ...
    lambda_v, lambda_w, eta_v, eta_w, phi_v, phi_w, ...
    xd_mob, yd_mob, xd_d_mob, yd_d_mob, 'sat');
fprintf('    Done. Final pos error: %.4f m\n', ...
    sqrt((X_mob(end,1)-xd_mob(T_mob(end)))^2 + (X_mob(end,2)-yd_mob(T_mob(end)))^2));

fprintf('  Running SMC with sign(s)...\n');
[T_mob_sign, X_mob_sign, ctrl_mob_sign] = run_mobile_smc(dt_mob, t_end_mob, x0_mob, ...
    lambda_v, lambda_w, eta_v, eta_w, 0, 0, ...
    xd_mob, yd_mob, xd_d_mob, yd_d_mob, 'sign');
fprintf('    Done. Final pos error: %.4f m\n', ...
    sqrt((X_mob_sign(end,1)-xd_mob(T_mob_sign(end)))^2 + (X_mob_sign(end,2)-yd_mob(T_mob_sign(end)))^2));

% ---- PLOTTING: Mobile Robot ----
figure('Name', 'Mobile Robot SMC', 'Position', [100 300 1200 500]);

subplot(1,3,1);
plot(X_mob(:,1), X_mob(:,2), 'b-', 'LineWidth', 1.5); hold on;
plot(X_mob_sign(:,1), X_mob_sign(:,2), 'g-', 'LineWidth', 1);
xd_vals = arrayfun(xd_mob, T_mob);
yd_vals = arrayfun(yd_mob, T_mob);
plot(xd_vals, yd_vals, 'r--', 'LineWidth', 1.5);
plot(X_mob(1,1), X_mob(1,2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
xlabel('x (m)'); ylabel('y (m)');
title('Path Tracking');
legend('sat(s/\phi)', 'sign(s)', 'Desired', 'Start', 'Location', 'best');
axis equal; grid on;

subplot(1,3,2);
ex_mob = arrayfun(xd_mob, T_mob) - X_mob(:,1);
ey_mob = arrayfun(yd_mob, T_mob) - X_mob(:,2);
plot(T_mob, sqrt(ex_mob.^2 + ey_mob.^2), 'b-', 'LineWidth', 1.5); hold on;
ex_mob_s = arrayfun(xd_mob, T_mob_sign) - X_mob_sign(:,1);
ey_mob_s = arrayfun(yd_mob, T_mob_sign) - X_mob_sign(:,2);
plot(T_mob_sign, sqrt(ex_mob_s.^2 + ey_mob_s.^2), 'g-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Position Error (m)');
title('Tracking Error');
legend('sat(s/\phi)', 'sign(s)', 'Location', 'best');
grid on;

subplot(1,3,3);
plot(T_mob, ctrl_mob(:,1), 'b-', T_mob, ctrl_mob(:,2), 'r-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Control');
title('Control Inputs (sat)');
legend('v (m/s)', '\omega (rad/s)', 'Location', 'best');
grid on;

sgtitle('Mobile Robot — Sliding Mode Control');

%% ========================================================================
%  PART 3: QUADROTOR DRONE SMC
%  ========================================================================
fprintf('\n--- PART 3: Quadrotor Drone SMC ---\n');

% Physical parameters
m_drone = 1.0;   g_drone = 9.81;
Ixx = 0.01;  Iyy = 0.01;  Izz = 0.02;
l_arm = 0.2;  c_drag = 0.01;

% SMC gains — position (outer loop)
% Note: position gains must be moderate to avoid demanding extreme tilt
% angles that destabilize the cascaded controller
lam_pos = [2; 2; 3];
eta_pos = [2; 2; 3];
k_pos   = [0.3; 0.3; 0.5];
phi_pos = [0.2; 0.2; 0.2];

% SMC gains — attitude (inner loop)
% Fast inner loop is critical for time-scale separation in cascaded SMC
lam_att = [20; 20; 10];
eta_att = [5; 5; 3];
phi_att = [0.02; 0.02; 0.02];

% Desired trajectory: helix
xd_drone    = @(t) cos(0.5*t);
yd_drone    = @(t) sin(0.5*t);
zd_drone    = @(t) 0.5*t;
xd_d_drone  = @(t) -0.5*sin(0.5*t);
yd_d_drone  = @(t)  0.5*cos(0.5*t);
zd_d_drone  = @(t) 0.5;
xd_dd_drone = @(t) -0.25*cos(0.5*t);
yd_dd_drone = @(t) -0.25*sin(0.5*t);
zd_dd_drone = @(t) 0;
psi_d_drone = @(t) 0;

% State: [x,y,z, dx,dy,dz, phi,theta,psi, dphi,dtheta,dpsi]
% Start at reference position
x0_drone = [xd_drone(0); yd_drone(0); zd_drone(0);  ...
            xd_d_drone(0); yd_d_drone(0); zd_d_drone(0);  ...
            0; 0; 0;  0; 0; 0];

dt_drone = 0.001;
t_end_drone = 25;

% Disturbance functions
dist_drone_zero = @(t) zeros(6,1);
dist_drone_wind = @(t) [2*sin(t); 2*cos(t); sin(2*t); 0.1*sin(3*t); 0.1*cos(3*t); 0];

fprintf('  Running Drone SMC (no disturbance)...\n');
[T_drone, X_drone, U_drone] = run_drone_smc(dt_drone, t_end_drone, x0_drone, ...
    m_drone, g_drone, Ixx, Iyy, Izz, l_arm, c_drag, ...
    lam_pos, eta_pos, k_pos, phi_pos, lam_att, eta_att, phi_att, ...
    xd_drone, yd_drone, zd_drone, xd_d_drone, yd_d_drone, zd_d_drone, ...
    xd_dd_drone, yd_dd_drone, zd_dd_drone, psi_d_drone, ...
    dist_drone_zero);
fprintf('    Done. Final 3D error: %.4f m\n', ...
    norm(X_drone(end,1:3) - [xd_drone(T_drone(end)), yd_drone(T_drone(end)), zd_drone(T_drone(end))]));

fprintf('  Running Drone SMC (with wind disturbance)...\n');
[T_drone_d, X_drone_d, U_drone_d] = run_drone_smc(dt_drone, t_end_drone, x0_drone, ...
    m_drone, g_drone, Ixx, Iyy, Izz, l_arm, c_drag, ...
    lam_pos, eta_pos, k_pos, phi_pos, lam_att, eta_att, phi_att, ...
    xd_drone, yd_drone, zd_drone, xd_d_drone, yd_d_drone, zd_d_drone, ...
    xd_dd_drone, yd_dd_drone, zd_dd_drone, psi_d_drone, ...
    dist_drone_wind);
fprintf('    Done. Final 3D error: %.4f m\n', ...
    norm(X_drone_d(end,1:3) - [xd_drone(T_drone_d(end)), yd_drone(T_drone_d(end)), zd_drone(T_drone_d(end))]));

% ---- PLOTTING: Drone ----
figure('Name', 'Drone SMC — 3D Tracking', 'Position', [150 200 1200 600]);

subplot(1,2,1);
plot3(X_drone(:,1), X_drone(:,2), X_drone(:,3), 'b-', 'LineWidth', 1.5); hold on;
xd_v = arrayfun(xd_drone, T_drone);
yd_v = arrayfun(yd_drone, T_drone);
zd_v = arrayfun(zd_drone, T_drone);
plot3(xd_v, yd_v, zd_v, 'r--', 'LineWidth', 1.5);
plot3(X_drone(1,1), X_drone(1,2), X_drone(1,3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('3D Tracking — No Disturbance');
legend('Actual', 'Desired', 'Start', 'Location', 'best');
grid on; view(30, 25);

subplot(1,2,2);
plot3(X_drone_d(:,1), X_drone_d(:,2), X_drone_d(:,3), 'b-', 'LineWidth', 1.5); hold on;
xd_v2 = arrayfun(xd_drone, T_drone_d);
yd_v2 = arrayfun(yd_drone, T_drone_d);
zd_v2 = arrayfun(zd_drone, T_drone_d);
plot3(xd_v2, yd_v2, zd_v2, 'r--', 'LineWidth', 1.5);
plot3(X_drone_d(1,1), X_drone_d(1,2), X_drone_d(1,3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('3D Tracking — With Wind Disturbance');
legend('Actual', 'Desired', 'Start', 'Location', 'best');
grid on; view(30, 25);

sgtitle('Quadrotor Drone — Hierarchical SMC');

% Position error plots
figure('Name', 'Drone SMC — Position Errors', 'Position', [150 50 1200 400]);

subplot(1,3,1);
plot(T_drone, X_drone(:,1) - xd_v, 'b-', 'LineWidth', 1); hold on;
plot(T_drone_d, X_drone_d(:,1) - xd_v2, 'r-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('e_x (m)'); title('X Error');
legend('No Dist', 'Wind', 'Location', 'best'); grid on;

subplot(1,3,2);
plot(T_drone, X_drone(:,2) - yd_v, 'b-', 'LineWidth', 1); hold on;
plot(T_drone_d, X_drone_d(:,2) - yd_v2, 'r-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('e_y (m)'); title('Y Error');
legend('No Dist', 'Wind', 'Location', 'best'); grid on;

subplot(1,3,3);
plot(T_drone, X_drone(:,3) - zd_v, 'b-', 'LineWidth', 1); hold on;
plot(T_drone_d, X_drone_d(:,3) - zd_v2, 'r-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('e_z (m)'); title('Z Error');
legend('No Dist', 'Wind', 'Location', 'best'); grid on;

sgtitle('Drone Position Tracking Errors');

% Drone control inputs
figure('Name', 'Drone SMC — Control Inputs', 'Position', [200 100 1200 400]);

subplot(1,4,1);
plot(T_drone, U_drone(:,1), 'b-', T_drone_d, U_drone_d(:,1), 'r-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('U_1 (N)'); title('Thrust'); legend('No Dist','Wind'); grid on;

subplot(1,4,2);
plot(T_drone, U_drone(:,2), 'b-', T_drone_d, U_drone_d(:,2), 'r-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('U_2 (Nm)'); title('Roll Torque'); grid on;

subplot(1,4,3);
plot(T_drone, U_drone(:,3), 'b-', T_drone_d, U_drone_d(:,3), 'r-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('U_3 (Nm)'); title('Pitch Torque'); grid on;

subplot(1,4,4);
plot(T_drone, U_drone(:,4), 'b-', T_drone_d, U_drone_d(:,4), 'r-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('U_4 (Nm)'); title('Yaw Torque'); grid on;

sgtitle('Drone Control Inputs');

%% ========================================================================
%  ACTIVITY 2: DISTURBANCE REJECTION COMPARISON
%  ========================================================================
fprintf('\n--- ACTIVITY 2: Disturbance Rejection Comparison ---\n');

% SMC sign with disturbance
fprintf('  Running SMC sign(s) with disturbance...\n');
[T_d1, X_d1, tau_d1, ~] = run_manipulator_smc( ...
    dt_manip, t_end_manip, x0_manip, m1, m2, l1, l2, g, ...
    Lambda_manip, K_sign, k_reach, 0, 'sign', [], [], ...
    qd_fun, qd_d_fun, qd_dd_fun, dist_fun_large);
fprintf('    Done.\n');

% SMC sat with disturbance
fprintf('  Running SMC sat(s/phi) with disturbance...\n');
[T_d2, X_d2, tau_d2, ~] = run_manipulator_smc( ...
    dt_manip, t_end_manip, x0_manip, m1, m2, l1, l2, g, ...
    Lambda_manip, K_sat, k_reach, 0.05, 'sat', [], [], ...
    qd_fun, qd_d_fun, qd_dd_fun, dist_fun_large);
fprintf('    Done.\n');

% Computed torque with disturbance (no switching term, just model-based)
fprintf('  Running Computed Torque with disturbance...\n');
[T_d3, X_d3, tau_d3, ~] = run_manipulator_smc( ...
    dt_manip, t_end_manip, x0_manip, m1, m2, l1, l2, g, ...
    Lambda_manip, diag([0, 0]), k_reach, 0, 'sat', [], [], ...
    qd_fun, qd_d_fun, qd_dd_fun, dist_fun_large);
fprintf('    Done.\n');

% Super-twisting with disturbance
fprintf('  Running Super-Twisting SMC with disturbance...\n');
[T_d4, X_d4, tau_d4, ~] = run_manipulator_smc( ...
    dt_manip, t_end_manip, x0_st, m1, m2, l1, l2, g, ...
    Lambda_manip, K_sign, k_reach, 0, 'super_twisting', k1_st, k2_st, ...
    qd_fun, qd_d_fun, qd_dd_fun, dist_fun_large);
fprintf('    Done.\n');

% ---- PLOTTING: Disturbance Comparison ----
figure('Name', 'Activity 2 — Disturbance Rejection', 'Position', [100 100 1200 600]);

% Tracking error comparison
subplot(2,2,1);
e_d1 = X_d1(:,1:2) - [sin(T_d1), cos(T_d1)];
e_d2 = X_d2(:,1:2) - [sin(T_d2), cos(T_d2)];
e_d3 = X_d3(:,1:2) - [sin(T_d3), cos(T_d3)];
e_d4 = X_d4(:,1:2) - [sin(T_d4), cos(T_d4)];

plot(T_d1, vecnorm(e_d1, 2, 2), 'b-', 'LineWidth', 1.2); hold on;
plot(T_d2, vecnorm(e_d2, 2, 2), 'g-', 'LineWidth', 1.2);
plot(T_d3, vecnorm(e_d3, 2, 2), 'r-', 'LineWidth', 1.2);
plot(T_d4, vecnorm(e_d4, 2, 2), 'k-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('||e|| (rad)');
title('Tracking Error Under d(t)=10sin(5t)');
legend('SMC sign', 'SMC sat', 'Computed Torque', 'Super-Twist', 'Location', 'best');
grid on;

% Control torque comparison (joint 1)
subplot(2,2,2);
plot(T_d1, tau_d1(:,1), 'b-', 'LineWidth', 0.8); hold on;
plot(T_d2, tau_d2(:,1), 'g-', 'LineWidth', 0.8);
plot(T_d3, tau_d3(:,1), 'r-', 'LineWidth', 0.8);
plot(T_d4, tau_d4(:,1), 'k-', 'LineWidth', 0.8);
xlabel('Time (s)'); ylabel('\tau_1 (Nm)');
title('Control Torque — Joint 1');
legend('SMC sign', 'SMC sat', 'Computed Torque', 'Super-Twist', 'Location', 'best');
grid on;

% Joint 1 tracking
subplot(2,2,3);
plot(T_d1, X_d1(:,1), 'b-', 'LineWidth', 1.2); hold on;
plot(T_d2, X_d2(:,1), 'g-', 'LineWidth', 1.2);
plot(T_d3, X_d3(:,1), 'r-', 'LineWidth', 1.2);
plot(T_d4, X_d4(:,1), 'k-', 'LineWidth', 1.2);
plot(T_d1, sin(T_d1), 'm--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('q_1 (rad)');
title('Joint 1 Tracking Under Disturbance');
legend('SMC sign', 'SMC sat', 'Computed Torque', 'Super-Twist', 'Desired', 'Location', 'best');
grid on;

% Joint 2 tracking
subplot(2,2,4);
plot(T_d1, X_d1(:,2), 'b-', 'LineWidth', 1.2); hold on;
plot(T_d2, X_d2(:,2), 'g-', 'LineWidth', 1.2);
plot(T_d3, X_d3(:,2), 'r-', 'LineWidth', 1.2);
plot(T_d4, X_d4(:,2), 'k-', 'LineWidth', 1.2);
plot(T_d1, cos(T_d1), 'm--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('q_2 (rad)');
title('Joint 2 Tracking Under Disturbance');
legend('SMC sign', 'SMC sat', 'Computed Torque', 'Super-Twist', 'Desired', 'Location', 'best');
grid on;

sgtitle('Activity 2: Disturbance Rejection Comparison — d(t) = 10sin(5t)');

fprintf('\n========================================\n');
fprintf(' All simulations complete.\n');
fprintf('========================================\n');


%% ########################################################################
%  LOCAL FUNCTIONS
%  ########################################################################

%% ========================================================================
%  FIXED-STEP RK4 INTEGRATOR
%  ========================================================================
function [T, X] = rk4_integrate(odefun, dt, t_end, x0)
    N = round(t_end / dt) + 1;
    n_states = length(x0);
    T = zeros(N, 1);
    X = zeros(N, n_states);
    X(1,:) = x0';
    T(1) = 0;
    for k = 1:(N-1)
        t_k = T(k);
        x_k = X(k,:)';
        k1 = odefun(t_k,          x_k);
        k2 = odefun(t_k + dt/2,   x_k + dt/2 * k1);
        k3 = odefun(t_k + dt/2,   x_k + dt/2 * k2);
        k4 = odefun(t_k + dt,     x_k + dt   * k3);
        X(k+1,:) = (x_k + dt/6 * (k1 + 2*k2 + 2*k3 + k4))';
        T(k+1) = t_k + dt;
    end
end

%% ========================================================================
%  MANIPULATOR DYNAMICS
%  ========================================================================
function [M, C, G_vec] = manipulator_dynamics(q, qdot, m1, m2, l1, l2, g)
    % Compute M(q), C(q,qdot), G(q) for 2-DOF planar manipulator
    c2 = cos(q(2));  s2 = sin(q(2));

    % Inertia matrix
    M = [m1*l1^2 + m2*(l1^2 + l2^2 + 2*l1*l2*c2),  m2*(l2^2 + l1*l2*c2);
         m2*(l2^2 + l1*l2*c2),                        m2*l2^2];

    % Coriolis matrix
    C = [-m2*l1*l2*s2*qdot(2),   -m2*l1*l2*s2*(qdot(1) + qdot(2));
          m2*l1*l2*s2*qdot(1),    0];

    % Gravity vector
    G_vec = [m1*g*l1*cos(q(1)) + m2*g*(l1*cos(q(1)) + l2*cos(q(1)+q(2)));
             m2*g*l2*cos(q(1)+q(2))];
end

%% ========================================================================
%  SATURATION FUNCTION
%  ========================================================================
function y = sat_func(s, phi)
    % Saturation function: sat(s/phi)
    if phi <= 0
        y = sign(s);
    else
        y = min(max(s / phi, -1), 1);
    end
end

%% ========================================================================
%  RUN MANIPULATOR SMC (fixed-step RK4 wrapper)
%  ========================================================================
function [T, X, tau_hist, s_hist] = run_manipulator_smc( ...
    dt, t_end, x0, m1, m2, l1, l2, g, ...
    Lambda, K_sw, k_reach, phi, mode, k1_st, k2_st, ...
    qd_fun, qd_d_fun, qd_dd_fun, dist_fun)

    % ODE function
    odefun = @(t, x) manip_ode(t, x, m1, m2, l1, l2, g, ...
        Lambda, K_sw, k_reach, phi, mode, k1_st, k2_st, ...
        qd_fun, qd_d_fun, qd_dd_fun, dist_fun);

    % Integrate with fixed-step RK4
    [T, X] = rk4_integrate(odefun, dt, t_end, x0);

    % Reconstruct control and sliding surface
    N = length(T);
    tau_hist = zeros(N, 2);
    s_hist   = zeros(N, 2);

    for i = 1:N
        [~, tau_i, s_i] = manip_ode(T(i), X(i,:)', m1, m2, l1, l2, g, ...
            Lambda, K_sw, k_reach, phi, mode, k1_st, k2_st, ...
            qd_fun, qd_d_fun, qd_dd_fun, dist_fun);
        tau_hist(i,:) = tau_i';
        s_hist(i,:)   = s_i';
    end
end

%% ========================================================================
%  MANIPULATOR ODE
%  ========================================================================
function [dxdt, tau, s] = manip_ode(t, x, m1, m2, l1, l2, g, ...
    Lambda, K_sw, k_reach, phi, mode, k1_st, k2_st, ...
    qd_fun, qd_d_fun, qd_dd_fun, dist_fun)

    is_st = strcmp(mode, 'super_twisting');

    if is_st
        q    = x(1:2);
        qdot = x(3:4);
        v_st = x(5:6);  % super-twisting integral state
    else
        q    = x(1:2);
        qdot = x(3:4);
        v_st = [0; 0];
    end

    % Desired trajectory
    qd   = qd_fun(t);
    qd_d = qd_d_fun(t);
    qd_dd= qd_dd_fun(t);

    % Errors
    e    = q - qd;
    edot = qdot - qd_d;

    % Sliding surface
    s = edot + Lambda * e;

    % Reference velocity and acceleration
    qr_dot = qd_d - Lambda * e;
    qr_ddot= qd_dd - Lambda * edot;

    % Dynamics
    [M, C, G_vec] = manipulator_dynamics(q, qdot, m1, m2, l1, l2, g);

    % Equivalent control
    tau_eq = M * qr_ddot + C * qr_dot + G_vec;

    % Switching control
    switch mode
        case 'sign'
            tau_sw = -K_sw * sign(s) - k_reach * s;

        case 'sat'
            s_sat = [sat_func(s(1), phi); sat_func(s(2), phi)];
            tau_sw = -K_sw * s_sat - k_reach * s;

        case 'super_twisting'
            % Super-twisting: u_sw = -k1*|s|^{1/2}*sign(s) + v
            %                  dv/dt = -k2*sign(s)
            abs_s_sqrt = [sqrt(abs(s(1))); sqrt(abs(s(2)))];
            tau_sw = -k1_st * (abs_s_sqrt .* sign(s)) + v_st - k_reach * s;

        otherwise
            tau_sw = -K_sw * sign(s);
    end

    % Total torque
    tau = tau_eq + tau_sw;

    % Disturbance
    d = dist_fun(t);

    % State derivative
    qddot = M \ (tau + d - C * qdot - G_vec);

    if is_st
        dv_st = -k2_st * sign(s);
        dxdt = [qdot; qddot; dv_st];
    else
        dxdt = [qdot; qddot];
    end
end

%% ========================================================================
%  RUN MOBILE ROBOT SMC (fixed-step RK4 wrapper)
%  ========================================================================
function [T, X, ctrl_hist] = run_mobile_smc(dt, t_end, x0, ...
    lambda_v, lambda_w, eta_v, eta_w, phi_v, phi_w, ...
    xd_fun, yd_fun, xd_d_fun, yd_d_fun, mode)

    odefun = @(t, x) mobile_ode(t, x, lambda_v, lambda_w, ...
        eta_v, eta_w, phi_v, phi_w, xd_fun, yd_fun, xd_d_fun, yd_d_fun, mode);

    % Integrate with fixed-step RK4
    [T, X] = rk4_integrate(odefun, dt, t_end, x0);

    % Reconstruct control
    N = length(T);
    ctrl_hist = zeros(N, 2);
    for i = 1:N
        [~, v_i, w_i] = mobile_ode(T(i), X(i,:)', ...
            lambda_v, lambda_w, eta_v, eta_w, phi_v, phi_w, ...
            xd_fun, yd_fun, xd_d_fun, yd_d_fun, mode);
        ctrl_hist(i,:) = [v_i, w_i];
    end
end

%% ========================================================================
%  MOBILE ROBOT ODE (Unicycle with polar-coordinate SMC)
%  ========================================================================
function [dxdt, v_cmd, w_cmd] = mobile_ode(t, x, ...
    lambda_v, lambda_w, eta_v, eta_w, phi_v, phi_w, ...
    xd_fun, yd_fun, xd_d_fun, yd_d_fun, mode)

    px = x(1);  py = x(2);  theta = x(3);

    % Desired position
    xd = xd_fun(t);  yd = yd_fun(t);
    dxd = xd_d_fun(t);  dyd = yd_d_fun(t);

    % Position errors
    ex = xd - px;
    ey = yd - py;

    % Polar coordinates
    rho = sqrt(ex^2 + ey^2);
    if rho < 1e-6
        rho = 1e-6;  % avoid singularity
    end
    alpha = atan2(ey, ex) - theta;
    % Wrap alpha to [-pi, pi]
    alpha = mod(alpha + pi, 2*pi) - pi;

    % Clamp alpha to avoid cos(alpha) = 0 singularity
    alpha_max = pi/2 - 0.05;
    alpha = max(min(alpha, alpha_max), -alpha_max);

    % rho_dot (from desired trajectory for sliding surface)
    rho_dot = -(dxd*ex + dyd*ey) / rho;  % approximate drho/dt from desired

    % Sliding surfaces
    s_v = -rho_dot + lambda_v * rho;  % note: drho/dt = -v*cos(alpha), so s_v involves v
    s_w = lambda_w * alpha;           % simplified for angular

    % Switching functions
    if strcmp(mode, 'sign')
        sw_v = sign(s_v);
        sw_w = sign(s_w);
    else
        sw_v = sat_func(s_v, phi_v);
        sw_w = sat_func(s_w, phi_w);
    end

    % Control laws
    cos_alpha = cos(alpha);
    if abs(cos_alpha) < 0.1
        cos_alpha = 0.1 * sign(cos_alpha + 1e-10);
    end

    v_cmd = (lambda_v * rho + eta_v * sw_v) / cos_alpha;

    % Clamp velocity to reasonable range
    v_cmd = max(min(v_cmd, 5.0), -1.0);

    w_cmd = v_cmd * sin(alpha) / rho + lambda_w * alpha + eta_w * sw_w;

    % Clamp angular velocity
    w_cmd = max(min(w_cmd, 5.0), -5.0);

    % Unicycle kinematics
    dxdt = [v_cmd * cos(theta);
            v_cmd * sin(theta);
            w_cmd];
end

%% ========================================================================
%  RUN DRONE SMC (fixed-step RK4 wrapper)
%  ========================================================================
function [T, X, U_hist] = run_drone_smc(dt, t_end, x0, ...
    m, g_val, Ixx, Iyy, Izz, l_arm, c_drag, ...
    lam_pos, eta_pos, k_pos, phi_pos, ...
    lam_att, eta_att, phi_att, ...
    xd_f, yd_f, zd_f, xd_d_f, yd_d_f, zd_d_f, ...
    xd_dd_f, yd_dd_f, zd_dd_f, psi_d_f, ...
    dist_fun)

    odefun = @(t, x) drone_ode(t, x, m, g_val, Ixx, Iyy, Izz, l_arm, c_drag, ...
        lam_pos, eta_pos, k_pos, phi_pos, ...
        lam_att, eta_att, phi_att, ...
        xd_f, yd_f, zd_f, xd_d_f, yd_d_f, zd_d_f, ...
        xd_dd_f, yd_dd_f, zd_dd_f, psi_d_f, dist_fun);

    % Integrate with fixed-step RK4
    [T, X] = rk4_integrate(odefun, dt, t_end, x0);

    % Reconstruct control (subsample every 10 steps to save memory)
    step = 10;
    idx = 1:step:length(T);
    T_sub = T(idx);
    X_sub = X(idx,:);
    N = length(idx);
    U_hist_sub = zeros(N, 4);
    for i = 1:N
        [~, U_i] = drone_ode(T_sub(i), X_sub(i,:)', m, g_val, Ixx, Iyy, Izz, l_arm, c_drag, ...
            lam_pos, eta_pos, k_pos, phi_pos, ...
            lam_att, eta_att, phi_att, ...
            xd_f, yd_f, zd_f, xd_d_f, yd_d_f, zd_d_f, ...
            xd_dd_f, yd_dd_f, zd_dd_f, psi_d_f, dist_fun);
        U_hist_sub(i,:) = U_i';
    end

    % Return subsampled for plotting (full resolution not needed for plots)
    T = T_sub;
    X = X_sub;
    U_hist = U_hist_sub;
end

%% ========================================================================
%  DRONE ODE (Hierarchical SMC)
%  ========================================================================
function [dxdt, U_ctrl] = drone_ode(t, x, m, g_val, Ixx, Iyy, Izz, l_arm, c_drag, ...
    lam_pos, eta_pos, k_pos, phi_pos, ...
    lam_att, eta_att, phi_att, ...
    xd_f, yd_f, zd_f, xd_d_f, yd_d_f, zd_d_f, ...
    xd_dd_f, yd_dd_f, zd_dd_f, psi_d_f, dist_fun)

    % Unpack state
    % x = [px, py, pz, vx, vy, vz, phi, theta, psi, dphi, dtheta, dpsi]
    pos  = x(1:3);    vel  = x(4:6);
    att  = x(7:9);    datt = x(10:12);

    phi_a   = att(1);  theta_a = att(2);  psi_a = att(3);
    dphi_a  = datt(1); dtheta_a= datt(2); dpsi_a = datt(3);

    % Disturbance
    d = dist_fun(t);
    d_pos = d(1:3);   % position disturbances
    d_att = d(4:6);   % attitude disturbances

    % Desired trajectory
    xd  = xd_f(t);    yd  = yd_f(t);    zd  = zd_f(t);
    dxd = xd_d_f(t);  dyd = yd_d_f(t);  dzd = zd_d_f(t);
    ddxd= xd_dd_f(t); ddyd= yd_dd_f(t); ddzd= zd_dd_f(t);
    psi_d = psi_d_f(t);

    % ---- OUTER LOOP: Position SMC ----
    % Position errors
    e_pos  = pos - [xd; yd; zd];
    ed_pos = vel - [dxd; dyd; dzd];

    % Sliding surfaces
    s_pos = ed_pos + lam_pos .* e_pos;

    % SMC virtual accelerations
    s_sat_pos = [sat_func(s_pos(1), phi_pos(1));
                 sat_func(s_pos(2), phi_pos(2));
                 sat_func(s_pos(3), phi_pos(3))];

    accel_cmd = [ddxd; ddyd; ddzd] - lam_pos .* ed_pos ...
                - eta_pos .* s_sat_pos - k_pos .* s_pos;

    % Thrust (altitude)
    cth = cos(theta_a);  cphi = cos(phi_a);
    denom_z = cth * cphi;
    if abs(denom_z) < 0.1
        denom_z = 0.1 * sign(denom_z + 1e-10);
    end
    U1 = m * (accel_cmd(3) + g_val) / denom_z;
    U1 = max(U1, 0);  % thrust must be positive

    % Desired roll and pitch from lateral accelerations
    accel_z_cmd = accel_cmd(3) + g_val;
    if abs(accel_z_cmd) < 0.1
        accel_z_cmd = 0.1;
    end

    % Clamp lateral acceleration commands to prevent extreme tilt
    accel_lat_max = 5.0;  % max lateral accel ≈ tan(27°)*g
    accel_cmd(1) = max(min(accel_cmd(1), accel_lat_max), -accel_lat_max);
    accel_cmd(2) = max(min(accel_cmd(2), accel_lat_max), -accel_lat_max);

    theta_d = atan2(accel_cmd(1)*cos(psi_a) + accel_cmd(2)*sin(psi_a), accel_z_cmd);
    phi_d   = atan2((accel_cmd(1)*sin(psi_a) - accel_cmd(2)*cos(psi_a)) * cos(theta_d), accel_z_cmd);

    % Clamp desired angles (~27 degrees max tilt)
    angle_max = 0.47;
    theta_d = max(min(theta_d, angle_max), -angle_max);
    phi_d   = max(min(phi_d, angle_max), -angle_max);

    % Desired attitude derivatives (numerical approx — zero for simplicity)
    dphi_d = 0;  dtheta_d = 0;  dpsi_d = 0;
    ddphi_d = 0; ddtheta_d = 0; ddpsi_d = 0;

    % ---- INNER LOOP: Attitude SMC ----
    e_att  = att - [phi_d; theta_d; psi_d];
    ed_att = datt - [dphi_d; dtheta_d; dpsi_d];

    s_att = ed_att + lam_att .* e_att;

    s_sat_att = [sat_func(s_att(1), phi_att(1));
                 sat_func(s_att(2), phi_att(2));
                 sat_func(s_att(3), phi_att(3))];

    % Gyroscopic coupling terms
    gyro_phi   = (Iyy - Izz) / Ixx * dtheta_a * dpsi_a;
    gyro_theta = (Izz - Ixx) / Iyy * dphi_a * dpsi_a;
    gyro_psi   = (Ixx - Iyy) / Izz * dphi_a * dtheta_a;

    % Attitude control torques
    U2 = Ixx / l_arm * (ddphi_d   - lam_att(1)*ed_att(1) - gyro_phi   - eta_att(1)*s_sat_att(1));
    U3 = Iyy / l_arm * (ddtheta_d - lam_att(2)*ed_att(2) - gyro_theta - eta_att(2)*s_sat_att(2));
    U4 = Izz / c_drag * (ddpsi_d  - lam_att(3)*ed_att(3) - gyro_psi   - eta_att(3)*s_sat_att(3));

    U_ctrl = [U1; U2; U3; U4];

    % ---- DYNAMICS ----
    % Translational dynamics
    ax = (cos(psi_a)*sin(theta_a)*cos(phi_a) + sin(psi_a)*sin(phi_a)) * U1 / m + d_pos(1)/m;
    ay = (sin(psi_a)*sin(theta_a)*cos(phi_a) - cos(psi_a)*sin(phi_a)) * U1 / m + d_pos(2)/m;
    az = cos(theta_a)*cos(phi_a) * U1 / m - g_val + d_pos(3)/m;

    % Rotational dynamics
    ddphi   = gyro_phi   + l_arm / Ixx * U2 + d_att(1)/Ixx;
    ddtheta = gyro_theta + l_arm / Iyy * U3 + d_att(2)/Iyy;
    ddpsi   = gyro_psi   + c_drag / Izz * U4 + d_att(3)/Izz;

    dxdt = [vel; ax; ay; az; datt; ddphi; ddtheta; ddpsi];
end
