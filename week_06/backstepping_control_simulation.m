%% ========================================================================
%  BACKSTEPPING CONTROL SIMULATION
%  Week 5: Control for Robots - M.Sc.
%  ========================================================================
%  This file implements backstepping control for three robotic systems:
%    1. 2-DOF Robot Manipulator
%    2. Unicycle Mobile Robot
%    3. Quadrotor Drone
%
%  Each system is simulated with ode45, and the results are plotted with
%  tracking performance, control effort, and error analysis.
%  ========================================================================

clear all; close all; clc;
fprintf('=== Week 5: Backstepping Control Simulation ===\n\n');

%% ========================================================================
%  SYSTEM 1: 2-DOF ROBOT MANIPULATOR
%  ========================================================================
fprintf('--- System 1: Robot Manipulator ---\n');

% Physical parameters
manip.m1 = 2.0;    % Mass of link 1 (kg)
manip.m2 = 1.5;    % Mass of link 2 (kg)
manip.l1 = 1.0;    % Length of link 1 (m)
manip.l2 = 0.8;    % Length of link 2 (m)
manip.lc1 = 0.5;   % Distance to COM of link 1 (m)
manip.lc2 = 0.4;   % Distance to COM of link 2 (m)
manip.I1 = 0.1;    % Inertia of link 1 (kg*m^2)
manip.I2 = 0.08;   % Inertia of link 2 (kg*m^2)
manip.g  = 9.81;   % Gravity (m/s^2)

% Backstepping gains
manip.k1 = 20;     % Position error gain
manip.k2 = 15;     % Velocity error gain

% Simulation
t_span = 0:0.005:10;
x0_manip = [0.2; -0.3; 0; 0];  % [q1, q2, dq1, dq2]

[t_m, x_m] = ode45(@(t,x) manip_backstep_ode(t, x, manip), t_span, x0_manip);

% Compute desired trajectory and errors for plotting
qd_m  = zeros(length(t_m), 2);
dqd_m = zeros(length(t_m), 2);
tau_m  = zeros(length(t_m), 2);
for i = 1:length(t_m)
    [qd_m(i,:), dqd_m(i,:), ~] = manip_desired_traj(t_m(i));
    tau_m(i,:) = manip_backstep_control(t_m(i), x_m(i,1:2)', x_m(i,3:4)', manip)';
end

% Plot manipulator results
figure('Name','Manipulator: Backstepping Control','Position',[50 50 1200 900]);

subplot(3,2,1);
plot(t_m, x_m(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t_m, qd_m(:,1), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('q_1 (rad)');
title('Joint 1: Tracking'); legend('Actual','Desired');
grid on;

subplot(3,2,2);
plot(t_m, x_m(:,2), 'b-', 'LineWidth', 1.5); hold on;
plot(t_m, qd_m(:,2), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('q_2 (rad)');
title('Joint 2: Tracking'); legend('Actual','Desired');
grid on;

subplot(3,2,3);
plot(t_m, x_m(:,1)-qd_m(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t_m, x_m(:,2)-qd_m(:,2), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Error (rad)');
title('Position Tracking Errors');
legend('e_1 = q_1 - q_{1d}','e_2 = q_2 - q_{2d}');
grid on;

subplot(3,2,4);
plot(t_m, x_m(:,3)-dqd_m(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t_m, x_m(:,4)-dqd_m(:,2), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Error (rad/s)');
title('Velocity Tracking Errors');
legend('de_1','de_2');
grid on;

subplot(3,2,5);
plot(t_m, tau_m(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t_m, tau_m(:,2), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\tau (N\cdotm)');
title('Control Torques'); legend('\tau_1','\tau_2');
grid on;

subplot(3,2,6);
% Lyapunov function
z1_m = x_m(:,1:2) - qd_m;
dz1_m = x_m(:,3:4) - dqd_m;
alpha1_m = dqd_m - manip.k1 * z1_m;
z2_m = x_m(:,3:4) - alpha1_m;
V_m = 0.5*sum(z1_m.^2, 2) + 0.5*sum(z2_m.^2, 2);
plot(t_m, V_m, 'k-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('V(t)');
title('Lyapunov Function V = 0.5||z_1||^2 + 0.5||z_2||^2');
grid on;

sgtitle('System 1: Manipulator Backstepping Control', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('  Final position error norm: %.6f rad\n', norm(z1_m(end,:)));
fprintf('  Final velocity error norm: %.6f rad/s\n', norm(dz1_m(end,:)));

%% ========================================================================
%  SYSTEM 2: UNICYCLE MOBILE ROBOT
%  ========================================================================
fprintf('\n--- System 2: Mobile Robot ---\n');

% Backstepping gains
mobile.k1 = 3.0;   % Longitudinal error gain
mobile.k2 = 5.0;   % Lateral error gain (coupling with v_r)
mobile.k3 = 4.0;   % Heading error gain

% Simulation
t_span_mob = 0:0.01:20;
x0_mobile = [1.0; 1.0; pi/6];  % Initial [x, y, theta] offset from reference

[t_mob, x_mob] = ode45(@(t,x) mobile_backstep_ode(t, x, mobile), t_span_mob, x0_mobile);

% Compute reference trajectory and controls for plotting
xr_mob = zeros(length(t_mob), 3);
v_mob  = zeros(length(t_mob), 1);
w_mob  = zeros(length(t_mob), 1);
err_mob = zeros(length(t_mob), 3);
for i = 1:length(t_mob)
    [xr, yr, thetar, vr, wr] = mobile_reference(t_mob(i));
    xr_mob(i,:) = [xr, yr, thetar];

    % Body-frame errors
    dx = xr - x_mob(i,1);
    dy = yr - x_mob(i,2);
    th = x_mob(i,3);
    e1 =  cos(th)*dx + sin(th)*dy;
    e2 = -sin(th)*dx + cos(th)*dy;
    e3 = thetar - th;
    e3 = atan2(sin(e3), cos(e3));  % wrap to [-pi, pi]
    err_mob(i,:) = [e1, e2, e3];

    v_mob(i) = vr*cos(e3) + mobile.k1*e1;
    w_mob(i) = wr + mobile.k2*vr*e2 + mobile.k3*sin(e3);
end

% Plot mobile robot results
figure('Name','Mobile Robot: Backstepping Control','Position',[100 50 1200 900]);

subplot(3,2,1);
plot(x_mob(:,1), x_mob(:,2), 'b-', 'LineWidth', 1.5); hold on;
plot(xr_mob(:,1), xr_mob(:,2), 'r--', 'LineWidth', 1.5);
plot(x_mob(1,1), x_mob(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(x_mob(end,1), x_mob(end,2), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
xlabel('x (m)'); ylabel('y (m)');
title('Path Tracking (XY Plane)');
legend('Actual','Reference','Start','End');
axis equal; grid on;

subplot(3,2,2);
plot(t_mob, x_mob(:,3)*180/pi, 'b-', 'LineWidth', 1.5); hold on;
plot(t_mob, xr_mob(:,3)*180/pi, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\theta (deg)');
title('Heading Tracking'); legend('Actual','Reference');
grid on;

subplot(3,2,3);
plot(t_mob, err_mob(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t_mob, err_mob(:,2), 'r-', 'LineWidth', 1.5);
plot(t_mob, err_mob(:,3), 'g-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Error');
title('Body-Frame Tracking Errors');
legend('e_1 (longitudinal)','e_2 (lateral)','e_3 (heading)');
grid on;

subplot(3,2,4);
V_mob = 0.5*(err_mob(:,1).^2 + err_mob(:,2).^2 + err_mob(:,3).^2);
plot(t_mob, V_mob, 'k-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('V(t)');
title('Lyapunov Function V = 0.5(e_1^2+e_2^2+e_3^2)');
grid on;

subplot(3,2,5);
plot(t_mob, v_mob, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('v (m/s)');
title('Linear Velocity Command');
grid on;

subplot(3,2,6);
plot(t_mob, w_mob, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\omega (rad/s)');
title('Angular Velocity Command');
grid on;

sgtitle('System 2: Mobile Robot Backstepping Control', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('  Final error norms: e1=%.4f m, e2=%.4f m, e3=%.4f rad\n', ...
    abs(err_mob(end,1)), abs(err_mob(end,2)), abs(err_mob(end,3)));

%% ========================================================================
%  SYSTEM 3: QUADROTOR DRONE
%  ========================================================================
fprintf('\n--- System 3: Quadrotor Drone ---\n');

% Physical parameters
quad.m   = 1.5;     % Mass (kg)
quad.g   = 9.81;    % Gravity (m/s^2)
quad.Ixx = 0.03;    % Inertia about x (kg*m^2)
quad.Iyy = 0.03;    % Inertia about y (kg*m^2)
quad.Izz = 0.06;    % Inertia about z (kg*m^2)

% Backstepping gains
quad.kp1 = 4.0;     % Position error gain
quad.kp2 = 5.0;     % Velocity error gain
quad.ka1 = 15.0;    % Attitude error gain
quad.ka2 = 8.0;     % Angular rate error gain

% Simulation
t_span_q = 0:0.005:15;
% State: [x y z vx vy vz phi theta psi p q r]
x0_quad = zeros(12, 1);
% Start at the reference trajectory initial point to avoid a large
% transient that demands extreme attitudes and destabilises the inner loop
[p0_ref, v0_ref, ~] = quad_reference(0);
x0_quad(1:3) = p0_ref;       % position = [2; 0; 2]
x0_quad(4:6) = v0_ref;       % velocity = [0; 0.8; 0.2]

[t_q, x_q] = ode45(@(t,x) quad_backstep_ode(t, x, quad), t_span_q, x0_quad);

% Compute reference and controls
pos_d_q   = zeros(length(t_q), 3);
vel_d_q   = zeros(length(t_q), 3);
thrust_q  = zeros(length(t_q), 1);
torque_q  = zeros(length(t_q), 3);
for i = 1:length(t_q)
    [pd, vd, ad] = quad_reference(t_q(i));
    pos_d_q(i,:) = pd';
    vel_d_q(i,:) = vd';
    [T_i, tau_i] = quad_backstep_control(x_q(i,:)', t_q(i), quad);
    thrust_q(i) = T_i;
    torque_q(i,:) = tau_i';
end

% Plot quadrotor results
figure('Name','Quadrotor: Backstepping Control','Position',[150 50 1400 1000]);

subplot(3,3,1);
plot3(x_q(:,1), x_q(:,2), x_q(:,3), 'b-', 'LineWidth', 1.5); hold on;
plot3(pos_d_q(:,1), pos_d_q(:,2), pos_d_q(:,3), 'r--', 'LineWidth', 1.5);
plot3(x_q(1,1), x_q(1,2), x_q(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
title('3D Trajectory'); legend('Actual','Reference','Start');
grid on; view(45, 30);

subplot(3,3,2);
plot(t_q, x_q(:,1), 'b-', 'LineWidth', 1.2); hold on;
plot(t_q, pos_d_q(:,1), 'r--', 'LineWidth', 1.2);
plot(t_q, x_q(:,2), 'g-', 'LineWidth', 1.2);
plot(t_q, pos_d_q(:,2), 'm--', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Position (m)');
title('X-Y Tracking');
legend('x','x_d','y','y_d');
grid on;

subplot(3,3,3);
plot(t_q, x_q(:,3), 'b-', 'LineWidth', 1.5); hold on;
plot(t_q, pos_d_q(:,3), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('z (m)');
title('Altitude Tracking'); legend('Actual','Desired');
grid on;

subplot(3,3,4);
plot(t_q, x_q(:,1)-pos_d_q(:,1), 'b-', 'LineWidth', 1.2); hold on;
plot(t_q, x_q(:,2)-pos_d_q(:,2), 'r-', 'LineWidth', 1.2);
plot(t_q, x_q(:,3)-pos_d_q(:,3), 'g-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Error (m)');
title('Position Tracking Errors');
legend('e_x','e_y','e_z');
grid on;

subplot(3,3,5);
plot(t_q, x_q(:,7)*180/pi, 'b-', 'LineWidth', 1.2); hold on;
plot(t_q, x_q(:,8)*180/pi, 'r-', 'LineWidth', 1.2);
plot(t_q, x_q(:,9)*180/pi, 'g-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Euler Angles');
legend('\phi (roll)','\theta (pitch)','\psi (yaw)');
grid on;

subplot(3,3,6);
plot(t_q, x_q(:,10)*180/pi, 'b-', 'LineWidth', 1.2); hold on;
plot(t_q, x_q(:,11)*180/pi, 'r-', 'LineWidth', 1.2);
plot(t_q, x_q(:,12)*180/pi, 'g-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Rate (deg/s)');
title('Angular Rates');
legend('p','q','r');
grid on;

subplot(3,3,7);
plot(t_q, thrust_q, 'k-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('T (N)');
title('Thrust Command');
grid on; hold on;
yline(quad.m*quad.g, 'r--', 'Hover thrust');

subplot(3,3,8);
plot(t_q, torque_q(:,1), 'b-', 'LineWidth', 1.2); hold on;
plot(t_q, torque_q(:,2), 'r-', 'LineWidth', 1.2);
plot(t_q, torque_q(:,3), 'g-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('\tau (N\cdotm)');
title('Body Torques');
legend('\tau_\phi','\tau_\theta','\tau_\psi');
grid on;

subplot(3,3,9);
pos_err_q = x_q(:,1:3) - pos_d_q;
V_q = 0.5*sum(pos_err_q.^2, 2) + 0.5*sum((x_q(:,4:6) - vel_d_q).^2, 2);
plot(t_q, V_q, 'k-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('V(t)');
title('Lyapunov Function (Position+Velocity)');
grid on;

sgtitle('System 3: Quadrotor Backstepping Control', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('  Final position error norm: %.4f m\n', norm(pos_err_q(end,:)));
fprintf('  Final velocity error norm: %.4f m/s\n', norm(x_q(end,4:6)-vel_d_q(end,:)));

%% ========================================================================
%  COMPARATIVE STUDY: BACKSTEPPING vs COMPUTED TORQUE vs ROBUST
%  (Manipulator with 20% mass uncertainty)
%  ========================================================================
fprintf('\n--- Comparative Study: Backstepping vs Computed Torque vs Robust ---\n');

% True system uses original parameters (manip)
% Controllers use 20% mass error
manip_uncertain = manip;
manip_uncertain.m1_hat = manip.m1 * 1.2;  % 20% overestimate
manip_uncertain.m2_hat = manip.m2 * 1.2;
manip_uncertain.k1 = 20; manip_uncertain.k2 = 15;  % Backstepping gains
manip_uncertain.Kp = 200; manip_uncertain.Kd = 30;   % CT/Robust gains
manip_uncertain.rho = 15;                             % Robust gain

t_span_comp = 0:0.005:10;
x0_comp = [0.2; -0.3; 0; 0];

% Backstepping with model uncertainty
[t_bs, x_bs] = ode45(@(t,x) manip_backstep_uncertain_ode(t, x, manip, manip_uncertain), ...
    t_span_comp, x0_comp);

% Computed Torque with model uncertainty
[t_ct, x_ct] = ode45(@(t,x) manip_ct_uncertain_ode(t, x, manip, manip_uncertain), ...
    t_span_comp, x0_comp);

% Robust control with model uncertainty
[t_rb, x_rb] = ode45(@(t,x) manip_robust_uncertain_ode(t, x, manip, manip_uncertain), ...
    t_span_comp, x0_comp);

% Compute errors
qd_comp = zeros(length(t_span_comp), 2);
for i = 1:length(t_span_comp)
    [qd_comp(i,:), ~, ~] = manip_desired_traj(t_span_comp(i));
end

err_bs = x_bs(:,1:2) - qd_comp;
err_ct = x_ct(:,1:2) - qd_comp;
err_rb = x_rb(:,1:2) - qd_comp;

figure('Name','Comparative Study: With Model Uncertainty','Position',[200 50 1200 700]);

subplot(2,2,1);
plot(t_bs, sqrt(sum(err_bs.^2,2)), 'b-', 'LineWidth', 1.5); hold on;
plot(t_ct, sqrt(sum(err_ct.^2,2)), 'r-', 'LineWidth', 1.5);
plot(t_rb, sqrt(sum(err_rb.^2,2)), 'g-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('||e|| (rad)');
title('Position Error Norm (20% Mass Uncertainty)');
legend('Backstepping','Computed Torque','Robust');
grid on;

subplot(2,2,2);
plot(t_bs, err_bs(:,1), 'b-', 'LineWidth', 1.2); hold on;
plot(t_ct, err_ct(:,1), 'r-', 'LineWidth', 1.2);
plot(t_rb, err_rb(:,1), 'g-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('e_1 (rad)');
title('Joint 1 Error Comparison');
legend('Backstepping','Computed Torque','Robust');
grid on;

subplot(2,2,3);
plot(t_bs, err_bs(:,2), 'b-', 'LineWidth', 1.2); hold on;
plot(t_ct, err_ct(:,2), 'r-', 'LineWidth', 1.2);
plot(t_rb, err_rb(:,2), 'g-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('e_2 (rad)');
title('Joint 2 Error Comparison');
legend('Backstepping','Computed Torque','Robust');
grid on;

subplot(2,2,4);
% Compute control energy for each method
tau_bs_all = zeros(length(t_bs), 2);
tau_ct_all = zeros(length(t_ct), 2);
tau_rb_all = zeros(length(t_rb), 2);
for i = 1:length(t_bs)
    tau_bs_all(i,:) = manip_backstep_control_uncertain(t_bs(i), x_bs(i,1:2)', x_bs(i,3:4)', manip_uncertain)';
    tau_ct_all(i,:) = manip_ct_control_uncertain(t_ct(i), x_ct(i,1:2)', x_ct(i,3:4)', manip_uncertain)';
    tau_rb_all(i,:) = manip_robust_control_uncertain(t_rb(i), x_rb(i,1:2)', x_rb(i,3:4)', manip_uncertain)';
end
energy_bs = cumtrapz(t_bs, sum(tau_bs_all.^2, 2));
energy_ct = cumtrapz(t_ct, sum(tau_ct_all.^2, 2));
energy_rb = cumtrapz(t_rb, sum(tau_rb_all.^2, 2));
plot(t_bs, energy_bs, 'b-', 'LineWidth', 1.5); hold on;
plot(t_ct, energy_ct, 'r-', 'LineWidth', 1.5);
plot(t_rb, energy_rb, 'g-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\int ||\tau||^2 dt');
title('Cumulative Control Energy');
legend('Backstepping','Computed Torque','Robust');
grid on;

sgtitle('Comparative Study: Backstepping vs CT vs Robust (20% Mass Error)', ...
    'FontSize', 14, 'FontWeight', 'bold');

fprintf('  Final errors (norm): BS=%.6f, CT=%.6f, Robust=%.6f rad\n', ...
    norm(err_bs(end,:)), norm(err_ct(end,:)), norm(err_rb(end,:)));
fprintf('  Total energy: BS=%.1f, CT=%.1f, Robust=%.1f\n', ...
    energy_bs(end), energy_ct(end), energy_rb(end));

fprintf('\n=== Simulation Complete ===\n');

%% ========================================================================
%  HELPER FUNCTIONS
%  ========================================================================

%% --- MANIPULATOR FUNCTIONS ---

function [qd, dqd, ddqd] = manip_desired_traj(t)
    % Sinusoidal reference trajectory for 2-DOF manipulator
    qd   = [0.5*sin(t); 0.5*cos(t)];
    dqd  = [0.5*cos(t); -0.5*sin(t)];
    ddqd = [-0.5*sin(t); -0.5*cos(t)];
end

function [M, C, G] = manip_dynamics_matrices(q, dq, params)
    % Full 2-DOF manipulator dynamics matrices
    % M(q)ddq + C(q,dq)dq + G(q) = tau

    m1 = params.m1; m2 = params.m2;
    l1 = params.l1; lc1 = params.lc1; lc2 = params.lc2;
    I1 = params.I1; I2 = params.I2;
    g  = params.g;

    c2 = cos(q(2)); s2 = sin(q(2));
    s1 = sin(q(1)); s12 = sin(q(1)+q(2));

    % Inertia matrix
    h = m2*l1*lc2;
    M11 = m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*c2) + I1 + I2;
    M12 = m2*(lc2^2 + l1*lc2*c2) + I2;
    M22 = m2*lc2^2 + I2;
    M = [M11, M12; M12, M22];

    % Coriolis/centrifugal matrix (Christoffel symbols)
    C11 = -h*s2*dq(2);
    C12 = -h*s2*(dq(1) + dq(2));
    C21 =  h*s2*dq(1);
    C22 =  0;
    C = [C11, C12; C21, C22];

    % Gravity vector
    G1 = (m1*lc1 + m2*l1)*g*s1 + m2*lc2*g*s12;
    G2 = m2*lc2*g*s12;
    G = [G1; G2];
end

function [M, C, G] = manip_dynamics_matrices_hat(q, dq, params)
    % Dynamics matrices with estimated (uncertain) masses
    params_hat = params;
    params_hat.m1 = params.m1_hat;
    params_hat.m2 = params.m2_hat;
    [M, C, G] = manip_dynamics_matrices(q, dq, params_hat);
end

function tau = manip_backstep_control(t, q, dq, params)
    % Backstepping controller for 2-DOF manipulator
    [qd, dqd, ddqd] = manip_desired_traj(t);

    % Step 1: Position error and virtual control
    z1 = q - qd;
    dz1 = dq - dqd;
    alpha1 = dqd - params.k1 * z1;   % Virtual control (desired velocity)

    % Step 2: Velocity error
    z2 = dq - alpha1;                  % = dz1 + k1*z1

    % Dynamics matrices
    [M, C, G] = manip_dynamics_matrices(q, dq, params);

    % Backstepping torque
    % tau = M*(ddqd - k1*dz1 - k2*z2) + C*alpha1 + G - z1
    tau = M * (ddqd - params.k1*dz1 - params.k2*z2) ...
        + C * alpha1 + G - z1;
end

function dx = manip_backstep_ode(t, x, params)
    % ODE for manipulator with backstepping control
    q  = x(1:2);
    dq = x(3:4);

    % True dynamics
    [M, C, G] = manip_dynamics_matrices(q, dq, params);

    % Controller
    tau = manip_backstep_control(t, q, dq, params);

    % State derivative
    ddq = M \ (tau - C*dq - G);
    dx = [dq; ddq];
end

% --- Uncertain manipulator controllers ---

function tau = manip_backstep_control_uncertain(t, q, dq, params)
    % Backstepping using estimated (wrong) dynamics
    [qd, dqd, ddqd] = manip_desired_traj(t);

    z1 = q - qd;
    dz1 = dq - dqd;
    alpha1 = dqd - params.k1 * z1;
    z2 = dq - alpha1;

    % Use ESTIMATED dynamics
    [M_hat, C_hat, G_hat] = manip_dynamics_matrices_hat(q, dq, params);

    tau = M_hat * (ddqd - params.k1*dz1 - params.k2*z2) ...
        + C_hat * alpha1 + G_hat - z1;
end

function tau = manip_ct_control_uncertain(t, q, dq, params)
    % Computed Torque using estimated (wrong) dynamics
    [qd, dqd, ddqd] = manip_desired_traj(t);

    e  = qd - q;
    de = dqd - dq;

    [M_hat, C_hat, G_hat] = manip_dynamics_matrices_hat(q, dq, params);

    tau = M_hat * (ddqd + params.Kd*de + params.Kp*e) + C_hat*dq + G_hat;
end

function tau = manip_robust_control_uncertain(t, q, dq, params)
    % Robust controller using estimated dynamics + robust term
    [qd, dqd, ddqd] = manip_desired_traj(t);

    e  = qd - q;
    de = dqd - dq;
    Lambda = 10 * eye(2);
    s = de + Lambda * e;

    [M_hat, C_hat, G_hat] = manip_dynamics_matrices_hat(q, dq, params);

    % Computed torque part
    dqr = dqd + Lambda*e;
    ddqr = ddqd + Lambda*de;
    tau = M_hat * ddqr + C_hat * dqr + G_hat;

    % Robust term: -rho * s / ||s||
    s_norm = norm(s);
    if s_norm > 1e-4
        tau = tau + params.rho * s / s_norm;
    end
end

function dx = manip_backstep_uncertain_ode(t, x, true_params, ctrl_params)
    q  = x(1:2);
    dq = x(3:4);
    [M, C, G] = manip_dynamics_matrices(q, dq, true_params);
    tau = manip_backstep_control_uncertain(t, q, dq, ctrl_params);
    ddq = M \ (tau - C*dq - G);
    dx = [dq; ddq];
end

function dx = manip_ct_uncertain_ode(t, x, true_params, ctrl_params)
    q  = x(1:2);
    dq = x(3:4);
    [M, C, G] = manip_dynamics_matrices(q, dq, true_params);
    tau = manip_ct_control_uncertain(t, q, dq, ctrl_params);
    ddq = M \ (tau - C*dq - G);
    dx = [dq; ddq];
end

function dx = manip_robust_uncertain_ode(t, x, true_params, ctrl_params)
    q  = x(1:2);
    dq = x(3:4);
    [M, C, G] = manip_dynamics_matrices(q, dq, true_params);
    tau = manip_robust_control_uncertain(t, q, dq, ctrl_params);
    ddq = M \ (tau - C*dq - G);
    dx = [dq; ddq];
end

%% --- MOBILE ROBOT FUNCTIONS ---

function [xr, yr, thetar, vr, wr] = mobile_reference(t)
    % Circular reference trajectory for mobile robot
    R = 5;          % Radius (m)
    omega0 = 0.3;   % Angular rate (rad/s)

    xr = R * cos(omega0 * t);
    yr = R * sin(omega0 * t);

    dxr = -R * omega0 * sin(omega0 * t);
    dyr =  R * omega0 * cos(omega0 * t);

    thetar = pi/2 + omega0 * t;  % Analytical heading (continuous, no atan2 wrap)
    vr = R * omega0;              % Constant speed for circular trajectory
    wr = omega0;                  % Constant angular rate for circular trajectory
end

function dx = mobile_backstep_ode(t, x, params)
    % ODE for unicycle mobile robot with backstepping control
    xp = x(1); yp = x(2); theta = x(3);

    % Reference
    [xr, yr, thetar, vr, wr] = mobile_reference(t);

    % Body-frame errors
    dx_e = xr - xp;
    dy_e = yr - yp;
    e1 =  cos(theta)*dx_e + sin(theta)*dy_e;
    e2 = -sin(theta)*dx_e + cos(theta)*dy_e;
    e3 = thetar - theta;
    e3 = atan2(sin(e3), cos(e3));  % Wrap to [-pi, pi]

    % Backstepping controller
    v     = vr * cos(e3) + params.k1 * e1;
    omega = wr + params.k2 * vr * e2 + params.k3 * sin(e3);

    % Unicycle kinematics
    dx = [v * cos(theta);
          v * sin(theta);
          omega];
end

%% --- QUADROTOR FUNCTIONS ---

function [pos_d, vel_d, acc_d] = quad_reference(t)
    % Helical trajectory for quadrotor
    R = 2.0;       % Radius (m)
    w0 = 0.4;      % Angular rate (rad/s)
    vz = 0.2;      % Vertical climb rate (m/s)
    z0 = 2.0;      % Initial desired altitude (m)

    pos_d = [R*cos(w0*t);
             R*sin(w0*t);
             z0 + vz*t];

    vel_d = [-R*w0*sin(w0*t);
              R*w0*cos(w0*t);
              vz];

    acc_d = [-R*w0^2*cos(w0*t);
             -R*w0^2*sin(w0*t);
              0];
end

function [T, tau_b] = quad_backstep_control(state, t, params)
    % Backstepping controller for quadrotor
    %
    % State: [x y z vx vy vz phi theta psi p q r]'
    % Outputs: T (thrust), tau_b (body torques [3x1])

    pos    = state(1:3);
    vel    = state(4:6);
    phi    = state(7);
    theta  = state(8);
    psi    = state(9);
    omega_b = state(10:12);  % [p; q; r]

    % Reference
    [pos_d, vel_d, acc_d] = quad_reference(t);
    psi_d = 0;  % Desired yaw = 0

    % ---- Outer Loop: Position Backstepping ----

    % Step 1: Position error
    z1 = pos - pos_d;
    dz1 = vel - vel_d;

    % Virtual control: desired velocity
    alpha1 = vel_d - params.kp1 * z1;

    % Step 2: Velocity error
    z2 = vel - alpha1;   % = dz1 + kp1*z1

    % Desired acceleration command
    a_des = acc_d - params.kp1 * dz1 - z1 - params.kp2 * z2 + [0; 0; params.g];

    % Extract thrust and desired angles
    a_des_norm = norm(a_des);
    if a_des_norm < 0.1
        a_des_norm = 0.1;  % Prevent division by zero
    end

    T = params.m * a_des_norm;

    % Desired body z-axis
    z_B_des = a_des / a_des_norm;

    % Desired roll and pitch from desired thrust direction
    % Using small-angle extraction from desired body z-axis
    phi_d   = asin(clamp_val(z_B_des(1)*sin(psi_d) - z_B_des(2)*cos(psi_d), -1, 1));
    theta_d = atan2(z_B_des(1)*cos(psi_d) + z_B_des(2)*sin(psi_d), z_B_des(3));

    % Clamp desired angles for safety
    phi_d   = clamp_val(phi_d, -deg2rad(35), deg2rad(35));
    theta_d = clamp_val(theta_d, -deg2rad(35), deg2rad(35));

    % ---- Inner Loop: Attitude Backstepping ----

    % Step 3: Attitude error
    Phi   = [phi; theta; psi];
    Phi_d = [phi_d; theta_d; psi_d];
    z3 = Phi - Phi_d;
    z3(3) = atan2(sin(z3(3)), cos(z3(3)));  % Wrap yaw error

    % Virtual control: desired angular rate
    alpha3 = -params.ka1 * z3;

    % Step 4: Angular rate error
    z4 = omega_b - alpha3;

    % Inertia
    J = diag([params.Ixx, params.Iyy, params.Izz]);

    % Desired angular acceleration
    % dalpha3 = -ka1 * dz3 = -ka1 * (omega_b - omega_b_d)
    %         approx -ka1 * omega_b (if omega_b_d changes slowly)
    dalpha3 = -params.ka1 * omega_b;

    % Body torques
    tau_b = J * (dalpha3 - params.ka2 * z4) ...
          + cross(omega_b, J * omega_b) - z3;

    % Clamp thrust
    T = clamp_val(T, 0, 4 * params.m * params.g);
end

function dx = quad_backstep_ode(t, x, params)
    % ODE for quadrotor with backstepping control
    %
    % State: x = [px py pz vx vy vz phi theta psi p q r]'

    pos    = x(1:3);
    vel    = x(4:6);
    phi    = x(7);
    theta  = x(8);
    psi    = x(9);
    p = x(10); q = x(11); r = x(12);

    % Control
    [T, tau_b] = quad_backstep_control(x, t, params);

    % Rotation matrix (ZYX convention)
    cphi = cos(phi); sphi = sin(phi);
    cth  = cos(theta); sth = sin(theta);
    cpsi = cos(psi); spsi = sin(psi);

    R = [cpsi*cth,  cpsi*sth*sphi - spsi*cphi,  cpsi*sth*cphi + spsi*sphi;
         spsi*cth,  spsi*sth*sphi + cpsi*cphi,  spsi*sth*cphi - cpsi*sphi;
         -sth,      cth*sphi,                    cth*cphi];

    % Translational dynamics
    gravity = [0; 0; -params.g];
    thrust_world = (T / params.m) * R * [0; 0; 1];
    dvel = gravity + thrust_world;

    % Rotational kinematics (Euler rate to body rate mapping)
    % [p; q; r] = W * [dphi; dtheta; dpsi]
    % W^{-1} gives [dphi; dtheta; dpsi] from [p; q; r]
    if abs(cth) < 1e-6
        cth_safe = sign(cth) * 1e-6;
    else
        cth_safe = cth;
    end

    dphi   = p + (sphi*sth/cth_safe)*q + (cphi*sth/cth_safe)*r;
    dtheta = cphi*q - sphi*r;
    dpsi   = (sphi/cth_safe)*q + (cphi/cth_safe)*r;

    % Rotational dynamics (Euler's equation)
    J = diag([params.Ixx, params.Iyy, params.Izz]);
    omega = [p; q; r];
    domega = J \ (tau_b - cross(omega, J*omega));

    dx = [vel; dvel; dphi; dtheta; dpsi; domega];
end

%% --- UTILITY FUNCTIONS ---

function val = clamp_val(val, lo, hi)
    val = max(lo, min(hi, val));
end
