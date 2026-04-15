%% ========================================================================
%  ROBUST CONTROL SIMULATION — Week 4: Control for Robots
%  Advanced Robotics Programme, M.Sc. Robotics & Autonomous Systems, AY 2025-26
%  ========================================================================
%  Three systems:
%    Part 1: 2-DOF Manipulator with robust computed torque
%    Part 2: Mobile Robot (unicycle) with robust tracking
%    Part 3: Quadrotor with robust position/attitude control
%  Each part: full dynamics, robust controller, disturbance injection,
%  ode45 simulation, comparison with non-robust controller.
%  ========================================================================

clear; close all; clc;
fprintf('=== Week 4: Robust Control Simulation ===\n\n');

%% ========================================================================
%  PART 1: 2-DOF MANIPULATOR — ROBUST COMPUTED TORQUE
%  ========================================================================
fprintf('--- Part 1: 2-DOF Manipulator ---\n');

% True physical parameters
m1 = 2.0;  m2 = 1.5;  l1 = 1.0;  l2 = 0.8;  g = 9.81;

% Nominal (imperfect) model parameters — 10-15% error
m1_hat = 1.8;  m2_hat = 1.3;  l1_hat = 1.0;  l2_hat = 0.8;

% Controller gains
Kp_m = diag([100, 100]);
Kd_m = diag([40, 40]);
Lambda_m = diag([10, 10]);

% Robust controller parameters
rho0_robust = 15.0;    % uncertainty bound for robust controller
epsilon_m   = 0.01;    % boundary layer thickness
rho0_none   = 0.0;     % NO robust term (for comparison)

% Desired trajectory (smooth sinusoidal)
qd_fun   = @(t) [sin(t); 0.5*cos(t)];
qd_d_fun = @(t) [cos(t); -0.5*sin(t)];
qd_dd_fun= @(t) [-sin(t); -0.5*cos(t)];

% Disturbance: bounded sinusoidal
d_max_m = 5.0;
dist_manip = @(t) d_max_m * [sin(3*t); cos(2*t)];

% Initial conditions
x0_m = [0.5; -0.3; 0; 0];
tspan_m = [0 10];

% --- Simulate: ROBUST controller ---
odefun_robust = @(t, x) manip_ode(t, x, m1, m2, l1, l2, g, ...
    m1_hat, m2_hat, l1_hat, l2_hat, ...
    Kd_m, Lambda_m, rho0_robust, epsilon_m, ...
    qd_fun, qd_d_fun, qd_dd_fun, dist_manip);
[t_rob, X_rob] = ode45(odefun_robust, tspan_m, x0_m);

% --- Simulate: NON-ROBUST controller (computed torque only) ---
odefun_norob = @(t, x) manip_ode(t, x, m1, m2, l1, l2, g, ...
    m1_hat, m2_hat, l1_hat, l2_hat, ...
    Kd_m, Lambda_m, rho0_none, epsilon_m, ...
    qd_fun, qd_d_fun, qd_dd_fun, dist_manip);
[t_nrob, X_nrob] = ode45(odefun_norob, tspan_m, x0_m);

% Compute tracking errors
e_rob  = zeros(length(t_rob), 2);
e_nrob = zeros(length(t_nrob), 2);
for i = 1:length(t_rob)
    qd_val = qd_fun(t_rob(i));
    e_rob(i,:) = (X_rob(i,1:2) - qd_val')';
end
for i = 1:length(t_nrob)
    qd_val = qd_fun(t_nrob(i));
    e_nrob(i,:) = (X_nrob(i,1:2) - qd_val')';
end

fprintf('  Robust:     max|e1| = %.4f rad, max|e2| = %.4f rad\n', ...
    max(abs(e_rob(end-100:end,1))), max(abs(e_rob(end-100:end,2))));
fprintf('  Non-robust: max|e1| = %.4f rad, max|e2| = %.4f rad\n', ...
    max(abs(e_nrob(end-100:end,1))), max(abs(e_nrob(end-100:end,2))));

% --- Plots: Manipulator ---
figure('Name','Part 1: Manipulator - Tracking Error Comparison','Position',[50 400 1200 500]);

subplot(2,2,1);
plot(t_rob, e_rob(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_nrob, e_nrob(:,1), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('e_1 [rad]');
title('Joint 1 Tracking Error');
legend('Robust','Non-Robust','Location','best');
grid on;

subplot(2,2,2);
plot(t_rob, e_rob(:,2), 'b', 'LineWidth', 1.5); hold on;
plot(t_nrob, e_nrob(:,2), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('e_2 [rad]');
title('Joint 2 Tracking Error');
legend('Robust','Non-Robust','Location','best');
grid on;

subplot(2,2,3);
plot(t_rob, X_rob(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_rob, arrayfun(@(t) sin(t), t_rob), 'k--', 'LineWidth', 1);
xlabel('Time [s]'); ylabel('q_1 [rad]');
title('Joint 1: Robust Tracking');
legend('Actual','Desired','Location','best');
grid on;

subplot(2,2,4);
plot(t_rob, X_rob(:,2), 'b', 'LineWidth', 1.5); hold on;
plot(t_rob, arrayfun(@(t) 0.5*cos(t), t_rob), 'k--', 'LineWidth', 1);
xlabel('Time [s]'); ylabel('q_2 [rad]');
title('Joint 2: Robust Tracking');
legend('Actual','Desired','Location','best');
grid on;

sgtitle('Part 1: 2-DOF Manipulator — Robust vs Non-Robust Control');

% Disturbance plot
figure('Name','Part 1: Disturbance Signal','Position',[50 50 600 300]);
tt_d = linspace(0, 10, 500);
d_vals = zeros(2, length(tt_d));
for i = 1:length(tt_d)
    d_vals(:,i) = dist_manip(tt_d(i));
end
plot(tt_d, d_vals(1,:), 'b', tt_d, d_vals(2,:), 'r', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Disturbance [Nm]');
title('Applied Disturbance d(t)'); legend('d_1','d_2');
grid on;


%% ========================================================================
%  PART 2: MOBILE ROBOT — ROBUST UNICYCLE TRACKING
%  ========================================================================
fprintf('\n--- Part 2: Mobile Robot (Unicycle) ---\n');

% Reference trajectory: figure-eight
R_mob = 2;  w_ref = 0.5;
xr_fun   = @(t)  R_mob * sin(w_ref*t);
yr_fun   = @(t)  R_mob * sin(2*w_ref*t) * 0.5;
dxr_fun  = @(t)  R_mob * w_ref * cos(w_ref*t);
dyr_fun  = @(t)  R_mob * w_ref * cos(2*w_ref*t);
thr_fun  = @(t)  atan2(dyr_fun(t), dxr_fun(t));
vr_fun   = @(t)  sqrt(dxr_fun(t)^2 + dyr_fun(t)^2);

% Compute reference omega numerically (finite difference)
dt_num = 1e-5;
wr_fun = @(t) (thr_fun(t+dt_num) - thr_fun(t-dt_num)) / (2*dt_num);

% Controller gains
k1_mob = 5;  k2_mob = 3;  k3_mob = 5;

% Robust parameters
rho_v_mob = 0.5;   rho_w_mob = 0.3;
eps_v_mob = 0.01;  eps_w_mob = 0.01;

% Disturbance: wheel slip + terrain
d_mob_fun = @(t) [0.3*sin(5*t); 0.3*cos(3*t); 0.2*sin(7*t)];

% Initial conditions
x0_mob = [3; 0; 0];  % start away from reference
tspan_mob = [0 25];

% --- Simulate: ROBUST ---
odefun_mob_rob = @(t, x) mobile_ode(t, x, xr_fun, yr_fun, thr_fun, ...
    vr_fun, wr_fun, k1_mob, k2_mob, k3_mob, ...
    rho_v_mob, rho_w_mob, eps_v_mob, eps_w_mob, d_mob_fun);
[t_mr, X_mr] = ode45(odefun_mob_rob, tspan_mob, x0_mob);

% --- Simulate: NON-ROBUST (rho = 0) ---
odefun_mob_nrob = @(t, x) mobile_ode(t, x, xr_fun, yr_fun, thr_fun, ...
    vr_fun, wr_fun, k1_mob, k2_mob, k3_mob, ...
    0, 0, eps_v_mob, eps_w_mob, d_mob_fun);
[t_mnr, X_mnr] = ode45(odefun_mob_nrob, tspan_mob, x0_mob);

% Position tracking errors
err_rob_mob  = zeros(length(t_mr), 1);
err_nrob_mob = zeros(length(t_mnr), 1);
for i = 1:length(t_mr)
    err_rob_mob(i) = sqrt((X_mr(i,1)-xr_fun(t_mr(i)))^2 + ...
                          (X_mr(i,2)-yr_fun(t_mr(i)))^2);
end
for i = 1:length(t_mnr)
    err_nrob_mob(i) = sqrt((X_mnr(i,1)-xr_fun(t_mnr(i)))^2 + ...
                           (X_mnr(i,2)-yr_fun(t_mnr(i)))^2);
end

fprintf('  Robust:     mean pos error = %.4f m\n', mean(err_rob_mob(end-100:end)));
fprintf('  Non-robust: mean pos error = %.4f m\n', mean(err_nrob_mob(end-100:end)));

% --- Plots: Mobile Robot ---
figure('Name','Part 2: Mobile Robot - Trajectory Comparison','Position',[100 350 1200 500]);

subplot(1,2,1);
tt_ref = linspace(0, 25, 1000);
xr_vals = arrayfun(xr_fun, tt_ref);
yr_vals = arrayfun(yr_fun, tt_ref);
plot(xr_vals, yr_vals, 'k--', 'LineWidth', 2); hold on;
plot(X_mr(:,1), X_mr(:,2), 'b', 'LineWidth', 1.5);
plot(X_mnr(:,1), X_mnr(:,2), 'r:', 'LineWidth', 1.5);
plot(x0_mob(1), x0_mob(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('x [m]'); ylabel('y [m]');
title('Trajectory: Reference vs Robust vs Non-Robust');
legend('Reference','Robust','Non-Robust','Start','Location','best');
grid on; axis equal;

subplot(1,2,2);
plot(t_mr, err_rob_mob, 'b', 'LineWidth', 1.5); hold on;
plot(t_mnr, err_nrob_mob, 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Position Error [m]');
title('Position Tracking Error');
legend('Robust','Non-Robust','Location','best');
grid on;

sgtitle('Part 2: Mobile Robot — Robust vs Non-Robust Tracking');


%% ========================================================================
%  PART 3: QUADROTOR — ROBUST CONTROL WITH WIND
%  ========================================================================
fprintf('\n--- Part 3: Quadrotor Drone ---\n');

% Drone parameters
m_d = 1.5;  Ixx = 0.03;  Iyy = 0.03;  Izz = 0.05;

% Position controller gains
kd_z = 15;  lam_z = 5;   rho_z = 8;    eps_z = 0.01;
kd_x = 10;  lam_x = 4;   rho_x = 5;    eps_x = 0.01;
kd_y = 10;  lam_y = 4;   rho_y = 5;    eps_y = 0.01;

% Attitude controller gains
kd_phi = 20;  lam_phi = 8;  rho_phi = 1;    eps_phi = 0.005;
kd_th  = 20;  lam_th  = 8;  rho_th  = 1;    eps_th  = 0.005;
kd_psi = 10;  lam_psi = 5;  rho_psi = 0.5;  eps_psi = 0.005;

% Desired trajectory: helix
pd_fun  = @(t) [2*sin(0.3*t); 2*cos(0.3*t); 1 + 0.5*sin(0.2*t)];
dpd_fun = @(t) [0.6*cos(0.3*t); -0.6*sin(0.3*t); 0.1*cos(0.2*t)];
ddpd_fun= @(t) [-0.18*sin(0.3*t); -0.18*cos(0.3*t); -0.02*sin(0.2*t)];

% Wind disturbance
f_wind_fun   = @(t) [3*sin(2*t+1); 2*cos(1.5*t); 1.5*sin(t)];
tau_wind_fun = @(t) [0.3*sin(4*t); 0.2*cos(3*t); 0.1*sin(2*t)];

% Gain vectors for passing to ODE
gains_pos = [kd_z, lam_z, rho_z, eps_z, ...
             kd_x, lam_x, rho_x, eps_x, ...
             kd_y, lam_y, rho_y, eps_y];
gains_att = [kd_phi, lam_phi, rho_phi, eps_phi, ...
             kd_th,  lam_th,  rho_th,  eps_th, ...
             kd_psi, lam_psi, rho_psi, eps_psi];

% Initial conditions: [x y z dx dy dz phi theta psi dphi dtheta dpsi]
% Start at the reference trajectory to avoid a huge initial transient
x0_d = zeros(12, 1);
x0_d(1:3) = pd_fun(0);       % position = [0; 2; 1]
x0_d(4:6) = dpd_fun(0);      % velocity = [0.6; 0; 0.1]
tspan_d = [0 25];

% --- Simulate: ROBUST ---
odefun_d_rob = @(t, x) drone_ode(t, x, m_d, g, Ixx, Iyy, Izz, ...
    gains_pos, gains_att, pd_fun, dpd_fun, ddpd_fun, ...
    f_wind_fun, tau_wind_fun);
opts = odeset('MaxStep', 0.05, 'RelTol', 1e-6, 'AbsTol', 1e-8);
[t_dr, X_dr] = ode45(odefun_d_rob, tspan_d, x0_d, opts);

% --- Simulate: NON-ROBUST (rho = 0 for all axes) ---
gains_pos_nr = [kd_z, lam_z, 0, eps_z, ...
                kd_x, lam_x, 0, eps_x, ...
                kd_y, lam_y, 0, eps_y];
gains_att_nr = [kd_phi, lam_phi, 0, eps_phi, ...
                kd_th,  lam_th,  0, eps_th, ...
                kd_psi, lam_psi, 0, eps_psi];

odefun_d_nrob = @(t, x) drone_ode(t, x, m_d, g, Ixx, Iyy, Izz, ...
    gains_pos_nr, gains_att_nr, pd_fun, dpd_fun, ddpd_fun, ...
    f_wind_fun, tau_wind_fun);
[t_dnr, X_dnr] = ode45(odefun_d_nrob, tspan_d, x0_d, opts);

% Position errors
err_dr  = zeros(length(t_dr), 1);
err_dnr = zeros(length(t_dnr), 1);
for i = 1:length(t_dr)
    pd_val = pd_fun(t_dr(i));
    err_dr(i) = norm(X_dr(i,1:3)' - pd_val);
end
for i = 1:length(t_dnr)
    pd_val = pd_fun(t_dnr(i));
    err_dnr(i) = norm(X_dnr(i,1:3)' - pd_val);
end

fprintf('  Robust:     mean 3D pos error = %.4f m\n', mean(err_dr(end-100:end)));
fprintf('  Non-robust: mean 3D pos error = %.4f m\n', mean(err_dnr(end-100:end)));

% --- Plots: Quadrotor ---
figure('Name','Part 3: Quadrotor - 3D Trajectory','Position',[150 300 1200 500]);

subplot(1,2,1);
% Reference trajectory
tt_ref_d = linspace(0, 25, 1000);
ref_d = zeros(3, length(tt_ref_d));
for i = 1:length(tt_ref_d)
    ref_d(:,i) = pd_fun(tt_ref_d(i));
end
plot3(ref_d(1,:), ref_d(2,:), ref_d(3,:), 'k--', 'LineWidth', 2); hold on;
plot3(X_dr(:,1), X_dr(:,2), X_dr(:,3), 'b', 'LineWidth', 1.5);
plot3(X_dnr(:,1), X_dnr(:,2), X_dnr(:,3), 'r:', 'LineWidth', 1.5);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('3D Trajectory'); legend('Reference','Robust','Non-Robust');
grid on; view(30, 25);

subplot(1,2,2);
plot(t_dr, err_dr, 'b', 'LineWidth', 1.5); hold on;
plot(t_dnr, err_dnr, 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('3D Position Error [m]');
title('Position Tracking Error');
legend('Robust','Non-Robust','Location','best');
grid on;

sgtitle('Part 3: Quadrotor — Robust vs Non-Robust under Wind Disturbance');

% Per-axis comparison
figure('Name','Part 3: Quadrotor - Per-Axis Tracking','Position',[200 250 1200 700]);

axis_labels = {'x','y','z'};
for ax = 1:3
    subplot(3,2,2*ax-1);
    plot(t_dr, X_dr(:,ax), 'b', 'LineWidth', 1.5); hold on;
    ref_ax = zeros(length(t_dr),1);
    for i = 1:length(t_dr)
        pv = pd_fun(t_dr(i));
        ref_ax(i) = pv(ax);
    end
    plot(t_dr, ref_ax, 'k--', 'LineWidth', 1);
    xlabel('Time [s]'); ylabel([axis_labels{ax} ' [m]']);
    title(['Robust: ' axis_labels{ax} '-axis tracking']);
    legend('Actual','Desired','Location','best'); grid on;

    subplot(3,2,2*ax);
    err_ax_r  = X_dr(:,ax) - ref_ax;
    ref_ax_nr = zeros(length(t_dnr),1);
    for i = 1:length(t_dnr)
        pv = pd_fun(t_dnr(i));
        ref_ax_nr(i) = pv(ax);
    end
    err_ax_nr = X_dnr(:,ax) - ref_ax_nr;
    plot(t_dr, err_ax_r, 'b', 'LineWidth', 1.5); hold on;
    plot(t_dnr, err_ax_nr, 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel(['Error ' axis_labels{ax} ' [m]']);
    title([axis_labels{ax} '-axis error comparison']);
    legend('Robust','Non-Robust','Location','best'); grid on;
end
sgtitle('Part 3: Quadrotor — Per-Axis Position Tracking');

% Wind disturbance plot
figure('Name','Part 3: Wind Disturbance','Position',[250 200 600 400]);
tt_w = linspace(0, 25, 500);
fw_vals = zeros(3, length(tt_w));
for i = 1:length(tt_w)
    fw_vals(:,i) = f_wind_fun(tt_w(i));
end
plot(tt_w, fw_vals(1,:), 'b', tt_w, fw_vals(2,:), 'r', ...
     tt_w, fw_vals(3,:), 'g', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Force [N]');
title('Wind Disturbance Forces');
legend('f_{wind,x}','f_{wind,y}','f_{wind,z}');
grid on;


%% ========================================================================
%  SUMMARY
%  ========================================================================
fprintf('\n=== SIMULATION COMPLETE ===\n');
fprintf('All figures generated. Compare robust vs non-robust across systems.\n');


%% ========================================================================
%  FUNCTION DEFINITIONS
%  ========================================================================

% -------------------------------------------------------------------------
% MANIPULATOR ODE
% -------------------------------------------------------------------------
function dx = manip_ode(t, x, m1, m2, l1, l2, g, ...
    m1h, m2h, l1h, l2h, Kd, Lambda, rho0, epsilon, ...
    qd_fun, qd_d_fun, qd_dd_fun, dist_fun)

    q  = x(1:2);
    dq = x(3:4);
    q1 = q(1); q2 = q(2);

    % TRUE dynamics matrices
    M11 = (m1+m2)*l1^2 + m2*l2^2 + 2*m2*l1*l2*cos(q2);
    M12 = m2*l2^2 + m2*l1*l2*cos(q2);
    M22 = m2*l2^2;
    M = [M11, M12; M12, M22];

    h = m2*l1*l2*sin(q2);
    C = [-h*dq(2), -h*(dq(1)+dq(2));
          h*dq(1),  0];

    G_vec = [(m1+m2)*g*l1*cos(q1) + m2*g*l2*cos(q1+q2);
              m2*g*l2*cos(q1+q2)];

    % NOMINAL (hat) dynamics matrices
    M11h = (m1h+m2h)*l1h^2 + m2h*l2h^2 + 2*m2h*l1h*l2h*cos(q2);
    M12h = m2h*l2h^2 + m2h*l1h*l2h*cos(q2);
    M22h = m2h*l2h^2;
    Mh = [M11h, M12h; M12h, M22h];

    hh = m2h*l1h*l2h*sin(q2);
    Ch = [-hh*dq(2), -hh*(dq(1)+dq(2));
           hh*dq(1),  0];

    Gh = [(m1h+m2h)*g*l1h*cos(q1) + m2h*g*l2h*cos(q1+q2);
           m2h*g*l2h*cos(q1+q2)];

    % Desired trajectory
    qd    = qd_fun(t);
    qd_d  = qd_d_fun(t);
    qd_dd = qd_dd_fun(t);

    % Errors
    e  = q - qd;
    ed = dq - qd_d;

    % Sliding variable
    s = ed + Lambda * e;

    % Reference velocity and acceleration
    dqr  = qd_d - Lambda * e;
    ddqr = qd_dd - Lambda * ed;

    % Robust term (smooth approximation)
    s_norm = norm(s);
    if rho0 > 0
        tau_robust = -rho0 * s / (s_norm + epsilon);
    else
        tau_robust = [0; 0];
    end

    % Control law: nominal computed torque + robust term
    tau = Mh * ddqr + Ch * dqr + Gh - Kd * s + tau_robust;

    % Disturbance
    d = dist_fun(t);

    % True dynamics: M*ddq + C*dq + G = tau + d
    ddq = M \ (tau + d - C*dq - G_vec);

    dx = [dq; ddq];
end

% -------------------------------------------------------------------------
% MOBILE ROBOT ODE (Unicycle with disturbance)
% -------------------------------------------------------------------------
function dx = mobile_ode(t, x, xr_fun, yr_fun, thr_fun, ...
    vr_fun, wr_fun, k1, k2, k3, ...
    rho_v, rho_w, eps_v, eps_w, dist_fun)

    px = x(1);  py = x(2);  th = x(3);

    % Reference
    xr_val  = xr_fun(t);
    yr_val  = yr_fun(t);
    thr_val = thr_fun(t);
    vr_val  = vr_fun(t);
    wr_val  = wr_fun(t);

    % Body-frame tracking error
    dx_err = xr_val - px;
    dy_err = yr_val - py;
    ex =  cos(th)*dx_err + sin(th)*dy_err;
    ey = -sin(th)*dx_err + cos(th)*dy_err;
    eth = thr_val - th;
    eth = atan2(sin(eth), cos(eth));  % wrap to [-pi, pi]

    % Nominal tracking controller
    v_nom = vr_val * cos(eth) + k1 * ex;
    w_nom = wr_val + vr_val * (k2 * ey + k3 * sin(eth));

    % Robust augmentation
    v = v_nom + rho_v * ex / (abs(ex) + eps_v);
    w = w_nom + rho_w * eth / (abs(eth) + eps_w);

    % Disturbance
    d = dist_fun(t);

    % Disturbed unicycle kinematics
    dx = [v*cos(th) + d(1);
          v*sin(th) + d(2);
          w          + d(3)];
end

% -------------------------------------------------------------------------
% QUADROTOR DRONE ODE (Robust position + attitude control)
% -------------------------------------------------------------------------
function dx = drone_ode(t, x, m, g, Ixx, Iyy, Izz, ...
    gains_pos, gains_att, pd_fun, dpd_fun, ddpd_fun, ...
    f_wind_fun, tau_wind_fun)

    % State extraction
    pos   = x(1:3);   % [x; y; z]
    vel   = x(4:6);   % [dx; dy; dz]
    phi   = x(7);     % roll
    theta = x(8);     % pitch
    psi   = x(9);     % yaw
    omega = x(10:12); % [dphi; dtheta; dpsi]

    % Unpack position gains
    kd_z  = gains_pos(1);  lam_z = gains_pos(2);  rho_z = gains_pos(3);  eps_z = gains_pos(4);
    kd_x  = gains_pos(5);  lam_x = gains_pos(6);  rho_x = gains_pos(7);  eps_x = gains_pos(8);
    kd_y  = gains_pos(9);  lam_y = gains_pos(10); rho_y = gains_pos(11); eps_y = gains_pos(12);

    % Unpack attitude gains
    kd_phi  = gains_att(1);  lam_phi = gains_att(2);  rho_phi = gains_att(3);  eps_phi = gains_att(4);
    kd_th   = gains_att(5);  lam_th  = gains_att(6);  rho_th  = gains_att(7);  eps_th  = gains_att(8);
    kd_psi  = gains_att(9);  lam_psi = gains_att(10); rho_psi = gains_att(11); eps_psi = gains_att(12);

    % Desired trajectory
    p_d   = pd_fun(t);
    dp_d  = dpd_fun(t);
    ddp_d = ddpd_fun(t);
    psi_d  = 0;
    dpsi_d = 0;
    ddpsi_d = 0;

    % Position errors
    ep  = p_d - pos;
    dep = dp_d - vel;

    % --- Z-axis (altitude) robust controller ---
    sz = dep(3) + lam_z * ep(3);
    T = m * (ddp_d(3) + lam_z*dep(3) + kd_z*sz) + m*g ...
        + rho_z * sz / (abs(sz) + eps_z);
    T = max(T, 0.1);  % thrust must be positive

    % --- X-axis robust controller -> desired theta ---
    sx = dep(1) + lam_x * ep(1);
    ax_cmd = ddp_d(1) + lam_x*dep(1) + kd_x*sx + rho_x*sx/(abs(sx)+eps_x);
    theta_d = m * ax_cmd / T;
    theta_d = max(min(theta_d, 0.5), -0.5);  % saturate to ±0.5 rad

    % --- Y-axis robust controller -> desired phi ---
    sy = dep(2) + lam_y * ep(2);
    ay_cmd = ddp_d(2) + lam_y*dep(2) + kd_y*sy + rho_y*sy/(abs(sy)+eps_y);
    phi_d = -m * ay_cmd / T;
    phi_d = max(min(phi_d, 0.5), -0.5);

    % --- Attitude robust controllers ---
    % Roll (phi)
    e_phi  = phi_d - phi;
    de_phi = 0 - omega(1);  % dphi_d ≈ 0 for near-hover
    s_phi  = de_phi + lam_phi * e_phi;
    tau_phi = Ixx * (lam_phi*de_phi + kd_phi*s_phi) ...
              + rho_phi * s_phi / (abs(s_phi) + eps_phi);

    % Pitch (theta)
    e_th  = theta_d - theta;
    de_th = 0 - omega(2);
    s_th  = de_th + lam_th * e_th;
    tau_th = Iyy * (lam_th*de_th + kd_th*s_th) ...
             + rho_th * s_th / (abs(s_th) + eps_th);

    % Yaw (psi)
    e_psi  = psi_d - psi;
    de_psi = dpsi_d - omega(3);
    s_psi  = de_psi + lam_psi * e_psi;
    tau_psi = Izz * (lam_psi*de_psi + kd_psi*s_psi) ...
              + rho_psi * s_psi / (abs(s_psi) + eps_psi);

    % Wind disturbances
    fw = f_wind_fun(t);
    tw = tau_wind_fun(t);

    % Rotation matrix (ZYX Euler angles)
    cphi = cos(phi); sphi = sin(phi);
    cth  = cos(theta); sth = sin(theta);
    cpsi = cos(psi); spsi = sin(psi);

    R = [cpsi*cth, cpsi*sth*sphi - spsi*cphi, cpsi*sth*cphi + spsi*sphi;
         spsi*cth, spsi*sth*sphi + cpsi*cphi, spsi*sth*cphi - cpsi*sphi;
         -sth,     cth*sphi,                  cth*cphi];

    % Translational dynamics
    thrust_body = [0; 0; T];
    accel = [0; 0; -g] + R * thrust_body / m + fw / m;

    % Rotational dynamics (simplified Euler equations)
    domega = [(tau_phi + tw(1) - (Izz - Iyy)*omega(2)*omega(3)) / Ixx;
              (tau_th  + tw(2) - (Ixx - Izz)*omega(1)*omega(3)) / Iyy;
              (tau_psi + tw(3) - (Iyy - Ixx)*omega(1)*omega(2)) / Izz];

    dx = [vel; accel; omega; domega];
end
