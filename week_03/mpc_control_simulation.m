% =========================================================================
% MPC Control Simulation for Three Robotic Systems
% Week 2: Model Predictive Control — Control for Robots
% Advanced Robotics Programme, AY 2025-26
%
% This script simulates MPC for:
%   1. 2-DOF Robot Manipulator (sinusoidal joint trajectory)
%   2. Differential-Drive Mobile Robot (circular path)
%   3. Quadrotor Drone (helical trajectory)
%
% Each system also includes a PD-control baseline for comparison.
% =========================================================================

clear all; close all; clc;

fprintf('=== Week 2: Model Predictive Control Simulation ===\n\n');

%% ========================================================================
%  1. ROBOT MANIPULATOR — MPC vs PD Control
% ========================================================================
fprintf('--- System 1: 2-DOF Robot Manipulator ---\n');

% Physical parameters (same as Week 1)
manipulator_params.m1 = 1;      % Mass of link 1 (kg)
manipulator_params.m2 = 1;      % Mass of link 2 (kg)
manipulator_params.l1 = 1;      % Length of link 1 (m)
manipulator_params.l2 = 1;      % Length of link 2 (m)
manipulator_params.g  = 9.81;   % Gravitational acceleration (m/s^2)

% Simulation parameters
dt_manip = 0.01;                % Sampling time (s)
t_span_manip = 0:dt_manip:20;  % 20 seconds simulation
n_manip = 4;                    % State dimension [q1, q2, dq1, dq2]
m_manip = 2;                    % Control dimension [tau1, tau2]

% --- Linearisation around operating point q0 = [0.1, 0.1] ---
q0_lin = [0.1; 0.1];
c2 = cos(q0_lin(2));
s2 = sin(q0_lin(2));
p = manipulator_params;

M11 = p.m1*p.l1^2 + p.m2*(p.l1^2 + p.l2^2 + 2*p.l1*p.l2*c2);
M12 = p.m2*(p.l2^2 + p.l1*p.l2*c2);
M22 = p.m2*p.l2^2;
M0 = [M11, M12; M12, M22];
M0_inv = inv(M0);

% Continuous-time linearised system
Ac_manip = [zeros(2), eye(2); zeros(2), zeros(2)];
Bc_manip = [zeros(2); M0_inv];

% Discretise using Euler method
Ad_manip = eye(n_manip) + Ac_manip * dt_manip;
Bd_manip = Bc_manip * dt_manip;

% --- MPC Parameters ---
N_manip = 20;                           % Prediction horizon
Q_manip = diag([100, 100, 1, 1]);      % State weight (position >> velocity)
R_manip = diag([0.01, 0.01]);          % Control weight (allow large torques)

% Build prediction matrices (A_cal and B_cal)
A_cal_manip = zeros(n_manip * N_manip, n_manip);
B_cal_manip = zeros(n_manip * N_manip, m_manip * N_manip);

for i = 1:N_manip
    A_cal_manip((i-1)*n_manip+1 : i*n_manip, :) = Ad_manip^i;
    for j = 1:i
        B_cal_manip((i-1)*n_manip+1 : i*n_manip, ...
                    (j-1)*m_manip+1 : j*m_manip) = Ad_manip^(i-j) * Bd_manip;
    end
end

% Block-diagonal weight matrices
Q_N_manip = kron(eye(N_manip), Q_manip);
R_N_manip = kron(eye(N_manip), R_manip);

% Hessian and precomputed matrices (constant for LTI system)
H_manip = B_cal_manip' * Q_N_manip * B_cal_manip + R_N_manip;
H_inv_manip = inv(H_manip);
F_manip = B_cal_manip' * Q_N_manip;

fprintf('  Prediction matrices built: A_cal is %dx%d, B_cal is %dx%d\n', ...
    size(A_cal_manip,1), size(A_cal_manip,2), ...
    size(B_cal_manip,1), size(B_cal_manip,2));
fprintf('  Hessian H is %dx%d\n', size(H_manip,1), size(H_manip,2));

% --- Desired trajectory functions ---
qd_func   = @(t) [0.5*sin(t); 0.5*cos(t)];
dqd_func  = @(t) [0.5*cos(t); -0.5*sin(t)];
ddqd_func = @(t) [-0.5*sin(t); -0.5*cos(t)];

% === MPC Simulation ===
x_mpc_manip = [0; 0; 0; 0];            % Initial state
x_hist_mpc_manip = zeros(n_manip, length(t_span_manip));
u_hist_mpc_manip = zeros(m_manip, length(t_span_manip));
x_hist_mpc_manip(:,1) = x_mpc_manip;

tic;
for k = 1:length(t_span_manip)-1
    t = t_span_manip(k);

    % Build reference vector over prediction horizon
    x_ref_vec = zeros(n_manip * N_manip, 1);
    for i = 1:N_manip
        t_future = t + i * dt_manip;
        qd_f = qd_func(t_future);
        dqd_f = dqd_func(t_future);
        x_ref_vec((i-1)*n_manip+1 : i*n_manip) = [qd_f; dqd_f];
    end

    % Compute optimal control sequence
    E_manip = x_ref_vec - A_cal_manip * x_mpc_manip;
    U_star = H_inv_manip * F_manip * E_manip;

    % Extract first control input (receding horizon)
    u_mpc = U_star(1:m_manip);

    % Add gravity and Coriolis feedforward compensation
    % (The linearised model is a double integrator and ignores gravity/Coriolis.
    %  We add feedforward so the MPC handles the linear part while the
    %  feedforward cancels the known nonlinearities.)
    q_curr = x_mpc_manip(1:2);
    dq_curr = x_mpc_manip(3:4);
    [~, C_ff, G_ff] = build_manipulator_model(q_curr, dq_curr, manipulator_params);
    u_mpc = u_mpc + G_ff + C_ff * dq_curr;

    % Enforce torque constraints: |tau| <= 50 N*m
    u_mpc = max(min(u_mpc, 50), -50);

    % Simulate true nonlinear dynamics (Euler integration step)
    [M_true, C_true, G_true] = build_manipulator_model(q_curr, dq_curr, manipulator_params);
    ddq = M_true \ (u_mpc - C_true * dq_curr - G_true);
    x_mpc_manip = x_mpc_manip + [dq_curr; ddq] * dt_manip;

    x_hist_mpc_manip(:,k+1) = x_mpc_manip;
    u_hist_mpc_manip(:,k) = u_mpc;
end
time_mpc_manip = toc;
fprintf('  MPC simulation time: %.3f s\n', time_mpc_manip);

% === PD Control Simulation (baseline) ===
Kp_pd = diag([50, 50]);
Kd_pd = diag([20, 20]);

x_pd_manip = [0; 0; 0; 0];
x_hist_pd_manip = zeros(n_manip, length(t_span_manip));
u_hist_pd_manip = zeros(m_manip, length(t_span_manip));
x_hist_pd_manip(:,1) = x_pd_manip;

tic;
for k = 1:length(t_span_manip)-1
    t = t_span_manip(k);

    q_des = qd_func(t);
    dq_des = dqd_func(t);

    q_curr = x_pd_manip(1:2);
    dq_curr = x_pd_manip(3:4);

    e = q_des - q_curr;
    de = dq_des - dq_curr;

    % PD control (no model compensation)
    u_pd = Kp_pd * e + Kd_pd * de;

    % Enforce same torque constraints
    u_pd = max(min(u_pd, 50), -50);

    % Simulate true nonlinear dynamics
    [M_true, C_true, G_true] = build_manipulator_model(q_curr, dq_curr, manipulator_params);
    ddq = M_true \ (u_pd - C_true * dq_curr - G_true);
    x_pd_manip = x_pd_manip + [dq_curr; ddq] * dt_manip;

    x_hist_pd_manip(:,k+1) = x_pd_manip;
    u_hist_pd_manip(:,k) = u_pd;
end
time_pd_manip = toc;
fprintf('  PD simulation time:  %.3f s\n', time_pd_manip);

% Compute desired trajectory for plotting
qd_vals_manip = zeros(2, length(t_span_manip));
for i = 1:length(t_span_manip)
    qd_vals_manip(:,i) = qd_func(t_span_manip(i));
end

%% ========================================================================
%  2. DIFFERENTIAL-DRIVE MOBILE ROBOT — MPC vs PD Control
% ========================================================================
fprintf('\n--- System 2: Differential-Drive Mobile Robot ---\n');

% Simulation parameters
dt_mob = 0.05;                  % Sampling time (s)
t_span_mob = 0:dt_mob:20;      % 20 seconds simulation
n_mob = 3;                      % State dimension [px, py, theta]
m_mob = 2;                      % Control dimension [v, omega]

% Reference trajectory parameters (circular path)
R_circle = 5;                   % Circle radius (m)
omega_circle = 0.5;             % Angular frequency (rad/s)
v_ref = R_circle * omega_circle; % Reference linear velocity (m/s)

% Linearised error-coordinate system
% Error state: e = [e_x; e_y; e_theta] in the REFERENCE body frame.
% Control: delta_u = [v - v_ref; omega - omega_ref].
% Unlike linearising around theta=0 (which fails when theta grows),
% this formulation is valid for ALL reference headings.
omega_ref = omega_circle;
Ac_mob = [0, omega_ref, 0; -omega_ref, 0, v_ref; 0, 0, 0];
Bc_mob = [1, 0; 0, 0; 0, 1];

% Discretise
Ad_mob = eye(n_mob) + Ac_mob * dt_mob;
Bd_mob = Bc_mob * dt_mob;

% MPC parameters
N_mob = 15;                     % Prediction horizon
Q_mob = diag([50, 50, 10]);    % State weight
R_mob_w = diag([0.1, 0.1]);    % Control weight

% Build prediction matrices
A_cal_mob = zeros(n_mob * N_mob, n_mob);
B_cal_mob = zeros(n_mob * N_mob, m_mob * N_mob);

for i = 1:N_mob
    A_cal_mob((i-1)*n_mob+1 : i*n_mob, :) = Ad_mob^i;
    for j = 1:i
        B_cal_mob((i-1)*n_mob+1 : i*n_mob, ...
                  (j-1)*m_mob+1 : j*m_mob) = Ad_mob^(i-j) * Bd_mob;
    end
end

Q_N_mob = kron(eye(N_mob), Q_mob);
R_N_mob = kron(eye(N_mob), R_mob_w);

H_mob = B_cal_mob' * Q_N_mob * B_cal_mob + R_N_mob;
H_inv_mob = inv(H_mob);
F_mob = B_cal_mob' * Q_N_mob;

fprintf('  Prediction matrices built: A_cal is %dx%d, B_cal is %dx%d\n', ...
    size(A_cal_mob,1), size(A_cal_mob,2), ...
    size(B_cal_mob,1), size(B_cal_mob,2));

% Reference trajectory functions
mob_ref_x     = @(t) R_circle * cos(omega_circle * t);
mob_ref_y     = @(t) R_circle * sin(omega_circle * t);
mob_ref_theta = @(t) omega_circle * t + pi/2;

% === MPC Simulation ===
x_mpc_mob = [R_circle; 0; pi/2];   % Start on the circle
x_hist_mpc_mob = zeros(n_mob, length(t_span_mob));
u_hist_mpc_mob = zeros(m_mob, length(t_span_mob));
x_hist_mpc_mob(:,1) = x_mpc_mob;

tic;
for k = 1:length(t_span_mob)-1
    t = t_span_mob(k);

    % Current reference state
    xr = mob_ref_x(t);
    yr = mob_ref_y(t);
    thetar = mob_ref_theta(t);

    % Compute tracking error in the reference body frame
    dx_g = x_mpc_mob(1) - xr;
    dy_g = x_mpc_mob(2) - yr;
    e_x =  cos(thetar) * dx_g + sin(thetar) * dy_g;
    e_y = -sin(thetar) * dx_g + cos(thetar) * dy_g;
    e_theta = atan2(sin(x_mpc_mob(3) - thetar), cos(x_mpc_mob(3) - thetar));
    e_state = [e_x; e_y; e_theta];

    % MPC: drive tracking error to zero (reference for error system = 0)
    x_ref_mob = zeros(n_mob * N_mob, 1);

    % MPC optimal control (delta from reference velocities)
    E_mob = x_ref_mob - A_cal_mob * e_state;
    U_star_mob = H_inv_mob * F_mob * E_mob;
    delta_u = U_star_mob(1:m_mob);

    % Apply feedforward reference + MPC correction
    u_mob = [v_ref + delta_u(1); omega_ref + delta_u(2)];

    % Enforce constraints
    u_mob(1) = max(min(u_mob(1), 5), 0);      % v in [0, 5] m/s
    u_mob(2) = max(min(u_mob(2), 2), -2);     % omega in [-2, 2] rad/s

    % Simulate true nonlinear dynamics (Euler step)
    theta_curr = x_mpc_mob(3);
    x_mpc_mob = x_mpc_mob + [u_mob(1)*cos(theta_curr); ...
                               u_mob(1)*sin(theta_curr); ...
                               u_mob(2)] * dt_mob;

    x_hist_mpc_mob(:,k+1) = x_mpc_mob;
    u_hist_mpc_mob(:,k) = u_mob;
end
time_mpc_mob = toc;
fprintf('  MPC simulation time: %.3f s\n', time_mpc_mob);

% === PD Control Simulation (baseline) ===
k_pd_f = 10;   % Forward gain
k_pd_l = 15;   % Lateral gain

x_pd_mob = [R_circle; 0; pi/2];
x_hist_pd_mob = zeros(n_mob, length(t_span_mob));
u_hist_pd_mob = zeros(m_mob, length(t_span_mob));
x_hist_pd_mob(:,1) = x_pd_mob;

tic;
for k = 1:length(t_span_mob)-1
    t = t_span_mob(k);

    % Desired position
    xd_t = mob_ref_x(t);
    yd_t = mob_ref_y(t);
    theta_curr = x_pd_mob(3);

    % Errors in global frame
    ex = xd_t - x_pd_mob(1);
    ey = yd_t - x_pd_mob(2);

    % Transform to robot frame
    cos_th = cos(theta_curr);
    sin_th = sin(theta_curr);
    ex_r =  ex * cos_th + ey * sin_th;   % forward error
    ey_r = -ex * sin_th + ey * cos_th;   % lateral error

    % PD control
    v_pd = k_pd_f * ex_r;
    omega_pd = k_pd_l * ey_r;

    % Enforce same constraints
    v_pd = max(min(v_pd, 5), 0);
    omega_pd = max(min(omega_pd, 2), -2);

    % Simulate nonlinear dynamics
    x_pd_mob = x_pd_mob + [v_pd*cos(theta_curr); ...
                             v_pd*sin(theta_curr); ...
                             omega_pd] * dt_mob;

    x_hist_pd_mob(:,k+1) = x_pd_mob;
    u_hist_pd_mob(:,k) = [v_pd; omega_pd];
end
time_pd_mob = toc;
fprintf('  PD simulation time:  %.3f s\n', time_pd_mob);

% Reference trajectory for plotting
ref_x_mob = arrayfun(mob_ref_x, t_span_mob);
ref_y_mob = arrayfun(mob_ref_y, t_span_mob);

%% ========================================================================
%  3. QUADROTOR DRONE — MPC vs PD Control
% ========================================================================
fprintf('\n--- System 3: Quadrotor Drone ---\n');

% Physical parameters
quad_params.m   = 1.0;         % Mass (kg)
quad_params.Ixx = 0.1;        % Moment of inertia about x (kg*m^2)
quad_params.Iyy = 0.1;        % Moment of inertia about y (kg*m^2)
quad_params.Izz = 0.2;        % Moment of inertia about z (kg*m^2)
quad_params.g   = 9.81;       % Gravitational acceleration (m/s^2)

% Simulation parameters
dt_q = 0.02;                   % Sampling time (50 Hz)
t_span_q = 0:dt_q:20;         % 20 seconds simulation
n_q = 12;                      % State dimension
m_q = 4;                       % Control dimension [T, tau_x, tau_y, tau_z]

% Linearised continuous-time system (hover equilibrium)
Ac_q = zeros(12, 12);
Ac_q(1:3, 4:6) = eye(3);              % dp/dt = v
Ac_q(4, 8)     = quad_params.g;       % dvx/dt = g * theta
Ac_q(5, 7)     = -quad_params.g;      % dvy/dt = -g * phi
Ac_q(7:9, 10:12) = eye(3);            % datt/dt = omega

Bc_q = zeros(12, 4);
Bc_q(6, 1) = 1 / quad_params.m;       % dvz/dt = T/m (delta from hover)
Bc_q(10, 2) = 1 / quad_params.Ixx;    % domega_x/dt = tau_x / Ixx
Bc_q(11, 3) = 1 / quad_params.Iyy;    % domega_y/dt = tau_y / Iyy
Bc_q(12, 4) = 1 / quad_params.Izz;    % domega_z/dt = tau_z / Izz

% Discretise
Ad_q = eye(n_q) + Ac_q * dt_q;
Bd_q = Bc_q * dt_q;

% MPC parameters
N_q = 15;
Q_q = diag([80, 80, 120, 1, 1, 1, 50, 50, 50, 1, 1, 1]);
R_q = diag([0.1, 0.1, 0.1, 0.1]);

% Build prediction matrices
A_cal_q = zeros(n_q * N_q, n_q);
B_cal_q = zeros(n_q * N_q, m_q * N_q);

for i = 1:N_q
    A_cal_q((i-1)*n_q+1 : i*n_q, :) = Ad_q^i;
    for j = 1:i
        B_cal_q((i-1)*n_q+1 : i*n_q, ...
                (j-1)*m_q+1 : j*m_q) = Ad_q^(i-j) * Bd_q;
    end
end

Q_N_q = kron(eye(N_q), Q_q);
R_N_q = kron(eye(N_q), R_q);

H_q = B_cal_q' * Q_N_q * B_cal_q + R_N_q;
H_inv_q = inv(H_q);
F_q = B_cal_q' * Q_N_q;

fprintf('  Prediction matrices built: A_cal is %dx%d, B_cal is %dx%d\n', ...
    size(A_cal_q,1), size(A_cal_q,2), ...
    size(B_cal_q,1), size(B_cal_q,2));
fprintf('  Hessian H is %dx%d\n', size(H_q,1), size(H_q,2));

% Reference trajectory (helical: circle in XY + sinusoidal in Z)
R_helix = 3;
omega_helix = 0.5;
vz_amp = 0.5;
vz_freq = 0.2;

quad_ref_px  = @(t) R_helix * sin(omega_helix * t);
quad_ref_py  = @(t) R_helix * cos(omega_helix * t);
quad_ref_pz  = @(t) 5 + vz_amp * sin(vz_freq * t);
quad_ref_vx  = @(t) R_helix * omega_helix * cos(omega_helix * t);
quad_ref_vy  = @(t) -R_helix * omega_helix * sin(omega_helix * t);
quad_ref_vz  = @(t) vz_amp * vz_freq * cos(vz_freq * t);

% === MPC Simulation ===
% Start at the reference trajectory initial point (avoids a huge transient
% from [0,0,0] to [0,3,5] that would drive large angles and violate the
% small-angle linearisation assumption).
x_mpc_q = [quad_ref_px(0); quad_ref_py(0); quad_ref_pz(0); ...
           quad_ref_vx(0); quad_ref_vy(0); quad_ref_vz(0); ...
           0; 0; 0; 0; 0; 0];
x_hist_mpc_q = zeros(n_q, length(t_span_q));
u_hist_mpc_q = zeros(m_q, length(t_span_q));
x_hist_mpc_q(:,1) = x_mpc_q;

tic;
for k = 1:length(t_span_q)-1
    t = t_span_q(k);

    % Build reference vector over horizon
    x_ref_q = zeros(n_q * N_q, 1);
    for i = 1:N_q
        t_f = t + i * dt_q;
        x_ref_q((i-1)*n_q+1 : i*n_q) = ...
            [quad_ref_px(t_f); quad_ref_py(t_f); quad_ref_pz(t_f); ...
             quad_ref_vx(t_f); quad_ref_vy(t_f); quad_ref_vz(t_f); ...
             0; 0; 0; 0; 0; 0];
    end

    % MPC solve (u here is delta from hover)
    E_q = x_ref_q - A_cal_q * x_mpc_q;
    U_star_q = H_inv_q * F_q * E_q;
    u_delta = U_star_q(1:m_q);

    % Add hover thrust offset
    u_applied = u_delta;
    u_applied(1) = u_delta(1) + quad_params.m * quad_params.g;

    % Enforce constraints
    u_applied(1)   = max(min(u_applied(1), 2*quad_params.m*quad_params.g), 0);
    u_applied(2:4) = max(min(u_applied(2:4), 2), -2);

    % Simulate true nonlinear dynamics
    x_mpc_q = simulate_quadrotor_step(x_mpc_q, u_applied, quad_params, dt_q);

    x_hist_mpc_q(:,k+1) = x_mpc_q;
    u_hist_mpc_q(:,k) = u_applied;
end
time_mpc_q = toc;
fprintf('  MPC simulation time: %.3f s\n', time_mpc_q);

% === PD Control Simulation (baseline) ===
Kp_pos_q = diag([20, 20, 30]);
Kd_pos_q = diag([10, 10, 15]);
Kp_att_q = diag([30, 30, 30]);
Kd_att_q = diag([12, 12, 12]);

x_pd_q = [quad_ref_px(0); quad_ref_py(0); quad_ref_pz(0); ...
          quad_ref_vx(0); quad_ref_vy(0); quad_ref_vz(0); ...
          0; 0; 0; 0; 0; 0];
x_hist_pd_q = zeros(n_q, length(t_span_q));
u_hist_pd_q = zeros(m_q, length(t_span_q));
x_hist_pd_q(:,1) = x_pd_q;

tic;
for k = 1:length(t_span_q)-1
    t = t_span_q(k);

    pos = x_pd_q(1:3);
    vel = x_pd_q(4:6);
    att = x_pd_q(7:9);
    omg = x_pd_q(10:12);

    pos_des = [quad_ref_px(t); quad_ref_py(t); quad_ref_pz(t)];
    vel_des = [quad_ref_vx(t); quad_ref_vy(t); quad_ref_vz(t)];

    % Position PD
    e_pos = pos_des - pos;
    e_vel = vel_des - vel;
    acc_cmd = Kp_pos_q * e_pos + Kd_pos_q * e_vel;

    T_pd = quad_params.m * (quad_params.g + acc_cmd(3));

    % Attitude PD (desired attitude = 0)
    tau_att = Kp_att_q * (zeros(3,1) - att) + Kd_att_q * (zeros(3,1) - omg);

    u_pd_q = [T_pd; tau_att];

    % Enforce same constraints
    u_pd_q(1)   = max(min(u_pd_q(1), 2*quad_params.m*quad_params.g), 0);
    u_pd_q(2:4) = max(min(u_pd_q(2:4), 2), -2);

    % Simulate nonlinear dynamics
    x_pd_q = simulate_quadrotor_step(x_pd_q, u_pd_q, quad_params, dt_q);

    x_hist_pd_q(:,k+1) = x_pd_q;
    u_hist_pd_q(:,k) = u_pd_q;
end
time_pd_q = toc;
fprintf('  PD simulation time:  %.3f s\n', time_pd_q);

% Reference trajectory for plotting
ref_px_q = arrayfun(quad_ref_px, t_span_q);
ref_py_q = arrayfun(quad_ref_py, t_span_q);
ref_pz_q = arrayfun(quad_ref_pz, t_span_q);

%% ========================================================================
%  4. PLOTTING RESULTS
% ========================================================================
fprintf('\n--- Generating Plots ---\n');

% ---- Figure 1: Manipulator Results ----
figure('Name', 'System 1: 2-DOF Manipulator — MPC vs PD', ...
       'Position', [50, 400, 1400, 600]);

% Joint angles: MPC
subplot(2,3,1);
plot(t_span_manip, x_hist_mpc_manip(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(t_span_manip, x_hist_mpc_manip(2,:), 'r', 'LineWidth', 1.5);
plot(t_span_manip, qd_vals_manip(1,:), 'b--', 'LineWidth', 1.2);
plot(t_span_manip, qd_vals_manip(2,:), 'r--', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Angle (rad)');
title('MPC: Joint Angles'); grid on;
legend('q_1 (MPC)', 'q_2 (MPC)', 'q_{1,d}', 'q_{2,d}', 'Location', 'best');

% Joint angles: PD
subplot(2,3,2);
plot(t_span_manip, x_hist_pd_manip(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(t_span_manip, x_hist_pd_manip(2,:), 'r', 'LineWidth', 1.5);
plot(t_span_manip, qd_vals_manip(1,:), 'b--', 'LineWidth', 1.2);
plot(t_span_manip, qd_vals_manip(2,:), 'r--', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Angle (rad)');
title('PD: Joint Angles'); grid on;
legend('q_1 (PD)', 'q_2 (PD)', 'q_{1,d}', 'q_{2,d}', 'Location', 'best');

% Tracking error comparison
subplot(2,3,3);
e_mpc_manip = qd_vals_manip - x_hist_mpc_manip(1:2,:);
e_pd_manip  = qd_vals_manip - x_hist_pd_manip(1:2,:);
plot(t_span_manip, vecnorm(e_mpc_manip), 'b', 'LineWidth', 1.5); hold on;
plot(t_span_manip, vecnorm(e_pd_manip), 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('||e|| (rad)');
title('Tracking Error Norm'); grid on;
legend('MPC', 'PD', 'Location', 'best');

% Control inputs: MPC
subplot(2,3,4);
plot(t_span_manip(1:end-1), u_hist_mpc_manip(1,1:end-1), 'b', 'LineWidth', 1); hold on;
plot(t_span_manip(1:end-1), u_hist_mpc_manip(2,1:end-1), 'r', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Torque (N\cdotm)');
title('MPC: Control Inputs'); grid on;
legend('\tau_1', '\tau_2', 'Location', 'best');

% Control inputs: PD
subplot(2,3,5);
plot(t_span_manip(1:end-1), u_hist_pd_manip(1,1:end-1), 'b', 'LineWidth', 1); hold on;
plot(t_span_manip(1:end-1), u_hist_pd_manip(2,1:end-1), 'r', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Torque (N\cdotm)');
title('PD: Control Inputs'); grid on;
legend('\tau_1', '\tau_2', 'Location', 'best');

% Error comparison (per joint)
subplot(2,3,6);
plot(t_span_manip, e_mpc_manip(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(t_span_manip, e_mpc_manip(2,:), 'r', 'LineWidth', 1.5);
plot(t_span_manip, e_pd_manip(1,:), 'b--', 'LineWidth', 1);
plot(t_span_manip, e_pd_manip(2,:), 'r--', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Error (rad)');
title('Per-Joint Error: MPC (solid) vs PD (dashed)'); grid on;
legend('e_1 MPC', 'e_2 MPC', 'e_1 PD', 'e_2 PD', 'Location', 'best');

sgtitle('System 1: 2-DOF Manipulator — MPC vs PD Control');

% ---- Figure 2: Mobile Robot Results ----
figure('Name', 'System 2: Mobile Robot — MPC vs PD', ...
       'Position', [50, 50, 1400, 600]);

% XY trajectory
subplot(2,3,1);
plot(x_hist_mpc_mob(1,:), x_hist_mpc_mob(2,:), 'b', 'LineWidth', 1.5); hold on;
plot(x_hist_pd_mob(1,:), x_hist_pd_mob(2,:), 'r', 'LineWidth', 1.5);
plot(ref_x_mob, ref_y_mob, 'k--', 'LineWidth', 1.2);
xlabel('X (m)'); ylabel('Y (m)');
title('XY Trajectory'); grid on; axis equal;
legend('MPC', 'PD', 'Reference', 'Location', 'best');

% Position error
subplot(2,3,2);
ex_mpc = ref_x_mob - x_hist_mpc_mob(1,:);
ey_mpc = ref_y_mob - x_hist_mpc_mob(2,:);
ex_pd  = ref_x_mob - x_hist_pd_mob(1,:);
ey_pd  = ref_y_mob - x_hist_pd_mob(2,:);
plot(t_span_mob, sqrt(ex_mpc.^2 + ey_mpc.^2), 'b', 'LineWidth', 1.5); hold on;
plot(t_span_mob, sqrt(ex_pd.^2 + ey_pd.^2), 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Position Error (m)');
title('Position Error Norm'); grid on;
legend('MPC', 'PD', 'Location', 'best');

% Heading
subplot(2,3,3);
ref_theta_mob = arrayfun(mob_ref_theta, t_span_mob);
plot(t_span_mob, x_hist_mpc_mob(3,:), 'b', 'LineWidth', 1.5); hold on;
plot(t_span_mob, x_hist_pd_mob(3,:), 'r', 'LineWidth', 1.5);
plot(t_span_mob, ref_theta_mob, 'k--', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('\theta (rad)');
title('Heading Angle'); grid on;
legend('MPC', 'PD', 'Reference', 'Location', 'best');

% Control: v
subplot(2,3,4);
plot(t_span_mob(1:end-1), u_hist_mpc_mob(1,1:end-1), 'b', 'LineWidth', 1); hold on;
plot(t_span_mob(1:end-1), u_hist_pd_mob(1,1:end-1), 'r', 'LineWidth', 1);
yline(5, 'k--', 'v_{max}'); yline(0, 'k--', 'v_{min}');
xlabel('Time (s)'); ylabel('v (m/s)');
title('Linear Velocity'); grid on;
legend('MPC', 'PD', 'Location', 'best');

% Control: omega
subplot(2,3,5);
plot(t_span_mob(1:end-1), u_hist_mpc_mob(2,1:end-1), 'b', 'LineWidth', 1); hold on;
plot(t_span_mob(1:end-1), u_hist_pd_mob(2,1:end-1), 'r', 'LineWidth', 1);
yline(2, 'k--', '\omega_{max}'); yline(-2, 'k--', '\omega_{min}');
xlabel('Time (s)'); ylabel('\omega (rad/s)');
title('Angular Velocity'); grid on;
legend('MPC', 'PD', 'Location', 'best');

% X and Y errors
subplot(2,3,6);
plot(t_span_mob, ex_mpc, 'b', 'LineWidth', 1.5); hold on;
plot(t_span_mob, ey_mpc, 'r', 'LineWidth', 1.5);
plot(t_span_mob, ex_pd, 'b--', 'LineWidth', 1);
plot(t_span_mob, ey_pd, 'r--', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Error (m)');
title('X,Y Errors: MPC (solid) vs PD (dashed)'); grid on;
legend('e_x MPC', 'e_y MPC', 'e_x PD', 'e_y PD', 'Location', 'best');

sgtitle('System 2: Differential-Drive Mobile Robot — MPC vs PD Control');

% ---- Figure 3: Quadrotor Results ----
figure('Name', 'System 3: Quadrotor — MPC vs PD', ...
       'Position', [100, 200, 1400, 800]);

% 3D trajectory
subplot(2,3,1);
plot3(x_hist_mpc_q(1,:), x_hist_mpc_q(2,:), x_hist_mpc_q(3,:), ...
      'b', 'LineWidth', 1.5); hold on;
plot3(x_hist_pd_q(1,:), x_hist_pd_q(2,:), x_hist_pd_q(3,:), ...
      'r', 'LineWidth', 1.5);
plot3(ref_px_q, ref_py_q, ref_pz_q, 'k--', 'LineWidth', 1.2);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Trajectory'); grid on;
legend('MPC', 'PD', 'Reference', 'Location', 'best');
view(30, 25);

% Position components
subplot(2,3,2);
plot(t_span_q, x_hist_mpc_q(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(t_span_q, x_hist_mpc_q(2,:), 'r', 'LineWidth', 1.5);
plot(t_span_q, x_hist_mpc_q(3,:), 'g', 'LineWidth', 1.5);
plot(t_span_q, ref_px_q, 'b--', 'LineWidth', 1);
plot(t_span_q, ref_py_q, 'r--', 'LineWidth', 1);
plot(t_span_q, ref_pz_q, 'g--', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Position (m)');
title('MPC: Position vs Reference'); grid on;
legend('x', 'y', 'z', 'x_d', 'y_d', 'z_d', 'Location', 'best');

% Position error norm
subplot(2,3,3);
e_pos_mpc_q = [ref_px_q; ref_py_q; ref_pz_q] - x_hist_mpc_q(1:3,:);
e_pos_pd_q  = [ref_px_q; ref_py_q; ref_pz_q] - x_hist_pd_q(1:3,:);
plot(t_span_q, vecnorm(e_pos_mpc_q), 'b', 'LineWidth', 1.5); hold on;
plot(t_span_q, vecnorm(e_pos_pd_q), 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('||e_{pos}|| (m)');
title('Position Error Norm'); grid on;
legend('MPC', 'PD', 'Location', 'best');

% Thrust
subplot(2,3,4);
plot(t_span_q(1:end-1), u_hist_mpc_q(1,1:end-1), 'b', 'LineWidth', 1); hold on;
plot(t_span_q(1:end-1), u_hist_pd_q(1,1:end-1), 'r', 'LineWidth', 1);
yline(quad_params.m*quad_params.g, 'k--', 'Hover thrust');
yline(2*quad_params.m*quad_params.g, 'k:', 'T_{max}');
xlabel('Time (s)'); ylabel('Thrust (N)');
title('Thrust Input'); grid on;
legend('MPC', 'PD', 'Location', 'best');

% Body torques (MPC)
subplot(2,3,5);
plot(t_span_q(1:end-1), u_hist_mpc_q(2,1:end-1), 'b', 'LineWidth', 1); hold on;
plot(t_span_q(1:end-1), u_hist_mpc_q(3,1:end-1), 'r', 'LineWidth', 1);
plot(t_span_q(1:end-1), u_hist_mpc_q(4,1:end-1), 'g', 'LineWidth', 1);
yline(2, 'k--'); yline(-2, 'k--');
xlabel('Time (s)'); ylabel('Torque (N\cdotm)');
title('MPC: Body Torques'); grid on;
legend('\tau_x', '\tau_y', '\tau_z', 'Location', 'best');

% Attitude angles
subplot(2,3,6);
plot(t_span_q, rad2deg(x_hist_mpc_q(7,:)), 'b', 'LineWidth', 1.5); hold on;
plot(t_span_q, rad2deg(x_hist_mpc_q(8,:)), 'r', 'LineWidth', 1.5);
plot(t_span_q, rad2deg(x_hist_mpc_q(9,:)), 'g', 'LineWidth', 1.5);
yline(30, 'k--', '\phi,\theta max'); yline(-30, 'k--');
xlabel('Time (s)'); ylabel('Angle (deg)');
title('MPC: Euler Angles'); grid on;
legend('\phi (roll)', '\theta (pitch)', '\psi (yaw)', 'Location', 'best');

sgtitle('System 3: Quadrotor Drone — MPC vs PD Control');

% ---- Figure 4: Summary Comparison ----
figure('Name', 'Summary: MPC vs PD for All Systems', ...
       'Position', [150, 300, 1200, 400]);

% Compute summary metrics
peak_err_mpc_manip = max(vecnorm(e_mpc_manip));
peak_err_pd_manip  = max(vecnorm(e_pd_manip));
peak_err_mpc_mob   = max(sqrt(ex_mpc.^2 + ey_mpc.^2));
peak_err_pd_mob    = max(sqrt(ex_pd.^2 + ey_pd.^2));
peak_err_mpc_q     = max(vecnorm(e_pos_mpc_q));
peak_err_pd_q      = max(vecnorm(e_pos_pd_q));

ss_err_mpc_manip = mean(vecnorm(e_mpc_manip(:,end-200:end)));
ss_err_pd_manip  = mean(vecnorm(e_pd_manip(:,end-200:end)));
ss_err_mpc_mob   = mean(sqrt(ex_mpc(end-40:end).^2 + ey_mpc(end-40:end).^2));
ss_err_pd_mob    = mean(sqrt(ex_pd(end-40:end).^2 + ey_pd(end-40:end).^2));
ss_err_mpc_q     = mean(vecnorm(e_pos_mpc_q(:,end-100:end)));
ss_err_pd_q      = mean(vecnorm(e_pos_pd_q(:,end-100:end)));

subplot(1,3,1);
bar_data = [peak_err_mpc_manip, peak_err_pd_manip; ...
            peak_err_mpc_mob, peak_err_pd_mob; ...
            peak_err_mpc_q, peak_err_pd_q];
bar(bar_data);
set(gca, 'XTickLabel', {'Manipulator', 'Mobile Robot', 'Quadrotor'});
ylabel('Peak Error'); title('Peak Tracking Error');
legend('MPC', 'PD', 'Location', 'best'); grid on;

subplot(1,3,2);
bar_data_ss = [ss_err_mpc_manip, ss_err_pd_manip; ...
               ss_err_mpc_mob, ss_err_pd_mob; ...
               ss_err_mpc_q, ss_err_pd_q];
bar(bar_data_ss);
set(gca, 'XTickLabel', {'Manipulator', 'Mobile Robot', 'Quadrotor'});
ylabel('Steady-State Error'); title('Mean Steady-State Error');
legend('MPC', 'PD', 'Location', 'best'); grid on;

subplot(1,3,3);
bar_time = [time_mpc_manip, time_pd_manip; ...
            time_mpc_mob, time_pd_mob; ...
            time_mpc_q, time_pd_q];
bar(bar_time);
set(gca, 'XTickLabel', {'Manipulator', 'Mobile Robot', 'Quadrotor'});
ylabel('Time (s)'); title('Computation Time');
legend('MPC', 'PD', 'Location', 'best'); grid on;

sgtitle('Summary: MPC vs PD Control — All Three Systems');

%% ========================================================================
%  5. PRINT NUMERICAL SUMMARY
% ========================================================================
fprintf('\n========================================\n');
fprintf('         NUMERICAL SUMMARY\n');
fprintf('========================================\n\n');

fprintf('System 1: 2-DOF Manipulator\n');
fprintf('  MPC — Peak error: %.4f rad, Steady-state: %.4f rad\n', peak_err_mpc_manip, ss_err_mpc_manip);
fprintf('  PD  — Peak error: %.4f rad, Steady-state: %.4f rad\n', peak_err_pd_manip, ss_err_pd_manip);
fprintf('  Computation: MPC = %.3fs, PD = %.3fs\n\n', time_mpc_manip, time_pd_manip);

fprintf('System 2: Mobile Robot\n');
fprintf('  MPC — Peak error: %.4f m, Steady-state: %.4f m\n', peak_err_mpc_mob, ss_err_mpc_mob);
fprintf('  PD  — Peak error: %.4f m, Steady-state: %.4f m\n', peak_err_pd_mob, ss_err_pd_mob);
fprintf('  Computation: MPC = %.3fs, PD = %.3fs\n\n', time_mpc_mob, time_pd_mob);

fprintf('System 3: Quadrotor Drone\n');
fprintf('  MPC — Peak error: %.4f m, Steady-state: %.4f m\n', peak_err_mpc_q, ss_err_mpc_q);
fprintf('  PD  — Peak error: %.4f m, Steady-state: %.4f m\n', peak_err_pd_q, ss_err_pd_q);
fprintf('  Computation: MPC = %.3fs, PD = %.3fs\n\n', time_mpc_q, time_pd_q);

fprintf('========================================\n');
fprintf('Done. All figures generated.\n');

%% ========================================================================
%  HELPER FUNCTIONS
% ========================================================================

function [M, C, G] = build_manipulator_model(q, dq, params)
% BUILD_MANIPULATOR_MODEL Compute M(q), C(q,dq), G(q) for a 2-DOF planar arm.
%
%   Inputs:
%       q     - [2x1] joint angles (rad)
%       dq    - [2x1] joint velocities (rad/s)
%       params - struct with fields: m1, m2, l1, l2, g
%
%   Outputs:
%       M - [2x2] mass (inertia) matrix
%       C - [2x2] Coriolis and centrifugal matrix
%       G - [2x1] gravity vector

    m1 = params.m1; m2 = params.m2;
    l1 = params.l1; l2 = params.l2;
    g  = params.g;

    c2 = cos(q(2));
    s2 = sin(q(2));
    c1 = cos(q(1));
    c12 = cos(q(1) + q(2));

    % Mass matrix M(q) — symmetric positive definite
    M11 = m1*l1^2 + m2*(l1^2 + l2^2 + 2*l1*l2*c2);
    M12 = m2*(l2^2 + l1*l2*c2);
    M22 = m2*l2^2;
    M = [M11, M12; M12, M22];

    % Coriolis matrix C(q, dq) — using Christoffel symbols
    h = m2*l1*l2*s2;
    C = [-h*dq(2),  -h*(dq(1) + dq(2));
          h*dq(1),   0];

    % Gravity vector G(q)
    G = [(m1 + m2)*g*l1*c1 + m2*g*l2*c12;
          m2*g*l2*c12];
end

function x_next = simulate_quadrotor_step(x, u, params, dt)
% SIMULATE_QUADROTOR_STEP Full nonlinear quadrotor dynamics (Euler step).
%
%   State x = [px; py; pz; vx; vy; vz; phi; theta; psi; wx; wy; wz]
%   Input u = [T; tau_x; tau_y; tau_z]
%
%   Translational dynamics use the full rotation matrix (small-angle
%   approximation is NOT used here — this is the true plant).

    pos = x(1:3);
    vel = x(4:6);
    att = x(7:9);    % [phi, theta, psi]
    omg = x(10:12);  % [omega_x, omega_y, omega_z]

    phi = att(1); th = att(2); psi = att(3);
    T = u(1);
    tau_body = u(2:4);

    m = params.m;
    g = params.g;
    Ixx = params.Ixx;
    Iyy = params.Iyy;
    Izz = params.Izz;

    % Translational acceleration (full nonlinear, world frame)
    ax = (T/m) * (cos(phi)*sin(th)*cos(psi) + sin(phi)*sin(psi));
    ay = (T/m) * (cos(phi)*sin(th)*sin(psi) - sin(phi)*cos(psi));
    az = (T/m) * cos(phi)*cos(th) - g;

    % Rotational acceleration (Euler's equations)
    domg_x = tau_body(1)/Ixx + (Iyy - Izz)/Ixx * omg(2)*omg(3);
    domg_y = tau_body(2)/Iyy + (Izz - Ixx)/Iyy * omg(1)*omg(3);
    domg_z = tau_body(3)/Izz + (Ixx - Iyy)/Izz * omg(1)*omg(2);

    % State derivative
    dx = [vel;              % dp/dt = v
          ax; ay; az;       % dv/dt = acceleration
          omg;              % datt/dt = omega (small-angle approx for kinematics)
          domg_x; domg_y; domg_z];  % domega/dt = Euler's equations

    % Euler integration
    x_next = x + dx * dt;
end
