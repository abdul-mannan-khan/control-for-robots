%% Week 1: Control Simulation with Saved Results
% Generates and saves all figures for trajectory tracking, error, and control effort

clear all; close all; clc;
fprintf('=== Week 1: Basic Control Simulation with Saved Results ===\n\n');

output_dir = '/home/it-services/auto_control_ws/results/week_01';

%% 1. Robot Manipulator Simulation
fprintf('Simulating 2-DOF Manipulator...\n');

manipulator_params.m1 = 1;
manipulator_params.m2 = 1;
manipulator_params.l1 = 1;
manipulator_params.l2 = 1;
manipulator_params.g = 9.81;
manipulator_params.Kp = diag([50 50]);
manipulator_params.Kv = diag([20 20]);

t_span = 0:0.01:20;
q0 = [0; 0]; dq0 = [0; 0];
x0_manip = [q0; dq0];

qd = @(t) [0.5*sin(t); 0.5*cos(t)];
dqd = @(t) [0.5*cos(t); -0.5*sin(t)];
ddqd = @(t) [-0.5*sin(t); -0.5*cos(t)];

[t_manip, x_manip] = ode45(@(t,x) manipulator_dynamics(t, x, manipulator_params, qd, dqd, ddqd), t_span, x0_manip);

% Compute desired and errors
qd_vals = zeros(length(t_manip),2);
tau_vals = zeros(length(t_manip),2);
for i=1:length(t_manip)
    qd_vals(i,:) = qd(t_manip(i))';
    e = qd(t_manip(i)) - x_manip(i,1:2)';
    de = dqd(t_manip(i)) - x_manip(i,3:4)';
    tau_vals(i,:) = (manipulator_params.Kp*e + manipulator_params.Kv*de)';
end
e_manip = qd_vals - x_manip(:,1:2);

%% 2. Mobile Robot Simulation
fprintf('Simulating Mobile Robot...\n');

mobile_params.k1 = 10;
mobile_params.k2 = 15;
mobile_params.k3 = 8;

x0_mobile = [0; 0; 0];
mobile_params.xd = @(t) 5*cos(0.5*t);
mobile_params.yd = @(t) 5*sin(0.5*t);

[t_mobile, x_mobile] = ode45(@(t,x) mobile_robot_dynamics(t, x, mobile_params), t_span, x0_mobile);

% Compute control inputs
v_mobile = zeros(length(t_mobile),1);
omega_mobile = zeros(length(t_mobile),1);
for i = 1:length(t_mobile)
    xd_t = mobile_params.xd(t_mobile(i));
    yd_t = mobile_params.yd(t_mobile(i));
    ex = xd_t - x_mobile(i,1);
    ey = yd_t - x_mobile(i,2);
    thr = x_mobile(i,3);
    ex_r =  ex*cos(thr) + ey*sin(thr);
    ey_r = -ex*sin(thr) + ey*cos(thr);
    v_mobile(i) = mobile_params.k1 * ex_r;
    omega_mobile(i) = mobile_params.k2 * ey_r;
end

%% 3. Quadrotor Simulation
fprintf('Simulating Quadrotor...\n');

quad_params.m = 1;
quad_params.Ixx = 0.1;  quad_params.Iyy = 0.1;  quad_params.Izz = 0.1;
quad_params.g = 9.81;
quad_params.Kp_pos = diag([20 20 30]);
quad_params.Kd_pos = diag([10 10 15]);
quad_params.Kp_att = diag([30 30 30]);
quad_params.Kd_att = diag([12 12 12]);

x0_quad = zeros(12,1);
xd_quad = @(t) [3*sin(0.5*t); 3*cos(0.5*t); 6 + 0.5*sin(0.2*t)];
dxd_quad = @(t) [1.5*cos(0.5*t); -1.5*sin(0.5*t); 0.1*cos(0.2*t)];
ddxd_quad = @(t) [-0.75*sin(0.5*t); -0.75*cos(0.5*t); -0.02*sin(0.2*t)];

[t_quad, x_quad] = ode45(@(t,x) quadrotor_dynamics(t, x, quad_params, xd_quad, dxd_quad, ddxd_quad), t_span, x0_quad);

% Compute desired trajectory
xd_vals = zeros(length(t_quad),3);
for i=1:length(t_quad)
    xd_vals(i,:) = xd_quad(t_quad(i))';
end

%% Generate and Save Plots

% === Figure 1: Manipulator Joint Tracking ===
fig1 = figure('Position', [100 100 1200 800], 'Visible', 'off');
subplot(2,2,1);
plot(t_manip, x_manip(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_manip, qd_vals(:,1), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Angle (rad)');
title('Joint 1 Tracking'); legend('Actual q_1', 'Desired q_{d1}');
grid on;

subplot(2,2,2);
plot(t_manip, x_manip(:,2), 'b', 'LineWidth', 1.5); hold on;
plot(t_manip, qd_vals(:,2), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Angle (rad)');
title('Joint 2 Tracking'); legend('Actual q_2', 'Desired q_{d2}');
grid on;

subplot(2,2,3);
plot(t_manip, e_manip(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_manip, e_manip(:,2), 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Error (rad)');
title('Tracking Errors'); legend('e_1', 'e_2');
grid on;

subplot(2,2,4);
plot(t_manip, tau_vals(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_manip, tau_vals(:,2), 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Torque (N\cdotm)');
title('Control Torques'); legend('\tau_1', '\tau_2');
grid on;

sgtitle('Week 1: 2-DOF Manipulator - PD Control', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig1, fullfile(output_dir, 'manipulator_tracking.png'));
saveas(fig1, fullfile(output_dir, 'manipulator_tracking.fig'));
fprintf('  Saved: manipulator_tracking.png\n');

% === Figure 2: Mobile Robot Tracking ===
fig2 = figure('Position', [100 100 1400 500], 'Visible', 'off');
subplot(1,3,1);
xd_plot = arrayfun(mobile_params.xd, t_mobile);
yd_plot = arrayfun(mobile_params.yd, t_mobile);
plot(x_mobile(:,1), x_mobile(:,2), 'b', 'LineWidth', 1.5); hold on;
plot(xd_plot, yd_plot, 'r--', 'LineWidth', 1.5);
plot(x_mobile(1,1), x_mobile(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
xlabel('X (m)'); ylabel('Y (m)');
title('Path Tracking'); legend('Actual', 'Desired', 'Start');
axis equal; grid on;

subplot(1,3,2);
x_err = xd_plot - x_mobile(:,1);
y_err = yd_plot - x_mobile(:,2);
plot(t_mobile, x_err, 'b', 'LineWidth', 1.5); hold on;
plot(t_mobile, y_err, 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Error (m)');
title('Position Errors'); legend('e_x', 'e_y');
grid on;

subplot(1,3,3);
plot(t_mobile, v_mobile, 'b', 'LineWidth', 1.5); hold on;
plot(t_mobile, omega_mobile, 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Control');
title('Control Inputs'); legend('v (m/s)', '\omega (rad/s)');
grid on;

sgtitle('Week 1: Mobile Robot - Tracking Control', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig2, fullfile(output_dir, 'mobile_robot_tracking.png'));
saveas(fig2, fullfile(output_dir, 'mobile_robot_tracking.fig'));
fprintf('  Saved: mobile_robot_tracking.png\n');

% === Figure 3: Quadrotor Tracking ===
fig3 = figure('Position', [100 100 1400 800], 'Visible', 'off');
subplot(2,3,1);
plot3(x_quad(:,1), x_quad(:,2), x_quad(:,3), 'b', 'LineWidth', 1.5); hold on;
plot3(xd_vals(:,1), xd_vals(:,2), xd_vals(:,3), 'r--', 'LineWidth', 1.5);
plot3(x_quad(1,1), x_quad(1,2), x_quad(1,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Trajectory'); legend('Actual', 'Desired', 'Start');
grid on; view(45, 30);

subplot(2,3,2);
plot(t_quad, x_quad(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_quad, xd_vals(:,1), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('X (m)');
title('X Position'); legend('Actual', 'Desired');
grid on;

subplot(2,3,3);
plot(t_quad, x_quad(:,3), 'b', 'LineWidth', 1.5); hold on;
plot(t_quad, xd_vals(:,3), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Z (m)');
title('Altitude (Z)'); legend('Actual', 'Desired');
grid on;

subplot(2,3,4);
e_quad = xd_vals - x_quad(:,1:3);
plot(t_quad, e_quad(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_quad, e_quad(:,2), 'r', 'LineWidth', 1.5);
plot(t_quad, e_quad(:,3), 'g', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Error (m)');
title('Position Errors'); legend('e_x', 'e_y', 'e_z');
grid on;

subplot(2,3,5);
plot(t_quad, rad2deg(x_quad(:,4)), 'b', 'LineWidth', 1.2); hold on;
plot(t_quad, rad2deg(x_quad(:,5)), 'r', 'LineWidth', 1.2);
plot(t_quad, rad2deg(x_quad(:,6)), 'g', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Euler Angles'); legend('\phi', '\theta', '\psi');
grid on;

subplot(2,3,6);
plot(t_quad, sqrt(e_quad(:,1).^2 + e_quad(:,2).^2 + e_quad(:,3).^2), 'k', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('||e|| (m)');
title('Position Error Norm');
grid on;

sgtitle('Week 1: Quadrotor - PD Position Control', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig3, fullfile(output_dir, 'quadrotor_tracking.png'));
saveas(fig3, fullfile(output_dir, 'quadrotor_tracking.fig'));
fprintf('  Saved: quadrotor_tracking.png\n');

% === Figure 4: Summary Comparison ===
fig4 = figure('Position', [100 100 1000 400], 'Visible', 'off');

subplot(1,3,1);
bar([rms(e_manip(:,1)), rms(e_manip(:,2))]);
set(gca, 'XTickLabel', {'Joint 1', 'Joint 2'});
ylabel('RMSE (rad)'); title('Manipulator RMSE');
grid on;

subplot(1,3,2);
bar([rms(x_err), rms(y_err)]);
set(gca, 'XTickLabel', {'X', 'Y'});
ylabel('RMSE (m)'); title('Mobile Robot RMSE');
grid on;

subplot(1,3,3);
bar([rms(e_quad(:,1)), rms(e_quad(:,2)), rms(e_quad(:,3))]);
set(gca, 'XTickLabel', {'X', 'Y', 'Z'});
ylabel('RMSE (m)'); title('Quadrotor RMSE');
grid on;

sgtitle('Week 1: Performance Summary', 'FontSize', 14, 'FontWeight', 'bold');
saveas(fig4, fullfile(output_dir, 'performance_summary.png'));
saveas(fig4, fullfile(output_dir, 'performance_summary.fig'));
fprintf('  Saved: performance_summary.png\n');

fprintf('\n=== Week 1 Results Saved Successfully ===\n');
fprintf('Output directory: %s\n', output_dir);

%% Helper Functions
function dx = manipulator_dynamics(t, x, params, qd, dqd, ddqd)
    q = x(1:2);
    dq = x(3:4);
    q_des = qd(t);
    dq_des = dqd(t);
    e = q_des - q;
    de = dq_des - dq;
    tau = params.Kp*e + params.Kv*de;
    dx = [dq; tau];
end

function dx = mobile_robot_dynamics(t, x, params)
    xr = x(1); yr = x(2); thr = x(3);
    xd_t = params.xd(t);
    yd_t = params.yd(t);
    ex = xd_t - xr;
    ey = yd_t - yr;
    cos_th = cos(thr);
    sin_th = sin(thr);
    ex_r =  ex*cos_th + ey*sin_th;
    ey_r = -ex*sin_th + ey*cos_th;
    v = params.k1 * ex_r;
    omega = params.k2 * ey_r;
    dx = [v*cos_th; v*sin_th; omega];
end

function dx = quadrotor_dynamics(t, x, params, xd, dxd, ddxd)
    pos = x(1:3);
    att = x(4:6);
    vel = x(7:9);
    omega = x(10:12);

    pos_des = xd(t);
    vel_des = dxd(t);
    acc_des = ddxd(t);

    e_pos = pos_des - pos;
    e_vel = vel_des - vel;

    acc_cmd = params.Kp_pos*e_pos + params.Kd_pos*e_vel + acc_des;

    dx = zeros(12,1);
    dx(1:3) = vel;
    dx(4:6) = omega;
    dx(7:9) = acc_cmd;
    dx(10:12) = zeros(3,1);
end
