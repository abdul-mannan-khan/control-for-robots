%% Week 7: Backstepping Control - Enhanced Quadrotor Analysis
fprintf('=== Week 7: Enhanced Quadrotor Control Analysis ===\n\n');

% Run the main simulation first
run('backstepping_control_simulation.m');

% Define output directory
output_dir = '/home/it-services/auto_control_ws/results/week_06';

%% Recompute quadrotor data with additional diagnostics
fprintf('\n--- Computing Enhanced Quadrotor Diagnostics ---\n');

% Quadrotor parameters (same as main simulation)
quad.m   = 1.5;
quad.g   = 9.81;
quad.Ixx = 0.03;
quad.Iyy = 0.03;
quad.Izz = 0.06;
quad.kp1 = 4.0;
quad.kp2 = 5.0;
quad.ka1 = 15.0;
quad.ka2 = 8.0;

% Reference trajectory function
quad_reference = @(t) deal(...
    [2*cos(0.4*t); 2*sin(0.4*t); 2 + 0.2*t], ...
    [-2*0.4*sin(0.4*t); 2*0.4*cos(0.4*t); 0.2], ...
    [-2*0.16*cos(0.4*t); -2*0.16*sin(0.4*t); 0]);

% Time vector for analysis (use data from main simulation)
N = length(t_q);

% Preallocate enhanced diagnostics
pos_err      = zeros(N, 3);   % Position errors [ex, ey, ez]
vel_err      = zeros(N, 3);   % Velocity errors
att_err      = zeros(N, 3);   % Attitude errors [phi_e, theta_e, psi_e]
att_des      = zeros(N, 3);   % Desired attitudes
z1_norm      = zeros(N, 1);   % Position error norm
z2_norm      = zeros(N, 1);   % Velocity error norm
z3_norm      = zeros(N, 1);   % Attitude error norm
thrust_ratio = zeros(N, 1);   % Thrust / hover thrust
ctrl_power   = zeros(N, 1);   % Instantaneous control power

for i = 1:N
    t = t_q(i);
    state = x_q(i,:)';

    % Extract state
    pos = state(1:3);
    vel = state(4:6);
    phi = state(7);
    theta = state(8);
    psi = state(9);
    omega_b = state(10:12);

    % Reference
    [pos_d, vel_d, acc_d] = quad_reference(t);
    psi_d = 0;

    % Position errors (z1)
    z1 = pos - pos_d;
    dz1 = vel - vel_d;
    pos_err(i,:) = z1';
    vel_err(i,:) = dz1';
    z1_norm(i) = norm(z1);

    % Virtual control and velocity error (z2)
    alpha1 = vel_d - quad.kp1 * z1;
    z2 = vel - alpha1;
    z2_norm(i) = norm(z2);

    % Desired acceleration and thrust direction
    a_des = acc_d - quad.kp1 * dz1 - z1 - quad.kp2 * z2 + [0; 0; quad.g];
    a_des_norm = max(norm(a_des), 0.1);
    z_B_des = a_des / a_des_norm;

    % Desired attitudes from thrust direction
    phi_d = asin(max(-1, min(1, z_B_des(1)*sin(psi_d) - z_B_des(2)*cos(psi_d))));
    theta_d = atan2(z_B_des(1)*cos(psi_d) + z_B_des(2)*sin(psi_d), z_B_des(3));
    phi_d = max(-deg2rad(35), min(deg2rad(35), phi_d));
    theta_d = max(-deg2rad(35), min(deg2rad(35), theta_d));

    att_des(i,:) = [phi_d, theta_d, psi_d] * 180/pi;

    % Attitude error (z3)
    z3 = [phi - phi_d; theta - theta_d; psi - psi_d];
    z3(3) = atan2(sin(z3(3)), cos(z3(3)));
    att_err(i,:) = z3' * 180/pi;
    z3_norm(i) = norm(z3);

    % Thrust ratio
    thrust_ratio(i) = thrust_q(i) / (quad.m * quad.g);

    % Control power: T*v_z + tau'*omega
    ctrl_power(i) = abs(thrust_q(i) * vel(3)) + abs(torque_q(i,:) * omega_b);
end

% Cumulative control effort
ctrl_effort_thrust = cumtrapz(t_q, thrust_q.^2);
ctrl_effort_torque = cumtrapz(t_q, sum(torque_q.^2, 2));

%% Figure 1: Position Error Dynamics (Detailed)
figure('Name','Quadrotor: Position Error Dynamics','Position',[100 100 1400 500]);

subplot(1,3,1);
plot(t_q, pos_err(:,1)*1000, 'b-', 'LineWidth', 1.5); hold on;
plot(t_q, pos_err(:,2)*1000, 'r-', 'LineWidth', 1.5);
plot(t_q, pos_err(:,3)*1000, 'g-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Position Error (mm)');
title('Position Errors e_x, e_y, e_z');
legend('e_x','e_y','e_z','Location','best');
grid on;

subplot(1,3,2);
plot(t_q, z1_norm*1000, 'k-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('||e_{pos}|| (mm)');
title('Position Error Norm');
grid on;

subplot(1,3,3);
plot(t_q, vel_err(:,1), 'b-', 'LineWidth', 1.2); hold on;
plot(t_q, vel_err(:,2), 'r-', 'LineWidth', 1.2);
plot(t_q, vel_err(:,3), 'g-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Velocity Error (m/s)');
title('Velocity Errors');
legend('\delta v_x','\delta v_y','\delta v_z','Location','best');
grid on;

sgtitle('Quadrotor Backstepping: Position Error Dynamics', 'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, fullfile(output_dir, 'Quad_Position_Error_Dynamics.png'));
saveas(gcf, fullfile(output_dir, 'Quad_Position_Error_Dynamics.fig'));
fprintf('  Saved: Quad_Position_Error_Dynamics.png\n');

%% Figure 2: Attitude Control Analysis
figure('Name','Quadrotor: Attitude Control','Position',[100 100 1400 800]);

subplot(2,3,1);
plot(t_q, x_q(:,7)*180/pi, 'b-', 'LineWidth', 1.5); hold on;
plot(t_q, att_des(:,1), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\phi (deg)');
title('Roll: Actual vs Desired');
legend('Actual','Desired','Location','best');
grid on;

subplot(2,3,2);
plot(t_q, x_q(:,8)*180/pi, 'b-', 'LineWidth', 1.5); hold on;
plot(t_q, att_des(:,2), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\theta (deg)');
title('Pitch: Actual vs Desired');
legend('Actual','Desired','Location','best');
grid on;

subplot(2,3,3);
plot(t_q, x_q(:,9)*180/pi, 'b-', 'LineWidth', 1.5); hold on;
plot(t_q, att_des(:,3), 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\psi (deg)');
title('Yaw: Actual vs Desired');
legend('Actual','Desired','Location','best');
grid on;

subplot(2,3,4);
plot(t_q, att_err(:,1), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\phi error (deg)');
title('Roll Error');
grid on;

subplot(2,3,5);
plot(t_q, att_err(:,2), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\theta error (deg)');
title('Pitch Error');
grid on;

subplot(2,3,6);
plot(t_q, z3_norm*180/pi, 'k-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('||e_{att}|| (deg)');
title('Attitude Error Norm');
grid on;

sgtitle('Quadrotor Backstepping: Attitude Control Analysis', 'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, fullfile(output_dir, 'Quad_Attitude_Control.png'));
saveas(gcf, fullfile(output_dir, 'Quad_Attitude_Control.fig'));
fprintf('  Saved: Quad_Attitude_Control.png\n');

%% Figure 3: Control Effort Breakdown
figure('Name','Quadrotor: Control Effort','Position',[100 100 1400 800]);

subplot(2,3,1);
plot(t_q, thrust_q, 'b-', 'LineWidth', 1.5); hold on;
yline(quad.m*quad.g, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Thrust (N)');
title('Total Thrust');
legend('T','Hover (mg)','Location','best');
grid on;

subplot(2,3,2);
plot(t_q, thrust_ratio, 'k-', 'LineWidth', 1.5); hold on;
yline(1.0, 'r--', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('T / (mg)');
title('Thrust Ratio (normalized)');
ylim([0.99 1.01]);
grid on;

subplot(2,3,3);
plot(t_q, (thrust_q - quad.m*quad.g)*1000, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\DeltaT (mN)');
title('Thrust Deviation from Hover');
grid on;

subplot(2,3,4);
plot(t_q, torque_q(:,1)*1000, 'b-', 'LineWidth', 1.2); hold on;
plot(t_q, torque_q(:,2)*1000, 'r-', 'LineWidth', 1.2);
plot(t_q, torque_q(:,3)*1000, 'g-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Torque (mN\cdotm)');
title('Body Torques (milli-Newton-meters)');
legend('\tau_\phi (roll)','\tau_\theta (pitch)','\tau_\psi (yaw)','Location','best');
grid on;

subplot(2,3,5);
plot(t_q, ctrl_effort_thrust/1000, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\int T^2 dt (kN^2\cdots)');
title('Cumulative Thrust Effort');
grid on;

subplot(2,3,6);
plot(t_q, ctrl_effort_torque*1000, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('\int ||\tau||^2 dt (mN^2m^2\cdots)');
title('Cumulative Torque Effort');
grid on;

sgtitle('Quadrotor Backstepping: Control Effort Analysis', 'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, fullfile(output_dir, 'Quad_Control_Effort.png'));
saveas(gcf, fullfile(output_dir, 'Quad_Control_Effort.fig'));
fprintf('  Saved: Quad_Control_Effort.png\n');

%% Figure 4: Backstepping Error Surfaces (z1, z2, z3)
figure('Name','Quadrotor: Backstepping Errors','Position',[100 100 1200 400]);

subplot(1,3,1);
semilogy(t_q, z1_norm*1000 + 0.01, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('||z_1|| (mm, log scale)');
title('z_1: Position Error Surface');
grid on;

subplot(1,3,2);
semilogy(t_q, z2_norm + 1e-4, 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('||z_2|| (m/s, log scale)');
title('z_2: Velocity Error Surface');
grid on;

subplot(1,3,3);
semilogy(t_q, z3_norm*180/pi + 0.001, 'g-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('||z_3|| (deg, log scale)');
title('z_3: Attitude Error Surface');
grid on;

sgtitle('Backstepping Error Surfaces (Lyapunov Coordinates)', 'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, fullfile(output_dir, 'Quad_Backstepping_Errors.png'));
saveas(gcf, fullfile(output_dir, 'Quad_Backstepping_Errors.fig'));
fprintf('  Saved: Quad_Backstepping_Errors.png\n');

%% Figure 5: Error-to-Control Relationship
figure('Name','Quadrotor: Error-Control Correlation','Position',[100 100 1200 800]);

subplot(2,2,1);
scatter(pos_err(:,1)*1000, pos_err(:,2)*1000, 3, t_q, 'filled');
xlabel('e_x (mm)'); ylabel('e_y (mm)');
title('XY Position Error Phase Portrait');
colorbar; colormap(jet);
c = colorbar; c.Label.String = 'Time (s)';
grid on; axis equal;

subplot(2,2,2);
scatter(att_err(:,1), att_err(:,2), 3, t_q, 'filled');
xlabel('\phi error (deg)'); ylabel('\theta error (deg)');
title('Roll-Pitch Error Phase Portrait');
colorbar; colormap(jet);
c = colorbar; c.Label.String = 'Time (s)';
grid on; axis equal;

subplot(2,2,3);
plot(pos_err(:,1)*1000, torque_q(:,2)*1000, 'b.', 'MarkerSize', 2);
xlabel('e_x (mm)'); ylabel('\tau_\theta (mN\cdotm)');
title('X-error vs Pitch Torque');
grid on;

subplot(2,2,4);
plot(pos_err(:,2)*1000, torque_q(:,1)*1000, 'r.', 'MarkerSize', 2);
xlabel('e_y (mm)'); ylabel('\tau_\phi (mN\cdotm)');
title('Y-error vs Roll Torque');
grid on;

sgtitle('Error-to-Control Correlation Analysis', 'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, fullfile(output_dir, 'Quad_Error_Control_Correlation.png'));
saveas(gcf, fullfile(output_dir, 'Quad_Error_Control_Correlation.fig'));
fprintf('  Saved: Quad_Error_Control_Correlation.png\n');

%% Figure 6: Comprehensive Summary
figure('Name','Quadrotor: Summary','Position',[50 50 1600 900]);

subplot(3,4,1);
plot3(x_q(:,1), x_q(:,2), x_q(:,3), 'b-', 'LineWidth', 1.5); hold on;
plot3(pos_d_q(:,1), pos_d_q(:,2), pos_d_q(:,3), 'r--', 'LineWidth', 1);
xlabel('x'); ylabel('y'); zlabel('z');
title('3D Trajectory'); view(45,30); grid on;

subplot(3,4,2);
plot(t_q, z1_norm*1000, 'k-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('mm');
title('Position Error ||e||');
grid on;

subplot(3,4,3);
plot(t_q, thrust_q, 'b-', 'LineWidth', 1.2); hold on;
yline(quad.m*quad.g, 'r--');
xlabel('Time (s)'); ylabel('N');
title('Thrust');
grid on;

subplot(3,4,4);
plot(t_q, sqrt(sum(torque_q.^2,2))*1000, 'r-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('mN\cdotm');
title('Torque Magnitude');
grid on;

subplot(3,4,5);
plot(t_q, x_q(:,7)*180/pi, 'b-', 'LineWidth', 1.2); hold on;
plot(t_q, att_des(:,1), 'r--', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('deg');
title('\phi (roll)'); grid on;

subplot(3,4,6);
plot(t_q, x_q(:,8)*180/pi, 'b-', 'LineWidth', 1.2); hold on;
plot(t_q, att_des(:,2), 'r--', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('deg');
title('\theta (pitch)'); grid on;

subplot(3,4,7);
plot(t_q, att_err(:,1), 'b-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('deg');
title('\phi error'); grid on;

subplot(3,4,8);
plot(t_q, att_err(:,2), 'r-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('deg');
title('\theta error'); grid on;

subplot(3,4,9);
plot(t_q, pos_err(:,1)*1000, 'b-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('mm');
title('e_x'); grid on;

subplot(3,4,10);
plot(t_q, pos_err(:,2)*1000, 'r-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('mm');
title('e_y'); grid on;

subplot(3,4,11);
plot(t_q, pos_err(:,3)*1000, 'g-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('mm');
title('e_z'); grid on;

subplot(3,4,12);
V_total = 0.5*z1_norm.^2 + 0.5*z2_norm.^2 + 0.5*z3_norm.^2;
plot(t_q, V_total, 'k-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('V(t)');
title('Lyapunov Function'); grid on;

sgtitle('Quadrotor Backstepping Control: Comprehensive Summary', 'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, fullfile(output_dir, 'Quad_Summary.png'));
saveas(gcf, fullfile(output_dir, 'Quad_Summary.fig'));
fprintf('  Saved: Quad_Summary.png\n');

%% Print statistics
fprintf('\n--- Quadrotor Control Performance Statistics ---\n');
fprintf('  Position error (steady-state):\n');
fprintf('    Mean ||e_pos||: %.4f mm\n', mean(z1_norm(end-500:end))*1000);
fprintf('    Max  ||e_pos||: %.4f mm\n', max(z1_norm)*1000);
fprintf('  Attitude error (steady-state):\n');
fprintf('    Mean ||e_att||: %.4f deg\n', mean(z3_norm(end-500:end))*180/pi);
fprintf('    Max  ||e_att||: %.4f deg\n', max(z3_norm)*180/pi);
fprintf('  Thrust:\n');
fprintf('    Mean: %.4f N (hover=%.4f N)\n', mean(thrust_q), quad.m*quad.g);
fprintf('    Max deviation: %.4f mN\n', max(abs(thrust_q - quad.m*quad.g))*1000);
fprintf('  Torques:\n');
fprintf('    Max |tau_phi|:   %.4f mN*m\n', max(abs(torque_q(:,1)))*1000);
fprintf('    Max |tau_theta|: %.4f mN*m\n', max(abs(torque_q(:,2)))*1000);
fprintf('    Max |tau_psi|:   %.4f mN*m\n', max(abs(torque_q(:,3)))*1000);

fprintf('\n=== Week 7 Enhanced Quadrotor Analysis Complete ===\n');
fprintf('Output directory: %s\n', output_dir);
