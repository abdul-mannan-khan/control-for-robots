%% Week 7: SMC Control - Enhanced Drone Analysis
fprintf('=== Week 7: Enhanced SMC Drone Control Analysis ===\n\n');

% Run the main simulation first (for manipulator and mobile robot results)
run('smc_control_simulation.m');

% Define output directory
output_dir = '/home/it-services/auto_control_ws/results/week_07';

%% ========================================================================
%  ENHANCED DRONE SMC: Compare Multiple Boundary Layer Widths
%  ========================================================================
fprintf('\n--- Enhanced Drone SMC Analysis ---\n');

% Physical parameters (same as main simulation)
m_drone = 1.0;   g_drone = 9.81;
Ixx = 0.01;  Iyy = 0.01;  Izz = 0.02;
l_arm = 0.2;  c_drag = 0.01;

% SMC gains - position (outer loop)
lam_pos = [2; 2; 3];
eta_pos = [2; 2; 3];
k_pos   = [0.3; 0.3; 0.5];

% SMC gains - attitude (inner loop)
lam_att = [20; 20; 10];
eta_att = [5; 5; 3];

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

% Initial state (start at reference)
x0_drone = [xd_drone(0); yd_drone(0); zd_drone(0);  ...
            xd_d_drone(0); yd_d_drone(0); zd_d_drone(0);  ...
            0; 0; 0;  0; 0; 0];

dt_drone = 0.001;
t_end_drone = 20;  % Shorter for comparison

% Disturbances
dist_zero = @(t) zeros(6,1);
dist_wind = @(t) [1.5*sin(0.8*t); 1.5*cos(0.8*t); 0.5*sin(1.2*t); ...
                  0.05*sin(2*t); 0.05*cos(2*t); 0];

% Test multiple boundary layer widths
phi_pos_values = {[0.05; 0.05; 0.05], [0.2; 0.2; 0.2], [0.5; 0.5; 0.5]};
phi_att_values = {[0.02; 0.02; 0.02], [0.1; 0.1; 0.1], [0.3; 0.3; 0.3]};
phi_labels = {'Narrow (0.02-0.05)', 'Medium (0.1-0.2)', 'Wide (0.3-0.5)'};

results_drone = cell(3, 2);  % 3 phi values x 2 disturbance conditions

for phi_idx = 1:3
    phi_pos_i = phi_pos_values{phi_idx};
    phi_att_i = phi_att_values{phi_idx};

    % No disturbance
    fprintf('  Running Drone SMC (%s, no dist)...\n', phi_labels{phi_idx});
    [T_i, X_i, U_i] = run_drone_smc_enhanced(dt_drone, t_end_drone, x0_drone, ...
        m_drone, g_drone, Ixx, Iyy, Izz, l_arm, c_drag, ...
        lam_pos, eta_pos, k_pos, phi_pos_i, lam_att, eta_att, phi_att_i, ...
        xd_drone, yd_drone, zd_drone, xd_d_drone, yd_d_drone, zd_d_drone, ...
        xd_dd_drone, yd_dd_drone, zd_dd_drone, psi_d_drone, dist_zero);
    results_drone{phi_idx, 1} = struct('T', T_i, 'X', X_i, 'U', U_i, ...
        'phi_pos', phi_pos_i, 'phi_att', phi_att_i, 'label', phi_labels{phi_idx});

    % With wind disturbance
    fprintf('  Running Drone SMC (%s, with wind)...\n', phi_labels{phi_idx});
    [T_i, X_i, U_i] = run_drone_smc_enhanced(dt_drone, t_end_drone, x0_drone, ...
        m_drone, g_drone, Ixx, Iyy, Izz, l_arm, c_drag, ...
        lam_pos, eta_pos, k_pos, phi_pos_i, lam_att, eta_att, phi_att_i, ...
        xd_drone, yd_drone, zd_drone, xd_d_drone, yd_d_drone, zd_d_drone, ...
        xd_dd_drone, yd_dd_drone, zd_dd_drone, psi_d_drone, dist_wind);
    results_drone{phi_idx, 2} = struct('T', T_i, 'X', X_i, 'U', U_i, ...
        'phi_pos', phi_pos_i, 'phi_att', phi_att_i, 'label', phi_labels{phi_idx});
end

%% Figure 1: Trajectory Tracking Comparison
figure('Name', 'Drone SMC: Trajectory Comparison', 'Position', [50 100 1400 500]);

colors = {'b', 'r', 'g'};
for col = 1:2  % No dist, Wind
    subplot(1, 2, col);

    % Reference trajectory
    T_ref = results_drone{1, col}.T;
    xd_v = arrayfun(xd_drone, T_ref);
    yd_v = arrayfun(yd_drone, T_ref);
    zd_v = arrayfun(zd_drone, T_ref);
    plot3(xd_v, yd_v, zd_v, 'k--', 'LineWidth', 2); hold on;

    for phi_idx = 1:3
        res = results_drone{phi_idx, col};
        plot3(res.X(:,1), res.X(:,2), res.X(:,3), colors{phi_idx}, 'LineWidth', 1.2);
    end

    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    if col == 1
        title('3D Tracking - No Disturbance');
    else
        title('3D Tracking - With Wind');
    end
    legend('Reference', phi_labels{:}, 'Location', 'best');
    grid on; view(30, 25);
end

sgtitle('Drone SMC: Effect of Boundary Layer Width on Tracking', 'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, fullfile(output_dir, 'Drone_SMC_Trajectory_Comparison.png'));
saveas(gcf, fullfile(output_dir, 'Drone_SMC_Trajectory_Comparison.fig'));
fprintf('  Saved: Drone_SMC_Trajectory_Comparison.png\n');

%% Figure 2: Position Errors with Different Boundary Layers
figure('Name', 'Drone SMC: Position Errors', 'Position', [50 100 1400 800]);

for col = 1:2
    for axis_idx = 1:3
        subplot(2, 3, (col-1)*3 + axis_idx);

        for phi_idx = 1:3
            res = results_drone{phi_idx, col};
            T = res.T;

            if axis_idx == 1
                xd_v = arrayfun(xd_drone, T);
                err = res.X(:,1) - xd_v;
                ylabel_str = 'e_x (m)';
            elseif axis_idx == 2
                yd_v = arrayfun(yd_drone, T);
                err = res.X(:,2) - yd_v;
                ylabel_str = 'e_y (m)';
            else
                zd_v = arrayfun(zd_drone, T);
                err = res.X(:,3) - zd_v;
                ylabel_str = 'e_z (m)';
            end

            plot(T, err, colors{phi_idx}, 'LineWidth', 1.2); hold on;
        end

        xlabel('Time (s)'); ylabel(ylabel_str);
        if col == 1
            title(sprintf('%s Error (No Dist)', char('X' + axis_idx - 1)));
        else
            title(sprintf('%s Error (Wind)', char('X' + axis_idx - 1)));
        end
        legend(phi_labels{:}, 'Location', 'best');
        grid on;
    end
end

sgtitle('Position Errors: Boundary Layer Width Comparison', 'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, fullfile(output_dir, 'Drone_SMC_Position_Errors_Comparison.png'));
saveas(gcf, fullfile(output_dir, 'Drone_SMC_Position_Errors_Comparison.fig'));
fprintf('  Saved: Drone_SMC_Position_Errors_Comparison.png\n');

%% Figure 3: Control Inputs Comparison (Key Plot)
figure('Name', 'Drone SMC: Control Inputs', 'Position', [50 50 1600 900]);

ctrl_labels = {'Thrust (N)', 'Roll Torque (mNm)', 'Pitch Torque (mNm)', 'Yaw Torque (mNm)'};
ctrl_scales = [1, 1000, 1000, 1000];  % Convert torques to mNm for readability

for col = 1:2
    for ctrl_idx = 1:4
        subplot(2, 4, (col-1)*4 + ctrl_idx);

        for phi_idx = 1:3
            res = results_drone{phi_idx, col};
            ctrl_val = res.U(:, ctrl_idx) * ctrl_scales(ctrl_idx);
            plot(res.T, ctrl_val, colors{phi_idx}, 'LineWidth', 1); hold on;
        end

        xlabel('Time (s)');
        ylabel(ctrl_labels{ctrl_idx});

        if col == 1
            if ctrl_idx == 1
                title('Thrust (No Dist)');
                yline(m_drone * g_drone, 'k--', 'LineWidth', 1);  % Hover thrust
            else
                title(sprintf('%s (No Dist)', ctrl_labels{ctrl_idx}));
            end
        else
            if ctrl_idx == 1
                title('Thrust (Wind)');
                yline(m_drone * g_drone, 'k--', 'LineWidth', 1);
            else
                title(sprintf('%s (Wind)', ctrl_labels{ctrl_idx}));
            end
        end

        if ctrl_idx == 1
            legend([phi_labels, {'Hover'}], 'Location', 'best');
        else
            legend(phi_labels{:}, 'Location', 'best');
        end
        grid on;
    end
end

sgtitle('Control Inputs: Effect of Boundary Layer Width (Torques in mNm)', 'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, fullfile(output_dir, 'Drone_SMC_Control_Comparison.png'));
saveas(gcf, fullfile(output_dir, 'Drone_SMC_Control_Comparison.fig'));
fprintf('  Saved: Drone_SMC_Control_Comparison.png\n');

%% Figure 4: Control Effort Analysis
figure('Name', 'Drone SMC: Control Effort', 'Position', [50 100 1400 500]);

subplot(1,3,1);
for phi_idx = 1:3
    res_no = results_drone{phi_idx, 1};
    res_wd = results_drone{phi_idx, 2};

    % RMS torque (roll + pitch)
    rms_no = sqrt(mean(res_no.U(:,2).^2 + res_no.U(:,3).^2)) * 1000;
    rms_wd = sqrt(mean(res_wd.U(:,2).^2 + res_wd.U(:,3).^2)) * 1000;

    bar_data(phi_idx, :) = [rms_no, rms_wd];
end
bar(bar_data);
set(gca, 'XTickLabel', phi_labels);
ylabel('RMS Torque (mNm)');
title('RMS Lateral Torque');
legend('No Dist', 'Wind', 'Location', 'best');
grid on;

subplot(1,3,2);
for phi_idx = 1:3
    res_no = results_drone{phi_idx, 1};
    res_wd = results_drone{phi_idx, 2};

    T = res_no.T;
    xd_v = arrayfun(xd_drone, T);
    yd_v = arrayfun(yd_drone, T);
    zd_v = arrayfun(zd_drone, T);

    rms_no = sqrt(mean((res_no.X(:,1)-xd_v).^2 + (res_no.X(:,2)-yd_v).^2 + (res_no.X(:,3)-zd_v).^2)) * 1000;

    T = res_wd.T;
    xd_v = arrayfun(xd_drone, T);
    yd_v = arrayfun(yd_drone, T);
    zd_v = arrayfun(zd_drone, T);
    rms_wd = sqrt(mean((res_wd.X(:,1)-xd_v).^2 + (res_wd.X(:,2)-yd_v).^2 + (res_wd.X(:,3)-zd_v).^2)) * 1000;

    bar_data2(phi_idx, :) = [rms_no, rms_wd];
end
bar(bar_data2);
set(gca, 'XTickLabel', phi_labels);
ylabel('RMS Position Error (mm)');
title('RMS Position Error');
legend('No Dist', 'Wind', 'Location', 'best');
grid on;

subplot(1,3,3);
% Cumulative control effort
for phi_idx = 1:3
    res = results_drone{phi_idx, 2};  % Wind case
    cum_effort = cumtrapz(res.T, res.U(:,2).^2 + res.U(:,3).^2) * 1e6;  % mNm^2*s
    plot(res.T, cum_effort, colors{phi_idx}, 'LineWidth', 1.5); hold on;
end
xlabel('Time (s)'); ylabel('Cumulative Torque Effort (mNm^2 \cdot s)');
title('Cumulative Torque Effort (Wind Case)');
legend(phi_labels{:}, 'Location', 'best');
grid on;

sgtitle('SMC Control Effort Analysis: Chattering vs Tracking Trade-off', 'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, fullfile(output_dir, 'Drone_SMC_Effort_Analysis.png'));
saveas(gcf, fullfile(output_dir, 'Drone_SMC_Effort_Analysis.fig'));
fprintf('  Saved: Drone_SMC_Effort_Analysis.png\n');

%% Figure 5: Attitude Response
figure('Name', 'Drone SMC: Attitude', 'Position', [50 100 1400 500]);

att_labels = {'\phi (roll)', '\theta (pitch)', '\psi (yaw)'};
for att_idx = 1:3
    subplot(1, 3, att_idx);

    % Wind disturbance case (more interesting)
    for phi_idx = 1:3
        res = results_drone{phi_idx, 2};
        plot(res.T, res.X(:, 6+att_idx) * 180/pi, colors{phi_idx}, 'LineWidth', 1.2); hold on;
    end

    xlabel('Time (s)'); ylabel([att_labels{att_idx}, ' (deg)']);
    title([att_labels{att_idx}, ' Response (Wind)']);
    legend(phi_labels{:}, 'Location', 'best');
    grid on;
end

sgtitle('Attitude Response Under Wind Disturbance', 'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, fullfile(output_dir, 'Drone_SMC_Attitude.png'));
saveas(gcf, fullfile(output_dir, 'Drone_SMC_Attitude.fig'));
fprintf('  Saved: Drone_SMC_Attitude.png\n');

%% Figure 6: Comprehensive Summary
figure('Name', 'Drone SMC: Summary', 'Position', [50 50 1600 900]);

% Use medium boundary layer for main display
res_mid_no = results_drone{2, 1};
res_mid_wd = results_drone{2, 2};

subplot(3,4,1);
T_ref = res_mid_wd.T;
xd_v = arrayfun(xd_drone, T_ref);
yd_v = arrayfun(yd_drone, T_ref);
zd_v = arrayfun(zd_drone, T_ref);
plot3(xd_v, yd_v, zd_v, 'r--', 'LineWidth', 1.5); hold on;
plot3(res_mid_wd.X(:,1), res_mid_wd.X(:,2), res_mid_wd.X(:,3), 'b-', 'LineWidth', 1.2);
xlabel('x'); ylabel('y'); zlabel('z');
title('3D Trajectory (Medium BL, Wind)');
legend('Ref', 'Actual'); grid on; view(30, 25);

subplot(3,4,2);
err_norm = sqrt((res_mid_wd.X(:,1)-xd_v).^2 + (res_mid_wd.X(:,2)-yd_v).^2 + (res_mid_wd.X(:,3)-zd_v).^2);
plot(res_mid_wd.T, err_norm * 1000, 'b-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('||e|| (mm)');
title('Position Error Norm'); grid on;

subplot(3,4,3);
plot(res_mid_wd.T, res_mid_wd.U(:,1), 'b-', 'LineWidth', 1.2); hold on;
yline(m_drone * g_drone, 'r--');
xlabel('Time (s)'); ylabel('T (N)');
title('Thrust'); legend('T', 'mg'); grid on;

subplot(3,4,4);
plot(res_mid_wd.T, sqrt(res_mid_wd.U(:,2).^2 + res_mid_wd.U(:,3).^2)*1000, 'b-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('||\tau_{lat}|| (mNm)');
title('Lateral Torque Magnitude'); grid on;

subplot(3,4,5);
plot(res_mid_wd.T, res_mid_wd.X(:,7)*180/pi, 'b-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('\phi (deg)');
title('Roll Angle'); grid on;

subplot(3,4,6);
plot(res_mid_wd.T, res_mid_wd.X(:,8)*180/pi, 'b-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('\theta (deg)');
title('Pitch Angle'); grid on;

subplot(3,4,7);
plot(res_mid_wd.T, res_mid_wd.U(:,2)*1000, 'b-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('\tau_\phi (mNm)');
title('Roll Torque'); grid on;

subplot(3,4,8);
plot(res_mid_wd.T, res_mid_wd.U(:,3)*1000, 'r-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('\tau_\theta (mNm)');
title('Pitch Torque'); grid on;

subplot(3,4,9);
plot(res_mid_wd.T, res_mid_wd.X(:,1) - xd_v, 'b-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('e_x (m)');
title('X Error'); grid on;

subplot(3,4,10);
plot(res_mid_wd.T, res_mid_wd.X(:,2) - yd_v, 'r-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('e_y (m)');
title('Y Error'); grid on;

subplot(3,4,11);
plot(res_mid_wd.T, res_mid_wd.X(:,3) - zd_v, 'g-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('e_z (m)');
title('Z Error'); grid on;

subplot(3,4,12);
% Sliding surface norm (position)
s_norm = zeros(length(res_mid_wd.T), 1);
for i = 1:length(res_mid_wd.T)
    t = res_mid_wd.T(i);
    e_pos = res_mid_wd.X(i,1:3)' - [xd_drone(t); yd_drone(t); zd_drone(t)];
    ed_pos = res_mid_wd.X(i,4:6)' - [xd_d_drone(t); yd_d_drone(t); zd_d_drone(t)];
    s = ed_pos + lam_pos .* e_pos;
    s_norm(i) = norm(s);
end
plot(res_mid_wd.T, s_norm, 'k-', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('||s_{pos}||');
title('Position Sliding Surface'); grid on;

sgtitle('Quadrotor SMC Summary (Medium Boundary Layer, Wind Disturbance)', 'FontSize', 14, 'FontWeight', 'bold');
saveas(gcf, fullfile(output_dir, 'Drone_SMC_Summary.png'));
saveas(gcf, fullfile(output_dir, 'Drone_SMC_Summary.fig'));
fprintf('  Saved: Drone_SMC_Summary.png\n');

%% Print Statistics
fprintf('\n--- Drone SMC Performance Statistics ---\n');
fprintf('%-25s  %12s  %12s  %15s  %15s\n', 'Boundary Layer', 'RMS Err (mm)', 'Max Err (mm)', 'RMS Torque(mNm)', 'Max Torque(mNm)');
fprintf('%s\n', repmat('-', 1, 85));

for phi_idx = 1:3
    res = results_drone{phi_idx, 2};  % Wind case
    T = res.T;
    xd_v = arrayfun(xd_drone, T);
    yd_v = arrayfun(yd_drone, T);
    zd_v = arrayfun(zd_drone, T);

    err = sqrt((res.X(:,1)-xd_v).^2 + (res.X(:,2)-yd_v).^2 + (res.X(:,3)-zd_v).^2);
    rms_err = sqrt(mean(err.^2)) * 1000;
    max_err = max(err) * 1000;

    tau_lat = sqrt(res.U(:,2).^2 + res.U(:,3).^2);
    rms_tau = sqrt(mean(tau_lat.^2)) * 1000;
    max_tau = max(tau_lat) * 1000;

    fprintf('%-25s  %12.2f  %12.2f  %15.2f  %15.2f\n', ...
        phi_labels{phi_idx}, rms_err, max_err, rms_tau, max_tau);
end

fprintf('\n=== Week 7 Enhanced Drone SMC Analysis Complete ===\n');
fprintf('Output directory: %s\n', output_dir);

%% ========================================================================
%  HELPER FUNCTION: Enhanced Drone SMC Simulation
%  ========================================================================
function [T, X, U_hist] = run_drone_smc_enhanced(dt, t_end, x0, ...
    m, g_val, Ixx, Iyy, Izz, l_arm, c_drag, ...
    lam_pos, eta_pos, k_pos, phi_pos, ...
    lam_att, eta_att, phi_att, ...
    xd_f, yd_f, zd_f, xd_d_f, yd_d_f, zd_d_f, ...
    xd_dd_f, yd_dd_f, zd_dd_f, psi_d_f, ...
    dist_fun)

    odefun = @(t, x) drone_ode_enhanced(t, x, m, g_val, Ixx, Iyy, Izz, l_arm, c_drag, ...
        lam_pos, eta_pos, k_pos, phi_pos, ...
        lam_att, eta_att, phi_att, ...
        xd_f, yd_f, zd_f, xd_d_f, yd_d_f, zd_d_f, ...
        xd_dd_f, yd_dd_f, zd_dd_f, psi_d_f, dist_fun);

    % RK4 integration
    N_steps = ceil(t_end / dt);
    T = (0:N_steps)' * dt;
    X = zeros(N_steps+1, length(x0));
    X(1,:) = x0';

    for k = 1:N_steps
        t_k = T(k);
        x_k = X(k,:)';

        k1 = odefun(t_k, x_k);
        k2 = odefun(t_k + dt/2, x_k + dt/2 * k1);
        k3 = odefun(t_k + dt/2, x_k + dt/2 * k2);
        k4 = odefun(t_k + dt, x_k + dt * k3);

        X(k+1,:) = (x_k + dt/6 * (k1 + 2*k2 + 2*k3 + k4))';
    end

    % Subsample and compute control
    step = 20;
    idx = 1:step:length(T);
    T = T(idx);
    X = X(idx,:);
    N = length(idx);
    U_hist = zeros(N, 4);

    for i = 1:N
        [~, U_i] = drone_ode_enhanced(T(i), X(i,:)', m, g_val, Ixx, Iyy, Izz, l_arm, c_drag, ...
            lam_pos, eta_pos, k_pos, phi_pos, ...
            lam_att, eta_att, phi_att, ...
            xd_f, yd_f, zd_f, xd_d_f, yd_d_f, zd_d_f, ...
            xd_dd_f, yd_dd_f, zd_dd_f, psi_d_f, dist_fun);
        U_hist(i,:) = U_i';
    end
end

function [dxdt, U_ctrl] = drone_ode_enhanced(t, x, m, g_val, Ixx, Iyy, Izz, l_arm, c_drag, ...
    lam_pos, eta_pos, k_pos, phi_pos, ...
    lam_att, eta_att, phi_att, ...
    xd_f, yd_f, zd_f, xd_d_f, yd_d_f, zd_d_f, ...
    xd_dd_f, yd_dd_f, zd_dd_f, psi_d_f, dist_fun)

    % Unpack state
    pos  = x(1:3);    vel  = x(4:6);
    att  = x(7:9);    datt = x(10:12);

    phi_a   = att(1);  theta_a = att(2);  psi_a = att(3);
    dphi_a  = datt(1); dtheta_a= datt(2); dpsi_a = datt(3);

    % Disturbance
    d = dist_fun(t);
    d_pos = d(1:3);
    d_att = d(4:6);

    % Desired trajectory
    xd  = xd_f(t);    yd  = yd_f(t);    zd  = zd_f(t);
    dxd = xd_d_f(t);  dyd = yd_d_f(t);  dzd = zd_d_f(t);
    ddxd= xd_dd_f(t); ddyd= yd_dd_f(t); ddzd= zd_dd_f(t);
    psi_d = psi_d_f(t);

    % ---- OUTER LOOP: Position SMC ----
    e_pos  = pos - [xd; yd; zd];
    ed_pos = vel - [dxd; dyd; dzd];
    s_pos = ed_pos + lam_pos .* e_pos;

    % Saturation function for boundary layer
    s_sat_pos = zeros(3,1);
    for i = 1:3
        if abs(s_pos(i)) <= phi_pos(i)
            s_sat_pos(i) = s_pos(i) / phi_pos(i);
        else
            s_sat_pos(i) = sign(s_pos(i));
        end
    end

    accel_cmd = [ddxd; ddyd; ddzd] - lam_pos .* ed_pos ...
                - eta_pos .* s_sat_pos - k_pos .* s_pos;

    % Thrust
    cth = cos(theta_a);  cphi = cos(phi_a);
    denom_z = cth * cphi;
    if abs(denom_z) < 0.1
        denom_z = 0.1 * sign(denom_z + 1e-10);
    end
    U1 = m * (accel_cmd(3) + g_val) / denom_z;
    U1 = max(U1, 0);

    % Desired angles
    accel_z_cmd = accel_cmd(3) + g_val;
    if abs(accel_z_cmd) < 0.1
        accel_z_cmd = 0.1;
    end

    accel_lat_max = 5.0;
    accel_cmd(1) = max(min(accel_cmd(1), accel_lat_max), -accel_lat_max);
    accel_cmd(2) = max(min(accel_cmd(2), accel_lat_max), -accel_lat_max);

    theta_d = atan2(accel_cmd(1)*cos(psi_a) + accel_cmd(2)*sin(psi_a), accel_z_cmd);
    phi_d   = atan2((accel_cmd(1)*sin(psi_a) - accel_cmd(2)*cos(psi_a)) * cos(theta_d), accel_z_cmd);

    angle_max = 0.47;
    theta_d = max(min(theta_d, angle_max), -angle_max);
    phi_d   = max(min(phi_d, angle_max), -angle_max);

    dphi_d = 0;  dtheta_d = 0;  dpsi_d = 0;
    ddphi_d = 0; ddtheta_d = 0; ddpsi_d = 0;

    % ---- INNER LOOP: Attitude SMC ----
    e_att  = att - [phi_d; theta_d; psi_d];
    ed_att = datt - [dphi_d; dtheta_d; dpsi_d];
    s_att = ed_att + lam_att .* e_att;

    s_sat_att = zeros(3,1);
    for i = 1:3
        if abs(s_att(i)) <= phi_att(i)
            s_sat_att(i) = s_att(i) / phi_att(i);
        else
            s_sat_att(i) = sign(s_att(i));
        end
    end

    % Gyroscopic terms
    gyro_phi   = (Iyy - Izz) / Ixx * dtheta_a * dpsi_a;
    gyro_theta = (Izz - Ixx) / Iyy * dphi_a * dpsi_a;
    gyro_psi   = (Ixx - Iyy) / Izz * dphi_a * dtheta_a;

    % Torques
    U2 = Ixx / l_arm * (ddphi_d   - lam_att(1)*ed_att(1) - gyro_phi   - eta_att(1)*s_sat_att(1));
    U3 = Iyy / l_arm * (ddtheta_d - lam_att(2)*ed_att(2) - gyro_theta - eta_att(2)*s_sat_att(2));
    U4 = Izz / c_drag * (ddpsi_d  - lam_att(3)*ed_att(3) - gyro_psi   - eta_att(3)*s_sat_att(3));

    U_ctrl = [U1; U2; U3; U4];

    % Dynamics
    ax = (cos(psi_a)*sin(theta_a)*cos(phi_a) + sin(psi_a)*sin(phi_a)) * U1 / m + d_pos(1)/m;
    ay = (sin(psi_a)*sin(theta_a)*cos(phi_a) - cos(psi_a)*sin(phi_a)) * U1 / m + d_pos(2)/m;
    az = cos(theta_a)*cos(phi_a) * U1 / m - g_val + d_pos(3)/m;

    ddphi   = gyro_phi   + l_arm / Ixx * U2 + d_att(1)/Ixx;
    ddtheta = gyro_theta + l_arm / Iyy * U3 + d_att(2)/Iyy;
    ddpsi   = gyro_psi   + c_drag / Izz * U4 + d_att(3)/Izz;

    dxdt = [vel; ax; ay; az; datt; ddphi; ddtheta; ddpsi];
end
