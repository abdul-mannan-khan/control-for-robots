function results = sim_smc(saveFig)
% SLIDING MODE CONTROL with optimization of (lambda, phi, eta)
% Applied to: 2-link manipulator, unicycle mobile robot, planar quadrotor
% Cost minimized: J = wE*integ(e^2) + wU*integ(u^2) + wChat*chatter_metric

if nargin < 1, saveFig = true; end
results = struct();

fprintf('\n=== SLIDING MODE CONTROL OPTIMIZATION ===\n');

%% --- 1. MANIPULATOR ---
fprintf('[SMC] Manipulator...\n');
p = manipulator_params();
q_des = [pi/3; -pi/4];
T = 3.0; dt = 0.002; N = round(T/dt);

cost_manip = @(z) smc_manip_cost(z, p, q_des, T, dt);
z0 = [10, 0.05, 5];        % [lambda, phi, eta] initial
zOpt = fminsearch(cost_manip, z0, optimset('Display','off','MaxIter',80,'TolX',1e-3));

[t1, Q1, U1, ~] = run_smc_manip(z0,  p, q_des, T, dt);
[t2, Q2, U2, ~] = run_smc_manip(zOpt, p, q_des, T, dt);
results.manipulator.z0 = z0; results.manipulator.zOpt = zOpt;
results.manipulator.J0 = cost_manip(z0); results.manipulator.JOpt = cost_manip(zOpt);
fprintf('  lambda: %.2f->%.2f, phi: %.3f->%.3f, eta: %.2f->%.2f | J: %.3f->%.3f\n',...
    z0(1),zOpt(1),z0(2),zOpt(2),z0(3),zOpt(3), results.manipulator.J0, results.manipulator.JOpt);

%% --- 2. MOBILE ROBOT (unicycle, trajectory tracking) ---
fprintf('[SMC] Mobile robot...\n');
T = 10; dt = 0.02;
cost_mob = @(z) smc_mobile_cost(z, T, dt);
z0m = [2, 0.1, 1];
zOm = fminsearch(cost_mob, z0m, optimset('Display','off','MaxIter',60,'TolX',1e-3));
[tm1, Xm1, Um1] = run_smc_mobile(z0m, T, dt);
[tm2, Xm2, Um2] = run_smc_mobile(zOm, T, dt);
results.mobile.z0=z0m; results.mobile.zOpt=zOm;
results.mobile.J0 = cost_mob(z0m); results.mobile.JOpt = cost_mob(zOm);
fprintf('  J: %.3f->%.3f\n', results.mobile.J0, results.mobile.JOpt);

%% --- 3. QUADROTOR (altitude + attitude SMC) ---
fprintf('[SMC] Quadrotor...\n');
pq = quadrotor_params();
T = 4; dt = 0.005;
cost_q = @(z) smc_quad_cost(z, pq, T, dt);
z0q = [4, 0.05, 2];
zOq = fminsearch(cost_q, z0q, optimset('Display','off','MaxIter',60,'TolX',1e-3));
[tq1, Xq1, Uq1] = run_smc_quad(z0q, pq, T, dt);
[tq2, Xq2, Uq2] = run_smc_quad(zOq, pq, T, dt);
results.quad.z0=z0q; results.quad.zOpt=zOq;
results.quad.J0 = cost_q(z0q); results.quad.JOpt = cost_q(zOq);
fprintf('  J: %.3f->%.3f\n', results.quad.J0, results.quad.JOpt);

%% --- Plot ---
f = figure('Visible','off','Position',[100 100 1500 950]);
lbl_base = sprintf('SMC hand-tuned (\\lambda=%.1f,\\phi=%.2f,\\eta=%.1f)',z0(1),z0(2),z0(3));
lbl_opt  = sprintf('SMC optimized (\\lambda=%.1f,\\phi=%.2f,\\eta=%.1f)',zOpt(1),zOpt(2),zOpt(3));
ref_tref = '--- reference / setpoint';

subplot(3,3,1); plot(t1,Q1(:,1),'r--','LineWidth',1.4,'DisplayName',lbl_base); hold on;
plot(t2,Q2(:,1),'b-','LineWidth',1.4,'DisplayName',lbl_opt);
yline(q_des(1),'k:','LineWidth',1.2,'DisplayName','q_{1,des}');
grid on; title('Manipulator joint angle q_1'); ylabel('q_1 [rad]'); xlabel('t [s]');
legend('Location','southeast','FontSize',7);

subplot(3,3,2); plot(t1,Q1(:,2),'r--','LineWidth',1.4,'DisplayName','baseline SMC'); hold on;
plot(t2,Q2(:,2),'b-','LineWidth',1.4,'DisplayName','optimized SMC');
yline(q_des(2),'k:','LineWidth',1.2,'DisplayName','q_{2,des}');
grid on; title('Manipulator joint angle q_2'); ylabel('q_2 [rad]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,3); plot(t1,U1(:,1),'r--','LineWidth',1.2,'DisplayName','baseline \tau_1'); hold on;
plot(t2,U2(:,1),'b-','LineWidth',1.2,'DisplayName','optimized \tau_1');
grid on; title('Manipulator torque \tau_1 (chatter reduced when optimized)');
ylabel('\tau_1 [N\cdotm]'); xlabel('t [s]'); legend('Location','best','FontSize',7);

subplot(3,3,4); tref=linspace(0,2*pi,200);
plot(cos(tref),sin(tref),'k:','LineWidth',1.3,'DisplayName','reference circle'); hold on;
plot(Xm1(:,1),Xm1(:,2),'r--','LineWidth',1.4,'DisplayName','baseline SMC');
plot(Xm2(:,1),Xm2(:,2),'b-','LineWidth',1.4,'DisplayName','optimized SMC');
grid on; axis equal; title('Mobile robot XY trajectory');
xlabel('x [m]'); ylabel('y [m]'); legend('Location','best','FontSize',7);

subplot(3,3,5); plot(tm1,Um1(:,1),'r--','LineWidth',1.2,'DisplayName','baseline v'); hold on;
plot(tm2,Um2(:,1),'b-','LineWidth',1.2,'DisplayName','optimized v');
grid on; title('Mobile linear velocity v'); ylabel('v [m/s]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,6); plot(tm1,Um1(:,2),'r--','LineWidth',1.2,'DisplayName','baseline \omega'); hold on;
plot(tm2,Um2(:,2),'b-','LineWidth',1.2,'DisplayName','optimized \omega');
grid on; title('Mobile angular velocity \omega'); ylabel('\omega [rad/s]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,7); plot(tq1,Xq1(:,2),'r--','LineWidth',1.4,'DisplayName','baseline SMC'); hold on;
plot(tq2,Xq2(:,2),'b-','LineWidth',1.4,'DisplayName','optimized SMC');
yline(1.0,'k:','LineWidth',1.2,'DisplayName','z_{des}=1m');
grid on; title('Quadrotor altitude z'); ylabel('z [m]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,8); plot(tq1,Xq1(:,3),'r--','LineWidth',1.4,'DisplayName','baseline SMC'); hold on;
plot(tq2,Xq2(:,3),'b-','LineWidth',1.4,'DisplayName','optimized SMC');
yline(0,'k:','LineWidth',1.0,'DisplayName','\phi_{des}=0');
grid on; title('Quadrotor roll \phi'); ylabel('\phi [rad]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,9); plot(tq1,Uq1(:,1),'r--','LineWidth',1.2,'DisplayName','baseline thrust'); hold on;
plot(tq2,Uq2(:,1),'b-','LineWidth',1.2,'DisplayName','optimized thrust');
yline(pq.m*pq.g,'k:','LineWidth',1.0,'DisplayName','hover thrust mg');
grid on; title('Quadrotor thrust T'); ylabel('T [N]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

sgtitle({'SLIDING MODE CONTROL  —  red dashed = hand-tuned gains,  blue solid = fminsearch-optimized gains,  black dotted = reference/setpoint',...
    sprintf('Cost reductions: manipulator %.1f%%, mobile %.1f%%, quadrotor %.1f%%',...
    100*(results.manipulator.J0-results.manipulator.JOpt)/results.manipulator.J0,...
    100*(results.mobile.J0-results.mobile.JOpt)/results.mobile.J0,...
    100*(results.quad.J0-results.quad.JOpt)/results.quad.J0)},...
    'FontSize',10,'FontWeight','bold');
if saveFig
    saveas(f, fullfile(fileparts(mfilename('fullpath')),'figures','smc_results.png'));
end
close(f);
end

% ======= Manipulator SMC helpers =======
function [t,Q,U,s_log] = run_smc_manip(z,p,q_des,T,dt)
lambda=z(1); phi=z(2); eta=z(3);
N=round(T/dt); t=(0:N-1)'*dt;
q=[0;0]; dq=[0;0];
Q=zeros(N,2); U=zeros(N,2); s_log=zeros(N,2);
for k=1:N
    e = q - q_des; de = dq;
    s = de + lambda*e;
    [M,C,G] = manipulator_dyn(q,dq,p);
    % u = -Mhat*(lambda*de) + C*dq + G - eta*sat(s/phi)   (feedback-linearization-based SMC)
    u = M*(-lambda*de) + C*dq + G - eta*sat(s./phi);
    ddq = M \ (u - C*dq - G);
    dq = dq + dt*ddq; q = q + dt*dq;
    Q(k,:)=q'; U(k,:)=u'; s_log(k,:)=s';
end
end

function J = smc_manip_cost(z,p,q_des,T,dt)
if any(z<=0) || z(1)>60 || z(2)>0.5 || z(3)>40, J=1e6; return; end
[~,Q,U,~]=run_smc_manip(z,p,q_des,T,dt);
e = Q - q_des';
chatter = sum(sum(abs(diff(U)))) * dt;
J = sum(sum(e.^2))*dt + 0.001*sum(sum(U.^2))*dt + 0.01*chatter;
end

% ======= Mobile SMC (trajectory: reference lemniscate) =======
function [t,X,U] = run_smc_mobile(z,T,dt)
lambda=z(1); phi=z(2); eta=z(3);
N=round(T/dt); t=(0:N-1)'*dt;
x=[0;0;0];
X=zeros(N,3); U=zeros(N,2);
for k=1:N
    tk = t(k);
    % reference: circle
    xr=[cos(0.3*tk); sin(0.3*tk); 0.3*tk+pi/2];
    dxr=[-0.3*sin(0.3*tk); 0.3*cos(0.3*tk); 0.3];
    % world-frame error
    ep = xr(1:2)-x(1:2);
    % rotate into body frame
    R=[cos(x(3)) sin(x(3)); -sin(x(3)) cos(x(3))];
    eb=R*ep; etheta = wrap(xr(3)-x(3));
    vref = norm(dxr(1:2));
    % sliding variables
    s1 = eb(1); s2 = etheta + lambda*eb(2);
    v = vref*cos(etheta) + eta*sat(s1/phi);
    omega = dxr(3) + eta*sat(s2/phi) + 2*lambda*eb(2);
    u=[v;omega];
    dx = unicycle_dyn(x,u);
    x = x + dt*dx; x(3)=wrap(x(3));
    X(k,:)=x'; U(k,:)=u';
end
end
function J = smc_mobile_cost(z,T,dt)
if any(z<=0) || z(1)>10 || z(2)>1 || z(3)>5, J=1e6; return; end
[t,X,U]=run_smc_mobile(z,T,dt);
xr=[cos(0.3*t) sin(0.3*t)];
e = X(:,1:2)-xr;
chatter = sum(sum(abs(diff(U))))*dt;
J = sum(sum(e.^2))*dt + 0.01*sum(sum(U.^2))*dt + 0.02*chatter;
end

% ======= Quadrotor SMC =======
function [t,X,U] = run_smc_quad(z,p,T,dt)
lambda=z(1); phi=z(2); eta=z(3);
N=round(T/dt); t=(0:N-1)'*dt;
x=[0;0;0;0;0;0];
z_des=1.0;       % altitude target
X=zeros(N,6); U=zeros(N,2);
for k=1:N
    ez = x(2)-z_des; dez = x(5);
    ephi = x(3); dephi = x(6);
    % altitude SMC (via thrust)
    sz = dez + lambda*ez;
    T_cmd = p.m*(p.g - lambda*dez - eta*sat(sz/phi));
    T_cmd = max(0.1, T_cmd);
    % attitude SMC
    sphi = dephi + lambda*ephi;
    tau = p.J*(-lambda*dephi - eta*sat(sphi/phi));
    u=[T_cmd; tau];
    dx = quadrotor_dyn(x,u,p);
    x = x + dt*dx;
    X(k,:)=x'; U(k,:)=u';
end
end
function J = smc_quad_cost(z,p,T,dt)
if any(z<=0) || z(1)>15 || z(2)>0.5 || z(3)>10, J=1e6; return; end
[~,X,U]=run_smc_quad(z,p,T,dt);
ez = X(:,2)-1.0; ephi = X(:,3);
chatter = sum(abs(diff(U(:,2))))*dt;
J = sum(ez.^2)*dt + 0.5*sum(ephi.^2)*dt + 0.001*sum(sum(U.^2))*dt + 0.05*chatter;
end

function y = sat(s)
y = max(-1, min(1, s));
end
function y = wrap(a)
y = mod(a+pi, 2*pi) - pi;
end
