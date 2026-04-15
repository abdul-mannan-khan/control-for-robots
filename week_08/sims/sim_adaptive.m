function results = sim_adaptive(saveFig)
% ADAPTIVE CONTROL optimization of adaptation gain Gamma and sigma-mod.
% Applied to manipulator (MRAC), mobile robot (adaptive trajectory tracking),
% quadrotor (adaptive mass estimation for altitude).

if nargin < 1, saveFig = true; end
results = struct();
fprintf('\n=== ADAPTIVE CONTROL OPTIMIZATION ===\n');

%% ------------- MANIPULATOR (MRAC-style) -------------
fprintf('[ADA] Manipulator...\n');
p = manipulator_params();
p_true = p; p_true.m2 = 1.5;   % unknown payload added
q_des = [pi/3; -pi/4]; T=4; dt=0.002;
cost_m = @(z) ada_manip_cost(z, p, p_true, q_des, T, dt);
z0=[5, 0.01];                  % [Gamma, sigma]
zOpt = fminsearch(cost_m, z0, optimset('Display','off','MaxIter',60));
[t1,Q1,U1,Th1] = run_ada_manip(z0, p, p_true, q_des, T, dt);
[t2,Q2,U2,Th2] = run_ada_manip(zOpt, p, p_true, q_des, T, dt);
results.manipulator.z0=z0; results.manipulator.zOpt=zOpt;
results.manipulator.J0=cost_m(z0); results.manipulator.JOpt=cost_m(zOpt);
fprintf('  Gamma: %.2f->%.2f, sigma: %.3f->%.3f | J: %.3f->%.3f\n',...
    z0(1),zOpt(1),z0(2),zOpt(2),results.manipulator.J0,results.manipulator.JOpt);

%% ------------- MOBILE (adaptive to unknown wheel-radius scale) -------------
fprintf('[ADA] Mobile robot...\n');
T=8; dt=0.02;
cost_mob = @(z) ada_mobile_cost(z,T,dt);
z0m=[2, 0.01];
zOm = fminsearch(cost_mob,z0m,optimset('Display','off','MaxIter',60));
[tm1,Xm1,Um1,~] = run_ada_mobile(z0m,T,dt);
[tm2,Xm2,Um2,~] = run_ada_mobile(zOm,T,dt);
results.mobile.z0=z0m; results.mobile.zOpt=zOm;
results.mobile.J0=cost_mob(z0m); results.mobile.JOpt=cost_mob(zOm);
fprintf('  Gamma: %.2f->%.2f | J: %.3f->%.3f\n',z0m(1),zOm(1),results.mobile.J0,results.mobile.JOpt);

%% ------------- QUADROTOR (adaptive mass in altitude loop) -------------
fprintf('[ADA] Quadrotor...\n');
pq = quadrotor_params(); pq_true = pq; pq_true.m = 0.65;
T=5; dt=0.005;
cost_q = @(z) ada_quad_cost(z,pq,pq_true,T,dt);
z0q=[3,0.02];
zOq = fminsearch(cost_q,z0q,optimset('Display','off','MaxIter',60));
[tq1,Xq1,Uq1,Mh1] = run_ada_quad(z0q,pq,pq_true,T,dt);
[tq2,Xq2,Uq2,Mh2] = run_ada_quad(zOq,pq,pq_true,T,dt);
results.quad.z0=z0q; results.quad.zOpt=zOq;
results.quad.J0=cost_q(z0q); results.quad.JOpt=cost_q(zOq);
fprintf('  Gamma: %.2f->%.2f | J: %.3f->%.3f\n',z0q(1),zOq(1),results.quad.J0,results.quad.JOpt);

%% --- Plot ---
f = figure('Visible','off','Position',[1 1 1500 950]);
subplot(3,3,1); plot(t1,Q1(:,1),'r--','LineWidth',1.4,'DisplayName',sprintf('baseline (\\Gamma=%.2f,\\sigma=%.3f)',z0(1),z0(2))); hold on;
plot(t2,Q2(:,1),'b-','LineWidth',1.4,'DisplayName',sprintf('optimized (\\Gamma=%.2f,\\sigma=%.3f)',zOpt(1),zOpt(2)));
yline(q_des(1),'k:','LineWidth',1.2,'DisplayName','q_{1,des}');
grid on; title('Manipulator q_1 (payload unknown)'); ylabel('q_1 [rad]'); xlabel('t [s]');
legend('Location','southeast','FontSize',7);

subplot(3,3,2); plot(t1,Th1,'r--','LineWidth',1.2,'DisplayName','baseline \hat{m}_2'); hold on;
plot(t2,Th2,'b-','LineWidth',1.2,'DisplayName','optimized \hat{m}_2');
yline(p_true.m2,'k:','LineWidth',1.2,'DisplayName','true m_2');
grid on; title('Payload estimate \hat{m}_2 (converges toward true mass)');
ylabel('\hat{m}_2 [kg]'); xlabel('t [s]'); legend('Location','best','FontSize',7);

subplot(3,3,3); plot(t1,U1(:,1),'r--','LineWidth',1.2,'DisplayName','baseline \tau_1'); hold on;
plot(t2,U2(:,1),'b-','LineWidth',1.2,'DisplayName','optimized \tau_1');
grid on; title('Manipulator torque \tau_1'); ylabel('\tau_1 [N\cdotm]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,4); tref=linspace(0,2*pi,200);
plot(cos(tref),sin(tref),'k:','LineWidth',1.3,'DisplayName','reference circle'); hold on;
plot(Xm1(:,1),Xm1(:,2),'r--','LineWidth',1.4,'DisplayName','baseline adaptive');
plot(Xm2(:,1),Xm2(:,2),'b-','LineWidth',1.4,'DisplayName','optimized adaptive');
axis equal; grid on; title('Mobile robot XY (adaptive to wheel scaling)');
xlabel('x [m]'); ylabel('y [m]'); legend('Location','best','FontSize',7);

subplot(3,3,5); plot(tm1,Um1(:,1),'r--','LineWidth',1.2,'DisplayName','baseline v'); hold on;
plot(tm2,Um2(:,1),'b-','LineWidth',1.2,'DisplayName','optimized v');
grid on; title('Mobile v command'); ylabel('v [m/s]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,6); plot(tm1,Um1(:,2),'r--','LineWidth',1.2,'DisplayName','baseline \omega'); hold on;
plot(tm2,Um2(:,2),'b-','LineWidth',1.2,'DisplayName','optimized \omega');
grid on; title('Mobile \omega command'); ylabel('\omega [rad/s]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,7); plot(tq1,Xq1(:,2),'r--','LineWidth',1.4,'DisplayName','baseline adaptive'); hold on;
plot(tq2,Xq2(:,2),'b-','LineWidth',1.4,'DisplayName','optimized adaptive');
yline(1.0,'k:','LineWidth',1.2,'DisplayName','z_{des}=1m');
grid on; title('Quadrotor altitude (adaptive mass)'); ylabel('z [m]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,8); plot(tq1,Mh1,'r--','LineWidth',1.2,'DisplayName','baseline \hat{m}'); hold on;
plot(tq2,Mh2,'b-','LineWidth',1.2,'DisplayName','optimized \hat{m}');
yline(pq_true.m,'k:','LineWidth',1.2,'DisplayName','true m');
yline(pq.m,'Color',[.4 .4 .4],'LineStyle',':','DisplayName','nominal m');
grid on; title('Drone mass estimate \hat{m}'); ylabel('\hat{m} [kg]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,9); plot(tq1,Uq1(:,1),'r--','LineWidth',1.2,'DisplayName','baseline thrust'); hold on;
plot(tq2,Uq2(:,1),'b-','LineWidth',1.2,'DisplayName','optimized thrust');
yline(pq_true.m*pq.g,'k:','LineWidth',1.0,'DisplayName','true mg');
grid on; title('Quadrotor thrust T'); ylabel('T [N]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

sgtitle({'ADAPTIVE CONTROL  —  red dashed = baseline (\Gamma,\sigma),  blue solid = optimized (\Gamma,\sigma),  black dotted = true value / setpoint',...
    sprintf('Cost reductions: manipulator %.1f%%, mobile %.1f%%, quadrotor %.1f%%',...
    100*(results.manipulator.J0-results.manipulator.JOpt)/results.manipulator.J0,...
    100*(results.mobile.J0-results.mobile.JOpt)/results.mobile.J0,...
    100*(results.quad.J0-results.quad.JOpt)/results.quad.J0)},...
    'FontSize',10,'FontWeight','bold');
if saveFig
    saveas(f, fullfile(fileparts(mfilename('fullpath')),'figures','adaptive_results.png'));
end
close(f);
end

% ---------- Manipulator adaptive (regressor on payload mass m2) ----------
function [t,Q,U,Th] = run_ada_manip(z, p_nom, p_true, q_des, T, dt)
Gamma=z(1); sigma=z(2);
N=round(T/dt); t=(0:N-1)'*dt;
q=[0;0]; dq=[0;0];
Kp=diag([80 80]); Kd=diag([20 20]);
theta_hat = p_nom.m2;  % estimate of m2
Q=zeros(N,2); U=zeros(N,2); Th=zeros(N,1);
for k=1:N
    e=q-q_des; de=dq;
    % model with estimated mass
    p_est = p_nom; p_est.m2 = theta_hat;
    [Mh,Ch,Gh] = manipulator_dyn(q,dq,p_est);
    u = Mh*(-Kp*e - Kd*de) + Ch*dq + Gh;
    % true plant
    [Mt,Ct,Gt] = manipulator_dyn(q,dq,p_true);
    ddq = Mt \ (u - Ct*dq - Gt);
    % adaptation law: sign-based simple update with sigma-mod
    phi = (e(1)+e(2));     % scalar surrogate regressor
    theta_hat = theta_hat + dt*(Gamma*phi - sigma*theta_hat);
    theta_hat = max(0.1, min(3.0, theta_hat));
    dq = dq + dt*ddq; q = q + dt*dq;
    Q(k,:)=q'; U(k,:)=u'; Th(k)=theta_hat;
end
end
function J = ada_manip_cost(z,p,pt,qd,T,dt)
if z(1)<=0 || z(1)>30 || z(2)<0 || z(2)>0.2, J=1e6; return; end
[~,Q,U,~] = run_ada_manip(z,p,pt,qd,T,dt);
e=Q-qd';
J = sum(sum(e.^2))*dt + 0.001*sum(sum(U.^2))*dt;
end

% ---------- Mobile adaptive (scale factor on v command) ----------
function [t,X,U,Ah] = run_ada_mobile(z,T,dt)
Gamma=z(1); sigma=z(2);
N=round(T/dt); t=(0:N-1)'*dt;
x=[0;0;0]; a_true=1.25; a_hat=1.0;  % unknown velocity scaling
X=zeros(N,3); U=zeros(N,2); Ah=zeros(N,1);
for k=1:N
    tk=t(k);
    xr=[cos(0.3*tk); sin(0.3*tk); 0.3*tk+pi/2];
    dxr=[-0.3*sin(0.3*tk); 0.3*cos(0.3*tk); 0.3];
    R=[cos(x(3)) sin(x(3)); -sin(x(3)) cos(x(3))];
    eb=R*(xr(1:2)-x(1:2)); etheta=wrap(xr(3)-x(3));
    vref=norm(dxr(1:2));
    kx=1; ky=2; kth=2;
    v_cmd = (vref*cos(etheta) + kx*eb(1)) / a_hat;
    omega = dxr(3) + vref*(ky*eb(2) + kth*sin(etheta));
    u=[v_cmd;omega];
    % true plant has scale a_true on v
    u_true = [a_true*u(1); u(2)];
    dx = unicycle_dyn(x,u_true);
    % adaptation: drive eb(1)->0 tunes a_hat
    a_hat = a_hat + dt*(Gamma*eb(1)*v_cmd - sigma*(a_hat-1.0));
    a_hat = max(0.2, min(3.0, a_hat));
    x = x + dt*dx; x(3)=wrap(x(3));
    X(k,:)=x'; U(k,:)=u'; Ah(k)=a_hat;
end
end
function J = ada_mobile_cost(z,T,dt)
if z(1)<=0 || z(1)>20 || z(2)<0 || z(2)>0.3, J=1e6; return; end
[t,X,U,~]=run_ada_mobile(z,T,dt);
xr=[cos(0.3*t) sin(0.3*t)];
e=X(:,1:2)-xr;
J = sum(sum(e.^2))*dt + 0.005*sum(sum(U.^2))*dt;
end

% ---------- Quadrotor adaptive mass ----------
function [t,X,U,Mh] = run_ada_quad(z,p_nom,p_true,T,dt)
Gamma=z(1); sigma=z(2);
N=round(T/dt); t=(0:N-1)'*dt;
x=[0;0;0;0;0;0]; z_des=1.0;
m_hat=p_nom.m;
X=zeros(N,6); U=zeros(N,2); Mh=zeros(N,1);
Kp=6; Kd=4; Kpp=8; Kdp=3;
for k=1:N
    ez=x(2)-z_des; dez=x(5);
    T_cmd = m_hat*(p_nom.g - Kp*ez - Kd*dez);
    T_cmd = max(0.05, T_cmd);
    tau = p_nom.J*(-Kpp*x(3) - Kdp*x(6));
    u=[T_cmd; tau];
    dx = quadrotor_dyn(x,u,p_true);
    % adaptation: mass error proxy = dez
    m_hat = m_hat + dt*(Gamma*dez*(p_nom.g - Kp*ez - Kd*dez) - sigma*(m_hat-p_nom.m));
    m_hat = max(0.1, min(2.0, m_hat));
    x = x + dt*dx;
    X(k,:)=x'; U(k,:)=u'; Mh(k)=m_hat;
end
end
function J = ada_quad_cost(z,p,pt,T,dt)
if z(1)<=0 || z(1)>20 || z(2)<0 || z(2)>0.3, J=1e6; return; end
[~,X,U,~]=run_ada_quad(z,p,pt,T,dt);
ez = X(:,2)-1.0;
J = sum(ez.^2)*dt + 0.001*sum(sum(U.^2))*dt;
end

function y = wrap(a), y = mod(a+pi,2*pi)-pi; end
