function results = sim_backstepping(saveFig)
% BACKSTEPPING optimization: optimize virtual-control gains k1..kn

if nargin<1, saveFig=true; end
results=struct();
fprintf('\n=== BACKSTEPPING CONTROL OPTIMIZATION ===\n');

%% ------ MANIPULATOR (2-step backstepping on each joint) ------
fprintf('[BST] Manipulator...\n');
p=manipulator_params(); q_des=[pi/3;-pi/4]; T=3; dt=0.002;
cost_m = @(z) bst_manip_cost(z,p,q_des,T,dt);
z0=[5 10];                  % [k1, k2]
zOpt=fminsearch(cost_m,z0,optimset('Display','off','MaxIter',80));
[t1,Q1,U1]=run_bst_manip(z0,p,q_des,T,dt);
[t2,Q2,U2]=run_bst_manip(zOpt,p,q_des,T,dt);
results.manipulator.z0=z0; results.manipulator.zOpt=zOpt;
results.manipulator.J0=cost_m(z0); results.manipulator.JOpt=cost_m(zOpt);
fprintf('  k1,k2: [%.1f %.1f]->[%.1f %.1f] | J: %.3f->%.3f\n',...
    z0,zOpt,results.manipulator.J0,results.manipulator.JOpt);

%% ------ MOBILE (Kanayama-style with backstepping) ------
fprintf('[BST] Mobile...\n');
T=8; dt=0.02;
cost_mob=@(z) bst_mobile_cost(z,T,dt);
z0m=[1 1 1];                % [kx, ky, ktheta]
zOm = fminsearch(cost_mob,z0m,optimset('Display','off','MaxIter',80));
[tm1,Xm1,Um1]=run_bst_mobile(z0m,T,dt);
[tm2,Xm2,Um2]=run_bst_mobile(zOm,T,dt);
results.mobile.z0=z0m; results.mobile.zOpt=zOm;
results.mobile.J0=cost_mob(z0m); results.mobile.JOpt=cost_mob(zOm);
fprintf('  gains: [%.2f %.2f %.2f]->[%.2f %.2f %.2f] | J: %.3f->%.3f\n',...
    z0m,zOm,results.mobile.J0,results.mobile.JOpt);

%% ------ QUADROTOR (position->attitude cascaded backstepping) ------
fprintf('[BST] Quadrotor...\n');
pq=quadrotor_params(); T=4; dt=0.005;
cost_q=@(z) bst_quad_cost(z,pq,T,dt);
z0q=[4 3 8 4];            % [kz1,kz2,kphi1,kphi2]
zOq = fminsearch(cost_q,z0q,optimset('Display','off','MaxIter',80));
[tq1,Xq1,Uq1]=run_bst_quad(z0q,pq,T,dt);
[tq2,Xq2,Uq2]=run_bst_quad(zOq,pq,T,dt);
results.quad.z0=z0q; results.quad.zOpt=zOq;
results.quad.J0=cost_q(z0q); results.quad.JOpt=cost_q(zOq);
fprintf('  J: %.3f->%.3f\n',results.quad.J0,results.quad.JOpt);

%% Plot
f=figure('Visible','off','Position',[1 1 1500 950]);
subplot(3,3,1); plot(t1,Q1(:,1),'r--','LineWidth',1.4,'DisplayName',sprintf('baseline (k_1=%.1f,k_2=%.1f)',z0(1),z0(2))); hold on;
plot(t2,Q2(:,1),'b-','LineWidth',1.4,'DisplayName',sprintf('optimized (k_1=%.1f,k_2=%.1f)',zOpt(1),zOpt(2)));
yline(q_des(1),'k:','LineWidth',1.2,'DisplayName','q_{1,des}');
grid on; title('Manipulator q_1'); ylabel('q_1 [rad]'); xlabel('t [s]');
legend('Location','southeast','FontSize',7);

subplot(3,3,2); plot(t1,Q1(:,2),'r--','LineWidth',1.4,'DisplayName','baseline'); hold on;
plot(t2,Q2(:,2),'b-','LineWidth',1.4,'DisplayName','optimized');
yline(q_des(2),'k:','LineWidth',1.2,'DisplayName','q_{2,des}');
grid on; title('Manipulator q_2'); ylabel('q_2 [rad]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,3); plot(t1,U1(:,1),'r--','LineWidth',1.2,'DisplayName','baseline \tau_1'); hold on;
plot(t2,U2(:,1),'b-','LineWidth',1.2,'DisplayName','optimized \tau_1');
grid on; title('Manipulator torque \tau_1'); ylabel('\tau_1 [N\cdotm]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,4); tref=linspace(0,2*pi,200);
plot(cos(tref),sin(tref),'k:','LineWidth',1.3,'DisplayName','reference circle'); hold on;
plot(Xm1(:,1),Xm1(:,2),'r--','LineWidth',1.4,'DisplayName',sprintf('baseline (k_x=%.1f,k_y=%.1f,k_\\theta=%.1f)',z0m(1),z0m(2),z0m(3)));
plot(Xm2(:,1),Xm2(:,2),'b-','LineWidth',1.4,'DisplayName',sprintf('optimized (k_x=%.1f,k_y=%.1f,k_\\theta=%.1f)',zOm(1),zOm(2),zOm(3)));
axis equal; grid on; title('Mobile XY (Kanayama tracker)');
xlabel('x [m]'); ylabel('y [m]'); legend('Location','best','FontSize',7);

subplot(3,3,5); plot(tm1,Um1(:,1),'r--','LineWidth',1.2,'DisplayName','baseline v'); hold on;
plot(tm2,Um2(:,1),'b-','LineWidth',1.2,'DisplayName','optimized v');
grid on; title('Mobile v command'); ylabel('v [m/s]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,6); plot(tm1,Um1(:,2),'r--','LineWidth',1.2,'DisplayName','baseline \omega'); hold on;
plot(tm2,Um2(:,2),'b-','LineWidth',1.2,'DisplayName','optimized \omega');
grid on; title('Mobile \omega command'); ylabel('\omega [rad/s]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,7); plot(tq1,Xq1(:,2),'r--','LineWidth',1.4,'DisplayName','baseline'); hold on;
plot(tq2,Xq2(:,2),'b-','LineWidth',1.4,'DisplayName','optimized');
yline(1.0,'k:','LineWidth',1.2,'DisplayName','z_{des}=1m');
grid on; title('Quadrotor altitude'); ylabel('z [m]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,8); plot(tq1,Xq1(:,3),'r--','LineWidth',1.4,'DisplayName','baseline'); hold on;
plot(tq2,Xq2(:,3),'b-','LineWidth',1.4,'DisplayName','optimized');
yline(0,'k:','LineWidth',1.0,'DisplayName','\phi_{des}=0');
grid on; title('Quadrotor roll \phi'); ylabel('\phi [rad]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,9); plot(tq1,Uq1(:,1),'r--','LineWidth',1.2,'DisplayName','baseline thrust'); hold on;
plot(tq2,Uq2(:,1),'b-','LineWidth',1.2,'DisplayName','optimized thrust');
yline(pq.m*pq.g,'k:','LineWidth',1.0,'DisplayName','hover thrust mg');
grid on; title('Quadrotor thrust T'); ylabel('T [N]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

sgtitle({'BACKSTEPPING  —  red dashed = baseline virtual-control gains,  blue solid = optimized gains,  black dotted = reference',...
    sprintf('Cost reductions: manipulator %.1f%%, mobile %.1f%%, quadrotor %.1f%%',...
    100*(results.manipulator.J0-results.manipulator.JOpt)/results.manipulator.J0,...
    100*(results.mobile.J0-results.mobile.JOpt)/results.mobile.J0,...
    100*(results.quad.J0-results.quad.JOpt)/results.quad.J0)},...
    'FontSize',10,'FontWeight','bold');
if saveFig
    saveas(f, fullfile(fileparts(mfilename('fullpath')),'figures','backstepping_results.png'));
end
close(f);
end

% ---- Manipulator backstepping ----
function [t,Q,U]=run_bst_manip(z,p,q_des,T,dt)
k1=z(1); k2=z(2);
N=round(T/dt); t=(0:N-1)'*dt;
q=[0;0]; dq=[0;0];
Q=zeros(N,2); U=zeros(N,2);
for k=1:N
    e1 = q - q_des;
    alpha = -k1*e1;          % virtual control for dq
    e2 = dq - alpha;
    dalpha = -k1*dq;
    [M,C,G]=manipulator_dyn(q,dq,p);
    % Lyapunov step: u = M*(dalpha - k2*e2 - e1) + C*dq + G
    u = M*(dalpha - k2*e2 - e1) + C*dq + G;
    ddq = M\(u - C*dq - G);
    dq = dq+dt*ddq; q=q+dt*dq;
    Q(k,:)=q'; U(k,:)=u';
end
end
function J = bst_manip_cost(z,p,qd,T,dt)
if any(z<=0) || any(z>60), J=1e6; return; end
[~,Q,U]=run_bst_manip(z,p,qd,T,dt);
e=Q-qd';
J = sum(sum(e.^2))*dt + 0.001*sum(sum(U.^2))*dt;
end

% ---- Mobile Kanayama ----
function [t,X,U]=run_bst_mobile(z,T,dt)
kx=z(1); ky=z(2); kth=z(3);
N=round(T/dt); t=(0:N-1)'*dt;
x=[0;0;0]; X=zeros(N,3); U=zeros(N,2);
for k=1:N
    tk=t(k);
    xr=[cos(0.3*tk); sin(0.3*tk); 0.3*tk+pi/2];
    dxr=[-0.3*sin(0.3*tk); 0.3*cos(0.3*tk); 0.3];
    R=[cos(x(3)) sin(x(3)); -sin(x(3)) cos(x(3))];
    eb=R*(xr(1:2)-x(1:2)); etheta=wrap(xr(3)-x(3));
    vref=norm(dxr(1:2));
    v = vref*cos(etheta)+kx*eb(1);
    omega = dxr(3)+vref*(ky*eb(2)+kth*sin(etheta));
    u=[v;omega];
    dx = unicycle_dyn(x,u);
    x=x+dt*dx; x(3)=wrap(x(3));
    X(k,:)=x'; U(k,:)=u';
end
end
function J = bst_mobile_cost(z,T,dt)
if any(z<=0) || any(z>20), J=1e6; return; end
[t,X,U]=run_bst_mobile(z,T,dt);
xr=[cos(0.3*t) sin(0.3*t)];
e=X(:,1:2)-xr;
J = sum(sum(e.^2))*dt + 0.01*sum(sum(U.^2))*dt;
end

% ---- Quadrotor cascaded backstepping ----
function [t,X,U]=run_bst_quad(z,p,T,dt)
kz1=z(1); kz2=z(2); kphi1=z(3); kphi2=z(4);
N=round(T/dt); t=(0:N-1)'*dt;
x=[0;0;0;0;0;0]; X=zeros(N,6); U=zeros(N,2);
z_des=1.0;
for k=1:N
    % altitude backstepping: e1=z-zdes, alpha=-kz1*e1, e2=vz-alpha
    ez1 = x(2)-z_des;
    alpha = -kz1*ez1;
    ez2 = x(5)-alpha;
    dalpha = -kz1*x(5);
    % dynamics: dvz = T*cos(phi)/m - g => T = m*(g + dalpha - kz2*ez2 - ez1)/cos(phi)
    T_cmd = p.m*(p.g + dalpha - kz2*ez2 - ez1)/max(cos(x(3)),0.3);
    T_cmd = max(0.05, T_cmd);
    % attitude backstepping: e1=phi, alpha=-kphi1*phi, e2=dphi-alpha
    ep1 = x(3);
    alphaP = -kphi1*ep1;
    ep2 = x(6) - alphaP;
    dalphaP = -kphi1*x(6);
    tau = p.J*(dalphaP - kphi2*ep2 - ep1);
    u=[T_cmd; tau];
    dx = quadrotor_dyn(x,u,p);
    x=x+dt*dx;
    X(k,:)=x'; U(k,:)=u';
end
end
function J = bst_quad_cost(z,p,T,dt)
if any(z<=0) || any(z>30), J=1e6; return; end
[~,X,U]=run_bst_quad(z,p,T,dt);
ez=X(:,2)-1.0; ephi=X(:,3);
J = sum(ez.^2)*dt + 0.5*sum(ephi.^2)*dt + 0.001*sum(sum(U.^2))*dt;
end

function y=wrap(a), y=mod(a+pi,2*pi)-pi; end
