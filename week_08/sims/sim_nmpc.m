function results = sim_nmpc(saveFig)
% NONLINEAR MPC via iLQR solver (proper Gauss-Newton Riccati pass, not fminsearch).
% Compares short vs. long prediction horizon on manipulator, mobile robot, quadrotor.
% Each sample: run iLQR with warm-start; apply first control; shift; repeat.

if nargin<1, saveFig=true; end
results = struct();
fprintf('\n=== NONLINEAR MPC OPTIMIZATION (iLQR-based) ===\n');

%% --- Manipulator NMPC ---
fprintf('[NMPC] Manipulator...\n');
p = manipulator_params(); q_des=[pi/3;-pi/4];
dt = 0.05; Tsim = 2.5; Nsim = round(Tsim/dt);
Q_run  = diag([80 80 1 1]);
R_run  = diag([0.01 0.01]);
Q_term = 40*diag([80 80 1 1]);
[tm1,Qm1,dQm1,Um1,Jm1] = run_nmpc_manip(5,  Q_run, R_run, Q_term, p, q_des, dt, Nsim);
[tm2,Qm2,dQm2,Um2,Jm2] = run_nmpc_manip(15, Q_run, R_run, Q_term, p, q_des, dt, Nsim);
results.manipulator.N_short=5;  results.manipulator.J_short=Jm1;
results.manipulator.N_long=15;  results.manipulator.J_long =Jm2;
fprintf('  N=5: J=%.4f  |  N=15: J=%.4f\n',Jm1,Jm2);

%% --- Mobile NMPC ---
fprintf('[NMPC] Mobile robot (circular trajectory)...\n');
dt = 0.1; Tsim = 12; Nsim = round(Tsim/dt);
Q_run_mob  = diag([10 10 1]);
R_run_mob  = diag([0.05 0.05]);
Q_term_mob = 20*Q_run_mob;
[tmob1,Xmob1,Umob1,Jmob1] = run_nmpc_mobile(5,  Q_run_mob,R_run_mob,Q_term_mob, dt,Nsim);
[tmob2,Xmob2,Umob2,Jmob2] = run_nmpc_mobile(15, Q_run_mob,R_run_mob,Q_term_mob, dt,Nsim);
results.mobile.N_short=5;  results.mobile.J_short=Jmob1;
results.mobile.N_long =15; results.mobile.J_long =Jmob2;
fprintf('  N=5: J=%.4f  |  N=15: J=%.4f\n',Jmob1,Jmob2);

%% --- Quadrotor NMPC ---
fprintf('[NMPC] Quadrotor...\n');
pq = quadrotor_params();
dt = 0.05; Tsim = 3; Nsim = round(Tsim/dt);
Q_run_q  = diag([1 50 20 0.1 1 0.1]);
R_run_q  = diag([0.001 0.01]);
Q_term_q = 30*Q_run_q;
[tq1,Xq1,Uq1,Jq1] = run_nmpc_quad(5,  Q_run_q,R_run_q,Q_term_q, pq, dt,Nsim);
[tq2,Xq2,Uq2,Jq2] = run_nmpc_quad(15, Q_run_q,R_run_q,Q_term_q, pq, dt,Nsim);
results.quad.N_short=5;  results.quad.J_short=Jq1;
results.quad.N_long =15; results.quad.J_long =Jq2;
fprintf('  N=5: J=%.4f  |  N=15: J=%.4f\n',Jq1,Jq2);

%% Plot
f=figure('Visible','off','Position',[1 1 1500 950]);
subplot(3,3,1); plot(tm1,Qm1(:,1),'r--','LineWidth',1.4,'DisplayName','NMPC short horizon N=5'); hold on;
plot(tm2,Qm2(:,1),'b-','LineWidth',1.4,'DisplayName','NMPC long horizon N=15');
yline(q_des(1),'k:','LineWidth',1.2,'DisplayName','q_{1,des}');
grid on; title('Manipulator q_1'); ylabel('q_1 [rad]'); xlabel('t [s]');
legend('Location','southeast','FontSize',7);

subplot(3,3,2); plot(tm1,Qm1(:,2),'r--','LineWidth',1.4,'DisplayName','N=5'); hold on;
plot(tm2,Qm2(:,2),'b-','LineWidth',1.4,'DisplayName','N=15');
yline(q_des(2),'k:','LineWidth',1.2,'DisplayName','q_{2,des}');
grid on; title('Manipulator q_2'); ylabel('q_2 [rad]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,3); plot(tm1,Um1(:,1),'r--','LineWidth',1.2,'DisplayName','N=5 \tau_1'); hold on;
plot(tm2,Um2(:,1),'b-','LineWidth',1.2,'DisplayName','N=15 \tau_1');
grid on; title('Manipulator torque \tau_1'); ylabel('\tau_1 [N\cdotm]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,4); tref=linspace(0,2*pi,200);
plot(cos(tref),sin(tref),'k:','LineWidth',1.3,'DisplayName','reference circle'); hold on;
plot(Xmob1(:,1),Xmob1(:,2),'r--','LineWidth',1.4,'DisplayName','NMPC N=5');
plot(Xmob2(:,1),Xmob2(:,2),'b-','LineWidth',1.4,'DisplayName','NMPC N=15');
axis equal; grid on; title('Mobile XY tracking');
xlabel('x [m]'); ylabel('y [m]'); legend('Location','best','FontSize',7);

subplot(3,3,5); plot(tmob1,Umob1(:,1),'r--','LineWidth',1.2,'DisplayName','N=5 v'); hold on;
plot(tmob2,Umob2(:,1),'b-','LineWidth',1.2,'DisplayName','N=15 v');
grid on; title('Mobile v command'); ylabel('v [m/s]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,6); plot(tmob1,Umob1(:,2),'r--','LineWidth',1.2,'DisplayName','N=5 \omega'); hold on;
plot(tmob2,Umob2(:,2),'b-','LineWidth',1.2,'DisplayName','N=15 \omega');
grid on; title('Mobile \omega command'); ylabel('\omega [rad/s]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,7); plot(tq1,Xq1(:,2),'r--','LineWidth',1.4,'DisplayName','NMPC N=5'); hold on;
plot(tq2,Xq2(:,2),'b-','LineWidth',1.4,'DisplayName','NMPC N=15');
yline(1.0,'k:','LineWidth',1.2,'DisplayName','z_{des}=1m');
grid on; title('Quadrotor altitude'); ylabel('z [m]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,8); plot(tq1,Xq1(:,3),'r--','LineWidth',1.4,'DisplayName','N=5'); hold on;
plot(tq2,Xq2(:,3),'b-','LineWidth',1.4,'DisplayName','N=15');
yline(0,'k:','LineWidth',1.0,'DisplayName','\phi_{des}=0');
grid on; title('Quadrotor roll \phi'); ylabel('\phi [rad]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

subplot(3,3,9); plot(tq1,Uq1(:,1),'r--','LineWidth',1.2,'DisplayName','N=5 thrust'); hold on;
plot(tq2,Uq2(:,1),'b-','LineWidth',1.2,'DisplayName','N=15 thrust');
yline(pq.T_hover,'k:','LineWidth',1.0,'DisplayName','hover thrust mg');
grid on; title('Quadrotor thrust T'); ylabel('T [N]'); xlabel('t [s]');
legend('Location','best','FontSize',7);

sgtitle({'NONLINEAR MPC (iLQR solver)  —  red dashed = short prediction horizon,  blue solid = long prediction horizon,  black dotted = reference',...
    sprintf('Closed-loop cost: manipulator %.2f vs %.2f  |  mobile %.2f vs %.2f  |  quadrotor %.2f vs %.2f',...
    Jm1,Jm2,Jmob1,Jmob2,Jq1,Jq2)},...
    'FontSize',10,'FontWeight','bold');
if saveFig
    saveas(f, fullfile(fileparts(mfilename('fullpath')),'figures','nmpc_results.png'));
end
close(f);
end

% ======================================================================
% Manipulator NMPC closed loop
% State x = [q1; q2; dq1; dq2], u = [tau1; tau2]
% ======================================================================
function [t,Q,dQ,U,J_cl] = run_nmpc_manip(N, Qr, Rr, Qt, p, q_des, dt, Nsim)
x = zeros(4,1);
U_seq = zeros(N,2);
T_hist = zeros(Nsim,2);
Q_hist = zeros(Nsim,2);
dQ_hist = zeros(Nsim,2);
U_hist = zeros(Nsim,2);
t = (0:Nsim-1)'*dt;

fdyn = @(xx,uu) manip_f(xx,uu,p);
runCost  = @(xx,uu) manip_run_cost(xx,uu,q_des,Qr,Rr);
termCost = @(xx)    manip_term_cost(xx,q_des,Qt);

opts = struct('max_iter',30,'tol',1e-4,'mu0',1e-3,'mu_max',1e6,'verbose',false);

for k=1:Nsim
    [~, U_opt, ~, ~] = ilqr_solve(fdyn, runCost, termCost, x, U_seq, dt, opts);
    u = U_opt(1,:)';
    % apply, step plant
    x = x + dt*fdyn(x,u);
    % record
    Q_hist(k,:)  = x(1:2)';
    dQ_hist(k,:) = x(3:4)';
    U_hist(k,:)  = u';
    % warm start: shift and replicate last
    U_seq = [U_opt(2:end,:); U_opt(end,:)];
end
Q = Q_hist; dQ = dQ_hist; U = U_hist;
% closed-loop cost (integrated)
e  = Q - q_des';
J_cl = sum(sum(e.^2))*dt + 0.001*sum(sum(U.^2))*dt;
end
function dx = manip_f(x,u,p)
q  = x(1:2); dq = x(3:4);
[M,C,G] = manipulator_dyn(q,dq,p);
ddq = M \ (u - C*dq - G);
dx  = [dq; ddq];
end
function L = manip_run_cost(x,u,q_des,Qr,Rr)
e = [x(1)-q_des(1); x(2)-q_des(2); x(3); x(4)];
L = e'*Qr*e + u'*Rr*u;
end
function Phi = manip_term_cost(x,q_des,Qt)
e = [x(1)-q_des(1); x(2)-q_des(2); x(3); x(4)];
Phi = e'*Qt*e;
end

% ======================================================================
% Mobile NMPC closed loop (circular reference)
% x = [px; py; theta], u = [v; omega]
% ======================================================================
function [t,X,U,J_cl] = run_nmpc_mobile(N, Qr, Rr, Qt, dt, Nsim)
x = [1;0;pi/2];               % start on reference
U_seq = zeros(N,2);
X_hist = zeros(Nsim,3); U_hist = zeros(Nsim,2);
t = (0:Nsim-1)'*dt;

fdyn = @(xx,uu) unicycle_dyn(xx,uu);
opts = struct('max_iter',30,'tol',1e-4,'mu0',1e-3,'mu_max',1e6,'verbose',false);

for k=1:Nsim
    tk = (k-1)*dt;
    refFun = @(ti) [cos(0.3*ti); sin(0.3*ti); 0.3*ti+pi/2];
    runCost  = @(xx,uu) mobile_run_cost_t(xx,uu,0,Qr,Rr);    % only placeholder
    % Build time-varying cost closure per-step: pass via anonymous per t_k+i*dt
    % We'll embed this in a wrapper:
    ilqrOpts = opts;
    % Build custom iLQR call with time-varying cost (simpler: regenerate cost references per step)
    [X_pred, U_opt, ~, ~] = ilqr_time_varying(fdyn, @(xx,uu,ti) mobile_rc(xx,uu,refFun(ti),Qr,Rr),...
                                              @(xx,ti) mobile_tc(xx,refFun(ti),Qt),...
                                              x, U_seq, dt, tk, ilqrOpts);
    u = U_opt(1,:)';
    x = x + dt*fdyn(x,u); x(3)=wrap(x(3));
    X_hist(k,:) = x'; U_hist(k,:) = u';
    U_seq = [U_opt(2:end,:); U_opt(end,:)];
end
X = X_hist; U = U_hist;
xr = [cos(0.3*t) sin(0.3*t)];
e  = X(:,1:2)-xr;
J_cl = sum(sum(e.^2))*dt + 0.01*sum(sum(U.^2))*dt;
end
function L = mobile_rc(x,u,xr,Qr,Rr)
e = [x(1)-xr(1); x(2)-xr(2); wrap(x(3)-xr(3))];
L = e'*Qr*e + u'*Rr*u;
end
function Phi = mobile_tc(x,xr,Qt)
e = [x(1)-xr(1); x(2)-xr(2); wrap(x(3)-xr(3))];
Phi = e'*Qt*e;
end
function L = mobile_run_cost_t(x,u,xr,Qr,Rr)  % unused placeholder
L = 0;
end

% ======================================================================
% Quadrotor NMPC
% ======================================================================
function [t,X,U,J_cl] = run_nmpc_quad(N, Qr, Rr, Qt, p, dt, Nsim)
x = zeros(6,1);
U_seq = repmat([p.T_hover, 0], N, 1);
X_hist = zeros(Nsim,6); U_hist = zeros(Nsim,2);
t = (0:Nsim-1)'*dt;

fdyn = @(xx,uu) quadrotor_dyn(xx,uu,p);
runCost  = @(xx,uu) quad_rc(xx,uu,p,Qr,Rr);
termCost = @(xx)    quad_tc(xx,p,Qt);

opts = struct('max_iter',30,'tol',1e-4,'mu0',1e-3,'mu_max',1e6,'verbose',false);

for k=1:Nsim
    [~, U_opt, ~, ~] = ilqr_solve(fdyn, runCost, termCost, x, U_seq, dt, opts);
    u = U_opt(1,:)';
    x = x + dt*fdyn(x,u);
    X_hist(k,:) = x'; U_hist(k,:) = u';
    U_seq = [U_opt(2:end,:); U_opt(end,:)];
end
X = X_hist; U = U_hist;
ez = X(:,2)-1.0;
J_cl = sum(ez.^2)*dt + 0.5*sum(X(:,3).^2)*dt + 0.001*sum(sum(U.^2))*dt;
end
function L = quad_rc(x,u,p,Qr,Rr)
e = [x(1); x(2)-1.0; x(3); x(4); x(5); x(6)];
du = u - [p.T_hover;0];
L = e'*Qr*e + du'*Rr*du;
end
function Phi = quad_tc(x,p,Qt)
e = [x(1); x(2)-1.0; x(3); x(4); x(5); x(6)];
Phi = e'*Qt*e;
end

% ======================================================================
% iLQR wrapper supporting time-varying cost (for trajectory tracking)
% ======================================================================
function [X, U, J, iters] = ilqr_time_varying(fdyn, runCost_t, termCost_t, x0, U_init, dt, t0, opts)
N  = size(U_init,1);
nu = size(U_init,2);
nx = numel(x0);
U  = U_init;
mu = opts.mu0;

X = zeros(N+1,nx); X(1,:) = x0';
for t=1:N, X(t+1,:) = X(t,:) + dt*fdyn(X(t,:)',U(t,:)')'; end
J = total_cost_tv(runCost_t, termCost_t, X, U, t0, dt);

for iters=1:opts.max_iter
    [A,B] = lin_dyn(fdyn,X,U,dt);
    [Lx,Lu,Lxx,Luu,Lux,Phix,Phixx] = cost_derivs_tv(runCost_t,termCost_t,X,U,t0,dt);

    Vx = Phix; Vxx = Phixx;
    k_ff = zeros(nu,N); K_fb = zeros(nu,nx,N);
    fail = false;
    for t = N:-1:1
        Qx  = Lx(:,t)     + A(:,:,t)'*Vx;
        Qu  = Lu(:,t)     + B(:,:,t)'*Vx;
        Qxx = Lxx(:,:,t)  + A(:,:,t)'*Vxx*A(:,:,t);
        Quu = Luu(:,:,t)  + B(:,:,t)'*Vxx*B(:,:,t) + mu*eye(nu);
        Qux = Lux(:,:,t)  + B(:,:,t)'*Vxx*A(:,:,t);
        Quu = 0.5*(Quu+Quu');
        [R,p] = chol(Quu);
        if p>0, fail = true; break; end
        kt = -R\(R'\Qu);  Kt = -R\(R'\Qux);
        k_ff(:,t) = kt; K_fb(:,:,t) = Kt;
        Vx  = Qx  + Kt'*Quu*kt + Kt'*Qu + Qux'*kt;
        Vxx = Qxx + Kt'*Quu*Kt + Kt'*Qux + Qux'*Kt;
        Vxx = 0.5*(Vxx+Vxx');
    end
    if fail
        mu = min(mu*4,opts.mu_max);
        if mu>=opts.mu_max, break; end
        continue;
    end

    alpha=1.0; accepted=false;
    for ls=1:12
        Xnew = zeros(size(X)); Unew = zeros(size(U));
        Xnew(1,:) = X(1,:);
        for t=1:N
            dx = (Xnew(t,:) - X(t,:))';
            du = alpha*k_ff(:,t) + K_fb(:,:,t)*dx;
            Unew(t,:) = U(t,:) + du';
            Xnew(t+1,:) = Xnew(t,:) + dt*fdyn(Xnew(t,:)',Unew(t,:)')';
        end
        Jnew = total_cost_tv(runCost_t,termCost_t,Xnew,Unew,t0,dt);
        if Jnew < J - 1e-6
            accepted=true; break;
        end
        alpha = 0.5*alpha;
    end
    if accepted
        dJ = J - Jnew; X=Xnew; U=Unew; J=Jnew;
        mu = max(mu*0.5, opts.mu0);
        if dJ < opts.tol, break; end
    else
        mu = min(mu*4,opts.mu_max);
        if mu>=opts.mu_max, break; end
    end
end
end
function J = total_cost_tv(rc, tc, X, U, t0, dt)
N = size(U,1); J = 0;
for t=1:N, J = J + rc(X(t,:)', U(t,:)', t0 + (t-1)*dt); end
J = J + tc(X(end,:)', t0 + N*dt);
end
function [A,B] = lin_dyn(fdyn, X, U, dt)
N = size(U,1); nx = size(X,2); nu = size(U,2);
A = zeros(nx,nx,N); B = zeros(nx,nu,N);
ep = 1e-6; Inx = eye(nx);
for t=1:N
    x=X(t,:)'; u=U(t,:)'; fx0 = fdyn(x,u);
    for i=1:nx
        dx=zeros(nx,1); dx(i)=ep;
        A(:,i,t) = Inx(:,i) + dt*(fdyn(x+dx,u)-fx0)/ep;
    end
    for i=1:nu
        du=zeros(nu,1); du(i)=ep;
        B(:,i,t) = dt*(fdyn(x,u+du)-fx0)/ep;
    end
end
end
function [Lx,Lu,Lxx,Luu,Lux,Phix,Phixx] = cost_derivs_tv(rc, tc, X, U, t0, dt)
N = size(U,1); nx = size(X,2); nu = size(U,2);
Lx = zeros(nx,N); Lu = zeros(nu,N);
Lxx = zeros(nx,nx,N); Luu = zeros(nu,nu,N); Lux = zeros(nu,nx,N);
ep = 1e-5;
for t=1:N
    x=X(t,:)'; u=U(t,:)'; ti = t0 + (t-1)*dt;
    for i=1:nx
        dx=zeros(nx,1); dx(i)=ep;
        Lx(i,t) = (rc(x+dx,u,ti)-rc(x-dx,u,ti))/(2*ep);
    end
    for i=1:nu
        du=zeros(nu,1); du(i)=ep;
        Lu(i,t) = (rc(x,u+du,ti)-rc(x,u-du,ti))/(2*ep);
    end
    for i=1:nx
        for j=i:nx
            di=zeros(nx,1); di(i)=ep;
            dj=zeros(nx,1); dj(j)=ep;
            L_pp=rc(x+di+dj,u,ti); L_pm=rc(x+di-dj,u,ti);
            L_mp=rc(x-di+dj,u,ti); L_mm=rc(x-di-dj,u,ti);
            Lxx(i,j,t)=(L_pp-L_pm-L_mp+L_mm)/(4*ep^2);
            Lxx(j,i,t)=Lxx(i,j,t);
        end
    end
    for i=1:nu
        for j=i:nu
            di=zeros(nu,1); di(i)=ep;
            dj=zeros(nu,1); dj(j)=ep;
            L_pp=rc(x,u+di+dj,ti); L_pm=rc(x,u+di-dj,ti);
            L_mp=rc(x,u-di+dj,ti); L_mm=rc(x,u-di-dj,ti);
            Luu(i,j,t)=(L_pp-L_pm-L_mp+L_mm)/(4*ep^2);
            Luu(j,i,t)=Luu(i,j,t);
        end
    end
    for i=1:nu
        for j=1:nx
            du=zeros(nu,1); du(i)=ep;
            dx=zeros(nx,1); dx(j)=ep;
            L_pp=rc(x+dx,u+du,ti); L_pm=rc(x+dx,u-du,ti);
            L_mp=rc(x-dx,u+du,ti); L_mm=rc(x-dx,u-du,ti);
            Lux(i,j,t)=(L_pp-L_pm-L_mp+L_mm)/(4*ep^2);
        end
    end
    Lxx(:,:,t) = 0.5*(Lxx(:,:,t)+Lxx(:,:,t)') + 1e-8*eye(nx);
    Luu(:,:,t) = 0.5*(Luu(:,:,t)+Luu(:,:,t)') + 1e-8*eye(nu);
end
xT = X(end,:)'; tT = t0 + N*dt;
Phix = zeros(nx,1); Phixx = zeros(nx,nx);
for i=1:nx
    dx=zeros(nx,1); dx(i)=ep;
    Phix(i) = (tc(xT+dx,tT)-tc(xT-dx,tT))/(2*ep);
end
for i=1:nx
    for j=i:nx
        di=zeros(nx,1); di(i)=ep;
        dj=zeros(nx,1); dj(j)=ep;
        P_pp=tc(xT+di+dj,tT); P_pm=tc(xT+di-dj,tT);
        P_mp=tc(xT-di+dj,tT); P_mm=tc(xT-di-dj,tT);
        Phixx(i,j)=(P_pp-P_pm-P_mp+P_mm)/(4*ep^2);
        Phixx(j,i)=Phixx(i,j);
    end
end
Phixx = 0.5*(Phixx+Phixx') + 1e-8*eye(nx);
end

function y=wrap(a), y = mod(a+pi,2*pi)-pi; end
