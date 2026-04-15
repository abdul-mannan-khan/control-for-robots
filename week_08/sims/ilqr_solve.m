function [X, U, J, iters] = ilqr_solve(fdyn, runCost, termCost, x0, U_init, dt, opts)
% Generic iLQR solver (first-order dynamics, quadratic-like cost).
% fdyn(x,u)        : continuous-time dynamics (nx x 1)
% runCost(x,u)     : returns [L, Lx(nx), Lu(nu), Lxx, Luu, Lux] scalars/vectors/mats
% termCost(xN)     : returns [Phi, Phix, Phixx]
% x0  : nx x 1 initial state
% U_init : N x nu initial control sequence
% dt : integration step (Euler)
% opts.max_iter, opts.tol, opts.mu0, opts.mu_max, opts.verbose

if nargin<7, opts=struct(); end
if ~isfield(opts,'max_iter'), opts.max_iter = 50; end
if ~isfield(opts,'tol'),      opts.tol = 1e-4; end
if ~isfield(opts,'mu0'),      opts.mu0 = 1e-3; end
if ~isfield(opts,'mu_max'),   opts.mu_max = 1e4; end
if ~isfield(opts,'verbose'),  opts.verbose = false; end

N  = size(U_init,1);
nu = size(U_init,2);
nx = numel(x0);
U  = U_init;
mu = opts.mu0;

X  = rollout(fdyn, x0, U, dt);
J  = total_cost(runCost, termCost, X, U);

for iters=1:opts.max_iter
    % ---- Backward pass: build Jacobians & cost Hessians via finite differences
    [A, B] = linearize_dyn(fdyn, X, U, dt);
    [Lx, Lu, Lxx, Luu, Lux, Phix, Phixx] = cost_derivs(runCost, termCost, X, U);

    Vx  = Phix;                    % nx x 1
    Vxx = Phixx;                   % nx x nx
    k_ff = zeros(nu, N);
    K_fb = zeros(nu, nx, N);
    backpass_fail = false;

    for t = N:-1:1
        Qx  = Lx(:,t)     + A(:,:,t)' * Vx;
        Qu  = Lu(:,t)     + B(:,:,t)' * Vx;
        Qxx = Lxx(:,:,t)  + A(:,:,t)' * Vxx * A(:,:,t);
        Quu = Luu(:,:,t)  + B(:,:,t)' * Vxx * B(:,:,t) + mu*eye(nu);
        Qux = Lux(:,:,t)  + B(:,:,t)' * Vxx * A(:,:,t);
        Quu = 0.5*(Quu+Quu');
        [R,p] = chol(Quu);
        if p>0
            backpass_fail = true; break;
        end
        kt = -R\(R'\Qu);
        Kt = -R\(R'\Qux);
        k_ff(:,t)    = kt;
        K_fb(:,:,t)  = Kt;
        Vx  = Qx  + Kt'*Quu*kt + Kt'*Qu + Qux'*kt;
        Vxx = Qxx + Kt'*Quu*Kt + Kt'*Qux + Qux'*Kt;
        Vxx = 0.5*(Vxx+Vxx');
    end

    if backpass_fail
        mu = min(mu*4, opts.mu_max);
        if opts.verbose, fprintf('  [iLQR] backpass non-PD; mu -> %g\n', mu); end
        if mu >= opts.mu_max, break; end
        continue;
    end

    % ---- Forward pass with line search on step size alpha
    alpha = 1.0;
    accepted = false;
    for ls = 1:12
        Unew = zeros(size(U));
        Xnew = zeros(size(X));
        Xnew(1,:) = X(1,:);
        for t=1:N
            dx = (Xnew(t,:) - X(t,:))';
            du = alpha * k_ff(:,t) + K_fb(:,:,t) * dx;
            Unew(t,:) = U(t,:) + du';
            Xnew(t+1,:) = Xnew(t,:) + dt * fdyn(Xnew(t,:)', Unew(t,:)')';
        end
        Jnew = total_cost(runCost, termCost, Xnew, Unew);
        if Jnew < J - 1e-6
            accepted = true;
            break;
        end
        alpha = 0.5*alpha;
    end

    if accepted
        dJ = J - Jnew;
        X  = Xnew; U = Unew; J = Jnew;
        mu = max(mu*0.5, opts.mu0);
        if opts.verbose
            fprintf('  [iLQR] iter %d  J=%.5e  alpha=%.3f  dJ=%.3e  mu=%.2e\n',iters,J,alpha,dJ,mu);
        end
        if dJ < opts.tol, break; end
    else
        mu = min(mu*4, opts.mu_max);
        if opts.verbose, fprintf('  [iLQR] iter %d line-search fail; mu -> %g\n',iters,mu); end
        if mu >= opts.mu_max, break; end
    end
end
end

% ====================================================================
function X = rollout(fdyn, x0, U, dt)
N  = size(U,1);
nx = numel(x0);
X  = zeros(N+1, nx);
X(1,:) = x0';
for t=1:N
    X(t+1,:) = X(t,:) + dt * fdyn(X(t,:)', U(t,:)')';
end
end
% ====================================================================
function J = total_cost(runCost, termCost, X, U)
N = size(U,1);
J = 0;
for t=1:N, J = J + runCost(X(t,:)', U(t,:)'); end
J = J + termCost(X(end,:)');
end
% ====================================================================
function [A, B] = linearize_dyn(fdyn, X, U, dt)
N  = size(U,1);
nx = size(X,2);
nu = size(U,2);
A  = zeros(nx,nx,N);
B  = zeros(nx,nu,N);
eps_fd = 1e-6;
for t=1:N
    x = X(t,:)'; u = U(t,:)';
    fx0 = fdyn(x,u);
    I_nx = eye(nx);
    for i=1:nx
        dx = zeros(nx,1); dx(i) = eps_fd;
        A(:,i,t) = I_nx(:,i) + dt*(fdyn(x+dx,u) - fx0)/eps_fd;
    end
    for i=1:nu
        du = zeros(nu,1); du(i) = eps_fd;
        B(:,i,t) = dt*(fdyn(x,u+du) - fx0)/eps_fd;
    end
end
end
% ====================================================================
function [Lx, Lu, Lxx, Luu, Lux, Phix, Phixx] = cost_derivs(runCost, termCost, X, U)
N  = size(U,1);
nx = size(X,2); nu = size(U,2);
Lx  = zeros(nx,N);  Lu  = zeros(nu,N);
Lxx = zeros(nx,nx,N);
Luu = zeros(nu,nu,N);
Lux = zeros(nu,nx,N);
eps_fd = 1e-5;
for t=1:N
    x = X(t,:)'; u = U(t,:)';
    L0 = runCost(x,u);
    % Gradient in x
    for i=1:nx
        dx = zeros(nx,1); dx(i)=eps_fd;
        Lx(i,t) = (runCost(x+dx,u) - runCost(x-dx,u))/(2*eps_fd);
    end
    for i=1:nu
        du = zeros(nu,1); du(i)=eps_fd;
        Lu(i,t) = (runCost(x,u+du) - runCost(x,u-du))/(2*eps_fd);
    end
    % Hessians
    for i=1:nx
        for j=i:nx
            dxi=zeros(nx,1); dxi(i)=eps_fd;
            dxj=zeros(nx,1); dxj(j)=eps_fd;
            L_pp = runCost(x+dxi+dxj,u);
            L_pm = runCost(x+dxi-dxj,u);
            L_mp = runCost(x-dxi+dxj,u);
            L_mm = runCost(x-dxi-dxj,u);
            Lxx(i,j,t) = (L_pp - L_pm - L_mp + L_mm) / (4*eps_fd^2);
            Lxx(j,i,t) = Lxx(i,j,t);
        end
    end
    for i=1:nu
        for j=i:nu
            dui=zeros(nu,1); dui(i)=eps_fd;
            duj=zeros(nu,1); duj(j)=eps_fd;
            L_pp = runCost(x,u+dui+duj);
            L_pm = runCost(x,u+dui-duj);
            L_mp = runCost(x,u-dui+duj);
            L_mm = runCost(x,u-dui-duj);
            Luu(i,j,t) = (L_pp - L_pm - L_mp + L_mm) / (4*eps_fd^2);
            Luu(j,i,t) = Luu(i,j,t);
        end
    end
    for i=1:nu
        for j=1:nx
            du=zeros(nu,1); du(i)=eps_fd;
            dx=zeros(nx,1); dx(j)=eps_fd;
            L_pp = runCost(x+dx,u+du);
            L_pm = runCost(x+dx,u-du);
            L_mp = runCost(x-dx,u+du);
            L_mm = runCost(x-dx,u-du);
            Lux(i,j,t) = (L_pp - L_pm - L_mp + L_mm) / (4*eps_fd^2);
        end
    end
    % symmetrize
    Lxx(:,:,t) = 0.5*(Lxx(:,:,t)+Lxx(:,:,t)') + 1e-8*eye(nx);
    Luu(:,:,t) = 0.5*(Luu(:,:,t)+Luu(:,:,t)') + 1e-8*eye(nu);
end
% Terminal cost
xT = X(end,:)';
Phi0 = termCost(xT);
Phix  = zeros(nx,1);
Phixx = zeros(nx,nx);
for i=1:nx
    dx=zeros(nx,1); dx(i)=eps_fd;
    Phix(i) = (termCost(xT+dx) - termCost(xT-dx))/(2*eps_fd);
end
for i=1:nx
    for j=i:nx
        dxi=zeros(nx,1); dxi(i)=eps_fd;
        dxj=zeros(nx,1); dxj(j)=eps_fd;
        P_pp = termCost(xT+dxi+dxj);
        P_pm = termCost(xT+dxi-dxj);
        P_mp = termCost(xT-dxi+dxj);
        P_mm = termCost(xT-dxi-dxj);
        Phixx(i,j) = (P_pp - P_pm - P_mp + P_mm) / (4*eps_fd^2);
        Phixx(j,i) = Phixx(i,j);
    end
end
Phixx = 0.5*(Phixx+Phixx') + 1e-8*eye(nx);
end
