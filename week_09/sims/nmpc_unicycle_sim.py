#!/usr/bin/env python3
"""Standalone offline simulation of the NMPC / iLQR unicycle controller from
`task_nmpc_node.py`. Runs a closed-loop simulation (no ROS required),
records trajectory + control, and saves three figures into ./figures/:

  figures/nmpc_unicycle_trajectory.png   — XY path vs reference circle
  figures/nmpc_unicycle_commands.png     — v(t), omega(t)
  figures/nmpc_unicycle_errors.png       — position / heading error vs time

The controller itself is identical in logic to the ROS2 node; this script
drives it in a closed loop with the nominal unicycle kinematics.
"""
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class NMPCUnicycle:
    def __init__(self, N=15, dt=0.1, v_ref=0.3, radius=1.0, max_iter=5):
        self.N, self.dt = N, dt
        self.v_ref, self.radius = v_ref, radius
        self.max_iter = max_iter
        self.nx, self.nu = 3, 2
        self.Q_run = np.diag([4.0, 4.0, 0.5])
        self.R_run = np.diag([0.05, 0.05])
        self.Q_term = 20.0 * self.Q_run
        self.U_warm = np.zeros((N, 2))
        self.mu = 1e-3

    def f(self, x, u):
        return np.array([u[0] * np.cos(x[2]), u[0] * np.sin(x[2]), u[1]])

    def step(self, x, u):
        xn = x + self.dt * self.f(x, u)
        xn[2] = wrap(xn[2])
        return xn

    def ref(self, t):
        w = self.v_ref / self.radius
        return np.array([self.radius * np.cos(w * t),
                         self.radius * np.sin(w * t),
                         w * t + np.pi / 2])

    def rollout(self, x0, U):
        X = np.zeros((self.N + 1, 3))
        X[0] = x0
        for k in range(self.N):
            X[k + 1] = self.step(X[k], U[k])
        return X

    def linearize(self, X, U):
        eps = 1e-6
        A = np.zeros((self.N, 3, 3))
        B = np.zeros((self.N, 3, 2))
        I = np.eye(3)
        for k in range(self.N):
            x, u = X[k], U[k]
            f0 = self.f(x, u)
            for i in range(3):
                dx = np.zeros(3); dx[i] = eps
                A[k, :, i] = I[:, i] + self.dt * (self.f(x + dx, u) - f0) / eps
            for i in range(2):
                du = np.zeros(2); du[i] = eps
                B[k, :, i] = self.dt * (self.f(x, u + du) - f0) / eps
        return A, B

    def total_cost(self, X, U, t0):
        J = 0.0
        for k in range(self.N):
            e = X[k] - self.ref(t0 + k * self.dt)
            e[2] = wrap(e[2])
            J += e @ self.Q_run @ e + U[k] @ self.R_run @ U[k]
        e = X[-1] - self.ref(t0 + self.N * self.dt)
        e[2] = wrap(e[2])
        J += e @ self.Q_term @ e
        return J

    def ilqr_pass(self, X, U, t0):
        A, B = self.linearize(X, U)
        k_ff = np.zeros((self.N, 2))
        K_fb = np.zeros((self.N, 2, 3))
        eT = X[-1] - self.ref(t0 + self.N * self.dt); eT[2] = wrap(eT[2])
        Vx = 2.0 * self.Q_term @ eT
        Vxx = 2.0 * self.Q_term.copy()
        for k in reversed(range(self.N)):
            e = X[k] - self.ref(t0 + k * self.dt); e[2] = wrap(e[2])
            Lx = 2.0 * self.Q_run @ e
            Lu = 2.0 * self.R_run @ U[k]
            Lxx = 2.0 * self.Q_run
            Luu = 2.0 * self.R_run
            Lux = np.zeros((2, 3))
            Qx = Lx + A[k].T @ Vx
            Qu = Lu + B[k].T @ Vx
            Qxx = Lxx + A[k].T @ Vxx @ A[k]
            Quu = Luu + B[k].T @ Vxx @ B[k] + self.mu * np.eye(2)
            Qux = Lux + B[k].T @ Vxx @ A[k]
            Quu = 0.5 * (Quu + Quu.T)
            try:
                L = np.linalg.cholesky(Quu)
            except np.linalg.LinAlgError:
                return X, U, False, 0.0
            k_ff[k] = -np.linalg.solve(L.T, np.linalg.solve(L, Qu))
            K_fb[k] = -np.linalg.solve(L.T, np.linalg.solve(L, Qux))
            Vx = Qx + K_fb[k].T @ Quu @ k_ff[k] + K_fb[k].T @ Qu + Qux.T @ k_ff[k]
            Vxx = Qxx + K_fb[k].T @ Quu @ K_fb[k] + K_fb[k].T @ Qux + Qux.T @ K_fb[k]
            Vxx = 0.5 * (Vxx + Vxx.T)
        J_old = self.total_cost(X, U, t0)
        alpha = 1.0
        for _ in range(10):
            Xn = np.zeros_like(X); Un = np.zeros_like(U)
            Xn[0] = X[0]
            for k in range(self.N):
                dx = Xn[k] - X[k]; dx[2] = wrap(dx[2])
                du = alpha * k_ff[k] + K_fb[k] @ dx
                Un[k] = U[k] + du
                Un[k, 0] = np.clip(Un[k, 0], -1.5, 1.5)
                Un[k, 1] = np.clip(Un[k, 1], -2.0, 2.0)
                Xn[k + 1] = self.step(Xn[k], Un[k])
            Jn = self.total_cost(Xn, Un, t0)
            if Jn < J_old - 1e-6:
                return Xn, Un, True, J_old - Jn
            alpha *= 0.5
        return X, U, False, 0.0


def simulate(T_sim=20.0, x0=np.array([0.5, -0.2, 0.0])):
    ctrl = NMPCUnicycle()
    dt = ctrl.dt
    N_sim = int(T_sim / dt)
    x = x0.copy()
    hist_t = np.zeros(N_sim)
    hist_x = np.zeros((N_sim, 3))
    hist_u = np.zeros((N_sim, 2))
    hist_xr = np.zeros((N_sim, 3))
    for k in range(N_sim):
        t = k * dt
        X = ctrl.rollout(x, ctrl.U_warm)
        for _ in range(ctrl.max_iter):
            X, ctrl.U_warm, ok, dJ = ctrl.ilqr_pass(X, ctrl.U_warm, t)
            if ok:
                ctrl.mu = max(ctrl.mu * 0.5, 1e-6)
                if dJ < 1e-4:
                    break
            else:
                ctrl.mu = min(ctrl.mu * 4, 1e4)
        u = ctrl.U_warm[0].copy()
        hist_t[k] = t
        hist_x[k] = x
        hist_u[k] = u
        hist_xr[k] = ctrl.ref(t)
        x = ctrl.step(x, u)
        ctrl.U_warm = np.vstack([ctrl.U_warm[1:], ctrl.U_warm[-1:]])
    return hist_t, hist_x, hist_u, hist_xr


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    fig_dir = os.path.join(here, "figures")
    os.makedirs(fig_dir, exist_ok=True)

    t, X, U, Xr = simulate()

    # --- Figure 1: XY trajectory vs reference circle ---
    fig, ax = plt.subplots(figsize=(7, 7))
    th = np.linspace(0, 2 * np.pi, 200)
    ax.plot(np.cos(th), np.sin(th), "k--", lw=1.4, label="reference circle (r = 1 m)")
    ax.plot(X[:, 0], X[:, 1], "b-", lw=1.8, label="NMPC-controlled unicycle")
    ax.plot(X[0, 0], X[0, 1], "go", markersize=10, label="start")
    ax.plot(X[-1, 0], X[-1, 1], "r^", markersize=10, label="end")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("NMPC (iLQR) Unicycle Trajectory Tracking — Week 9 / Week 8 §9")
    ax.set_aspect("equal"); ax.grid(True); ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(os.path.join(fig_dir, "nmpc_unicycle_trajectory.png"), dpi=130)
    plt.close(fig)

    # --- Figure 2: control signals ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
    ax1.plot(t, U[:, 0], "b-", lw=1.5, label="v (linear velocity)")
    ax1.axhline(1.5, color="r", ls=":", alpha=0.6, label="v clamp ±1.5")
    ax1.axhline(-1.5, color="r", ls=":", alpha=0.6)
    ax1.set_ylabel("v [m/s]"); ax1.grid(True); ax1.legend(loc="upper right")
    ax1.set_title("NMPC Control Commands — iLQR feedforward + feedback")
    ax2.plot(t, U[:, 1], "g-", lw=1.5, label="ω (angular velocity)")
    ax2.axhline(2.0, color="r", ls=":", alpha=0.6, label="ω clamp ±2.0")
    ax2.axhline(-2.0, color="r", ls=":", alpha=0.6)
    ax2.set_ylabel("ω [rad/s]"); ax2.set_xlabel("t [s]"); ax2.grid(True); ax2.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(fig_dir, "nmpc_unicycle_commands.png"), dpi=130)
    plt.close(fig)

    # --- Figure 3: errors ---
    ep = np.linalg.norm(X[:, :2] - Xr[:, :2], axis=1)
    eth = wrap(X[:, 2] - Xr[:, 2])
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(t, ep, "b-", lw=1.5, label="position error ||e_p|| [m]")
    ax.plot(t, np.abs(eth), "g-", lw=1.5, label="heading error |e_θ| [rad]")
    ax.set_xlabel("t [s]"); ax.set_ylabel("error magnitude")
    ax.set_title("NMPC closed-loop tracking errors")
    ax.grid(True); ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(fig_dir, "nmpc_unicycle_errors.png"), dpi=130)
    plt.close(fig)

    print("Figures saved to:", fig_dir)
    print("  final position error : %.4f m" % ep[-1])
    print("  final heading error  : %.4f rad" % abs(eth[-1]))
    print("  mean |v|             : %.3f m/s" % np.mean(np.abs(U[:, 0])))
    print("  mean |ω|             : %.3f rad/s" % np.mean(np.abs(U[:, 1])))


if __name__ == "__main__":
    main()
