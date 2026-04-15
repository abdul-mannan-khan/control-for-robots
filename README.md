# Control for Robots — M.Sc. Lecture Course

A 12-week M.Sc.-level lecture series on advanced control for robotic systems, with working MATLAB simulations and output figures for every week. The course progresses from classical manipulator dynamics and computed-torque through nonlinear and optimal control, and ends with system integration for mobile robots, manipulators, and quadrotors.

## Contents

```
week_01/  Manipulator Dynamics and Computed-Torque Control
week_02/  Model Predictive Control (introduction)
week_03/  Model Predictive Control (design, simulation)
week_04/  Adaptive Control (MRAC / Slotine–Li)
week_05/  Robust Control (H-infinity style, min-max)
week_06/  Backstepping Control
week_07/  Sliding Mode Control
week_08/  Nonlinear Control Optimization  (PMP, HJB, iLQR, NMPC; per-controller optimization for all of 4–7)
week_09/  ROS2 Control Implementation  (PID / LQR / NMPC as ROS2 nodes)
week_10/  ROS2 Quadrotor Control
week_11/  ROS2 Navigation Control
week_12/  Controller Comparison and Industry Applications
results/  Reference output figures from prior MATLAB runs
```

Each `week_XX/` directory contains:
- `Week*.html` — the main lecture (open in any modern browser; MathJax renders the equations)
- `lecture/` and `lecture_public/` — supplementary lecture HTMLs for the secondary materials
- `*.m` — MATLAB simulation scripts
- `sims/` (weeks 8, 9) — expanded simulation packages with dynamics, controllers, and a `figures/` subfolder of output plots

## Running the simulations

### 🐳 Quickest path — Docker (no MATLAB licence required)

If you don't want to fight library versions, build the container. It ships with Python 3.12 (numpy, scipy, matplotlib, jupyterlab) plus GNU Octave (MATLAB-compatible; runs the `.m` scripts without a MATLAB licence).

```bash
# Build once
docker build -t control-for-robots .

# or using docker compose
docker compose build

# Run the Week 8 MATLAB/Octave simulations (saves figures to week_08/sims/figures/)
docker run --rm -v "$(pwd)":/work control-for-robots \
    bash docker/run_week08_sims.sh

# Run the Week 9 Python NMPC simulation (saves figures to week_09/sims/figures/)
docker run --rm -v "$(pwd)":/work control-for-robots \
    bash docker/run_week09_nmpc.sh

# Serve the lecture HTMLs to your browser on localhost:8000
docker run --rm -it -p 8000:8000 -v "$(pwd)":/work control-for-robots

# Optional Jupyter Lab at localhost:8888
docker compose --profile jupyter up
```

Images are built verified as of first commit: Python NMPC sim produces `final position error 1.7 mm`; Octave runs the simulations.

### MATLAB (weeks 3, 4, 5, 6, 7)

```matlab
cd week_04
adaptive_control_simulation
```

Each script generates a figure and saves it to the `results/week_XX/` directory.

### Week 8 — nonlinear control optimization (MATLAB)

```matlab
cd week_08/sims
run_all
```

Runs all five controller-family optimizers (SMC, Adaptive, Robust, Backstepping, NMPC via iLQR) on a 2-link manipulator, a unicycle mobile robot, and a planar quadrotor. Produces five PNG figures in `week_08/sims/figures/`. Total wall-time ~45 s on a recent laptop. Requires base MATLAB + Control System Toolbox only.

### Week 9 — NMPC via iLQR (Python, no ROS required)

```bash
cd week_09/sims
python3 nmpc_unicycle_sim.py
```

Closed-loop simulation of the NMPC controller on a unicycle tracking a 1 m circle; produces three PNG figures in `week_09/sims/figures/` (trajectory, commands, errors). Requires `numpy` and `matplotlib`.

## What's in Week 8 (the unifying block)

Week 8 treats every nonlinear controller from weeks 4–7 as a member of a common **optimization** framework:

- **Trajectory-level optimization:** Pontryagin's Minimum Principle, nonlinear Hamilton–Jacobi–Bellman, direct transcription, iLQR / DDP, and nonlinear MPC
- **Controller-level optimization:** Every non-optimal controller has free parameters:
  - SMC: `(λ, φ, η)` — sliding slope, boundary layer, switching gain
  - Adaptive: `(Γ, σ)` — adaptation gain, sigma modification
  - Robust: `(K_p, K_d, ρ, Λ)` — PD gains, robust-term magnitude, sliding-surface slope
  - Backstepping: `(k₁, …, kₙ)` — virtual-control gains
  - NMPC: `(N, Q, R, P, X_f)` — horizon, stage and terminal weights, terminal set

Each is formulated as a nonlinear optimization problem (min-max / Bayesian / LMI) and solved in MATLAB with `fminsearch`. All derivations are in `week_08/Week8_Control_Optimization.html`.

## License

MIT — see [LICENSE](LICENSE).

## Citation

If you use these lecture materials or simulations in teaching or research, please cite:

```
@misc{khan2026control,
  author       = {Abdul Manan Khan},
  title        = {Control for Robots -- M.Sc. Lecture Course},
  year         = {2026},
  howpublished = {\url{https://github.com/abdul-mannan-khan/control-for-robots}}
}
```

## Contact

Dr. Abdul Manan Khan — abdul.mannan.617@gmail.com
