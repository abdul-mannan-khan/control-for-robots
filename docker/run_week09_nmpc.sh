#!/usr/bin/env bash
# Run the Week 9 Python NMPC unicycle simulation inside the container.
# From the host:
#   docker run --rm -v "$(pwd)":/work control-for-robots bash docker/run_week09_nmpc.sh
set -euo pipefail
cd /work/week_09/sims
echo "=== Running Week 9 NMPC simulation ==="
python3 nmpc_unicycle_sim.py
echo ""
echo "=== Output figures ==="
ls -la figures/
