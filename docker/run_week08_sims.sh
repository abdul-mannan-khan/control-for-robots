#!/usr/bin/env bash
# Run all Week 8 MATLAB/Octave simulations inside the Docker container.
# From the host:
#   docker run --rm -v "$(pwd)":/work control-for-robots bash docker/run_week08_sims.sh
set -euo pipefail
cd /work/week_08/sims
echo "=== Running Week 8 simulations under Octave ==="
octave --no-gui --eval "run_all"
echo ""
echo "=== Output figures ==="
ls -la figures/
