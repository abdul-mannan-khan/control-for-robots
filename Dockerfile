# Control for Robots — student runtime container
#
# What this gives you:
#   * Python 3.12 with numpy, scipy, matplotlib (for the week_09 NMPC sim
#     and for notebook-style exploration)
#   * GNU Octave  (free, open-source MATLAB-compatible interpreter —
#     runs the course's *.m simulation scripts without a MATLAB licence)
#   * Jupyter  (optional interactive front-end)
#   * A simple HTTP server on port 8000 so students can browse the
#     lecture HTML files in their own browser
#
# Build:    docker build -t control-for-robots .
# Run:      docker run --rm -it -p 8000:8000 -v "$(pwd)":/work control-for-robots
#
#   then open  http://localhost:8000/  in your browser
#
FROM python:3.12-slim

LABEL org.opencontainers.image.title="Control for Robots" \
      org.opencontainers.image.description="Student runtime for the M.Sc. Control for Robots lecture series" \
      org.opencontainers.image.licenses="MIT"

# ---- system deps: Octave (MATLAB-compatible), minimal X11 for headless plots ----
RUN apt-get update \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        octave \
        octave-control \
        octave-optim \
        octave-signal \
        ca-certificates \
        fonts-dejavu \
 && rm -rf /var/lib/apt/lists/*

# ---- Python deps for the NMPC sim + Jupyter ----
RUN pip install --no-cache-dir \
        numpy \
        scipy \
        matplotlib \
        jupyterlab

WORKDIR /work

# Default command: serve the repo on :8000 so students can open lectures in a browser
EXPOSE 8000
CMD ["python3", "-m", "http.server", "8000"]
