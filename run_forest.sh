#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/.nomad/env.sh"
# Keep generated CSV/diagnostic output out of source and chat folders.
mkdir -p -- "$NOMAD_WS_ROOT/log/mvsim-runs"
nomad_forest_run_dir="$(mktemp -d "$NOMAD_WS_ROOT/log/mvsim-runs/run-XXXXXXXX")"
cd -- "$nomad_forest_run_dir"
exec ros2 launch nomad_sim forest.launch.py "$@"
