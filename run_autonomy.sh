#!/usr/bin/env bash
# Start Gazebo, sensor-based autonomous planning, and RViz together.
set -eo pipefail
nomad_autonomy_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$nomad_autonomy_root/src/nomad_path_planning/run_all.sh" "$@"
