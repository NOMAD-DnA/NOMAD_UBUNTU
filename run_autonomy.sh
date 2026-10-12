#!/usr/bin/env bash
# Start Gazebo, sensor-based autonomous planning, and RViz together.
set -eo pipefail
# Match run_forest.sh: isolate Gazebo Transport from other team PCs.
export NOMAD_GZ_PARTITION="${NOMAD_GZ_PARTITION:-nomad_gazebo_$(hostname)_${NOMAD_ROS_DOMAIN_ID:-42}}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
nomad_autonomy_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$nomad_autonomy_root/src/nomad_path_planning/run_all.sh" "$@"
