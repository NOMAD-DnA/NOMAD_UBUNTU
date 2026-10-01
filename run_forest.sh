#!/usr/bin/env bash
set -eo pipefail
# Keep Gazebo Transport local and separate from other team PCs.
export NOMAD_GZ_PARTITION="${NOMAD_GZ_PARTITION:-nomad_gazebo_$(hostname)_${NOMAD_ROS_DOMAIN_ID:-42}}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
source "$(dirname -- "${BASH_SOURCE[0]}")/.nomad/env.sh"
# Keep generated CSV/diagnostic output out of source and chat folders.
mkdir -p -- "$NOMAD_WS_ROOT/log/gazebo-runs"
nomad_forest_run_dir="$(mktemp -d "$NOMAD_WS_ROOT/log/gazebo-runs/run-XXXXXXXX")"
cd -- "$nomad_forest_run_dir"
exec ros2 launch nomad_gazebo forest.launch.py "$@"
