#!/usr/bin/env bash
set -euo pipefail
LAB_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export NOMAD_ROS_DOMAIN_ID="${NOMAD_SCENARIO_DOMAIN_ID:-142}"
set +u
source "$LAB_WS/.nomad/env.sh"
set -u
export PYTHONPATH="$LAB_WS/src/nomad_bringup${PYTHONPATH:+:$PYTHONPATH}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-$LAB_WS/log/scenario-lab-matplotlib}"
mkdir -p -- "$MPLCONFIGDIR"
unset NOMAD_RUN_LOG_DIR
if [[ "${1:-}" == "--run" ]]; then
  shift
  exec python3 -m nomad_bringup.scenario_lab.runner "$@"
fi
exec python3 -m nomad_bringup.scenario_lab.gui "$@"
