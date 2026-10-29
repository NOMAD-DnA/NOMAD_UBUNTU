#!/usr/bin/env bash
set -euo pipefail
COMPARISON_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export NOMAD_ROS_DOMAIN_ID="${NOMAD_COMPARISON_DOMAIN_ID:-144}"
set +u
source "$COMPARISON_WS/.nomad/env.sh"
set -u
export PYTHONPATH="$COMPARISON_WS/알고리즘_비교실험:$COMPARISON_WS/src/nomad_bringup${PYTHONPATH:+:$PYTHONPATH}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-$COMPARISON_WS/log/algorithm-comparison-matplotlib}"
mkdir -p -- "$MPLCONFIGDIR"
unset NOMAD_RUN_LOG_DIR
if [[ "${1:-}" == "--run" ]]; then
  shift
  exec python3 -m nomad_bringup.scenario_lab.runner "$@"
fi
exec python3 -m algorithm_comparison.gui "$@"
