#!/usr/bin/env bash
set -eo pipefail
experiment_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$experiment_workspace/.nomad/env.sh"
export MPLCONFIGDIR="${MPLCONFIGDIR:-$experiment_workspace/log/experiment-matplotlib}"
mkdir -p "$MPLCONFIGDIR"
exec python3 "$experiment_workspace/src/nomad_bringup/experiments/experiment.py" "$@"
