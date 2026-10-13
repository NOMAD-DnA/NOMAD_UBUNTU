#!/usr/bin/env bash
set -eo pipefail
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$workspace/.nomad/env.sh"
/usr/bin/python3 "$workspace/src/nomad_bringup/gui/cleanup_previous.py"
export MPLCONFIGDIR="${MPLCONFIGDIR:-$workspace/log/gui-matplotlib}"
mkdir -p -- "$MPLCONFIGDIR"
exec /usr/bin/python3 "$workspace/src/nomad_bringup/gui/launcher.py" "$@"
