#!/usr/bin/env bash
set -euo pipefail
nomad_gui_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec bash "$nomad_gui_repo/tools/visual_trial_20261106/laptop_monitor/run_app.sh" "$@"
