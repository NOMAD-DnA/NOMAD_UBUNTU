#!/usr/bin/env bash
set -euo pipefail
repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec bash "$repo_root/tools/visual_trial_20261106/laptop_monitor/run_app.sh" "$@"
