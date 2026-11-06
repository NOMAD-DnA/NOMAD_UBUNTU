#!/usr/bin/env bash
set -euo pipefail
task_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$task_ws/src/nomad_perception/traversability/run_semantic_adapter.sh" "$@"
