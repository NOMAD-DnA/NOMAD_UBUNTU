#!/usr/bin/env bash
set -euo pipefail
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec "$workspace/src/nomad_semantic/scripts/run.sh" "$@"
