#!/usr/bin/env bash
set -eo pipefail
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: ./run_semantic.sh [-p model:=/path/model.onnx] [-p backend:=cuda|cpu|trt] [-p view:=true]'
  exit 0
fi
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
while [[ ! -f "$workspace/.nomad/env.sh" ]]; do
  if [[ "$workspace" == / ]]; then echo 'NOMAD workspace not found.' >&2; exit 2; fi
  workspace=$(dirname -- "$workspace")
done
source "$workspace/.nomad/env.sh"
semantic_python=${NOMAD_SEMANTIC_PYTHON:-"$workspace/.nomad/deps/semantic/bin/python"}
if [[ ! -x "$semantic_python" ]]; then echo 'Run ./.nomad/install_semantic.sh first.' >&2; exit 2; fi
export PYTHONPATH="$workspace/src/nomad_semantic${PYTHONPATH:+:$PYTHONPATH}"
exec "$semantic_python" -m nomad_semantic.node --ros-args -p use_sim_time:=true -p rate:=10.0 "$@"
