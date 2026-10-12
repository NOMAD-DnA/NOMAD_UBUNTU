#!/usr/bin/env bash
set -euo pipefail
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ "$(uname -m)" != x86_64 ]]; then
  echo 'CUDA installer supports x86_64. Jetson needs its matching TensorRT/ONNXRuntime environment.' >&2
  exit 2
fi
python3 -m venv --system-site-packages "$workspace/.nomad/deps/semantic"
"$workspace/.nomad/deps/semantic/bin/python" -m pip install -r "$workspace/.nomad/semantic/requirements-cuda.txt"
set +u
source "$workspace/.nomad/env.sh"
cd "$workspace"
colcon build --base-paths src --symlink-install --packages-select nomad_semantic
