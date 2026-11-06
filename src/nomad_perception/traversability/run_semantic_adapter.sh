#!/usr/bin/env bash
# Runtime entrypoint: shell + native C++, no Python or inference process.
set -euo pipefail
task_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_runtime="${NOMAD_RUNTIME_DIR:-$task_root/runtime-semantic-test}"
task_ros="${NOMAD_ROS_PREFIX:-/opt/ros/humble/install}"
task_model="${NOMAD_SEMANTIC_MODEL_CONFIG:-/home/nvidia/models/segformer/ade20k_b2/config.json}"
task_binary="$task_runtime/lib/nomad_costmap_cpp/nomad_semantic_id_adapter"
task_mapping="${NOMAD_SEMANTIC_MAPPING:-$task_runtime/share/nomad_costmap_cpp/config/ade20k_to_field12.json}"
for task_file in "$task_binary" "$task_mapping" "$task_model"; do
  if [[ ! -f "$task_file" ]]; then echo "Required file is missing: $task_file" >&2; exit 2; fi
done
export LD_LIBRARY_PATH="$task_runtime/lib:$task_ros/lib"
export AMENT_PREFIX_PATH="$task_runtime:$task_ros"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-59}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# Prevent two copies of this launcher in the same ROS domain publishing duplicates.
exec 9>"$task_root/.semantic_adapter_${ROS_DOMAIN_ID}.lock"
flock -n 9 || { echo 'A semantic adapter launcher already owns this domain.' >&2; exit 2; }
exec "$task_binary" --mapping "$task_mapping" --source-model-config "$task_model" "$@"
