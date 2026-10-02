#!/usr/bin/env bash
set -eo pipefail
# Keep Gazebo Transport local and separate from other team PCs.
export NOMAD_GZ_PARTITION="${NOMAD_GZ_PARTITION:-nomad_gazebo_$(hostname)_${NOMAD_ROS_DOMAIN_ID:-42}}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
source "$(dirname -- "${BASH_SOURCE[0]}")/.nomad/env.sh"
# Existing invocation stays the same; an explicit world_file bypasses the menu.
nomad_custom_world=false
nomad_bare_map=false
for nomad_argument in "$@"; do
  case "$nomad_argument" in
    world_file:=*) nomad_custom_world=true ;;
    vegetation:=False|vegetation:=false|vegetation:=0) nomad_bare_map=true ;;
  esac
done
if ! $nomad_custom_world; then
  nomad_selector_args=()
  if $nomad_bare_map; then nomad_selector_args+=(--bare); fi
  nomad_selected_world="$(python3 "$NOMAD_WS_ROOT/src/nomad_gazebo/scripts/select_map.py" "${nomad_selector_args[@]}")"
  set -- "world_file:=$nomad_selected_world" "$@"
fi
# Keep generated CSV/diagnostic output out of source and chat folders.
mkdir -p -- "$NOMAD_WS_ROOT/log/gazebo-runs"
nomad_forest_run_dir="$(mktemp -d "$NOMAD_WS_ROOT/log/gazebo-runs/run-XXXXXXXX")"
python3 - "$NOMAD_WS_ROOT/log/gazebo-runs/active_map.json" "$@" <<'MAP_METADATA'
import datetime,json,sys
from pathlib import Path
world=next((a.split(':=',1)[1] for a in sys.argv[2:] if a.startswith('world_file:=')), '')
if world:
    path=Path(world).resolve()
    metadata={'world':str(path),'map_folder':path.parent.parent.name,'selected_at':datetime.datetime.now().isoformat()}
    Path(sys.argv[1]).write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
MAP_METADATA
cd -- "$nomad_forest_run_dir"
exec ros2 launch nomad_gazebo forest.launch.py "$@"
