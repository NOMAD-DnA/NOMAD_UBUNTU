#!/usr/bin/env bash
# Run on the board host, outside any ROS container. Installs no daemon/service.
set -euo pipefail
if (( EUID != 0 )); then
  exec sudo -- bash "$0" "$@"
fi
source_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
python3 - "$source_dir/network_agent.py" <<'PY'
import json,sys
from pathlib import Path
assert sys.version_info >= (3,8), 'Python 3.8 or later is required'
model=Path('/proc/device-tree/model').read_text()
assert any(name in model for name in ('Raspberry Pi 4','Raspberry Pi 5','Jetson','Xavier')), 'Unsupported board'
state=Path('/var/lib/nomad-gui-network/state.json')
if state.exists():
 assert json.loads(state.read_text()).get('status') not in ('queued','switching','awaiting_confirmation','restore_queued','rolling_back'), 'Finish the current switch before installing'
compile(Path(sys.argv[1]).read_bytes(),sys.argv[1],'exec')
PY
install -d -m 0755 /usr/local/lib/nomad-network
install -m 0755 "$source_dir/network_agent.py" /usr/local/lib/nomad-network/network_agent.py
printf 'Installed nomad-gui-hotspot-20261110-1. Current Wi-Fi unchanged.\n'
