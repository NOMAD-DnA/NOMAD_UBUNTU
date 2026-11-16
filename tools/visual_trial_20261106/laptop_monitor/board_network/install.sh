#!/usr/bin/env bash
# Manual fallback for the GUI "보드 기능 설치/정리" button. Run on the board host,
# outside any ROS container. Installs the laptop Wi-Fi agent and its watchdog
# service, removes the 2026-11-10 300-second switch and old nomad-gui-*/
# nomad-hotspot-* profiles. The current Wi-Fi connection is not changed.
set -euo pipefail
if (( EUID != 0 )); then
  exec sudo -- bash "$0" "$@"
fi
source_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
python3 - "$source_dir/wifi_agent.py" <<'PY'
import sys
from pathlib import Path
assert sys.version_info >= (3,8), 'Python 3.8 or later is required'
model=Path('/proc/device-tree/model').read_text()
assert any(n in model for n in ('Raspberry Pi 4','Raspberry Pi 5','Jetson','Xavier')), 'Unsupported board'
compile(Path(sys.argv[1]).read_bytes(),sys.argv[1],'exec')
PY
board=$(python3 -c "import sys;sys.path.insert(0,'$source_dir');import wifi_agent as a;print(a.board_identity()[0])")
request=$(mktemp)
printf '{"action":"install","board":"%s"}\n' "$board" > "$request"
python3 "$source_dir/wifi_agent.py" --request-file "$request"
