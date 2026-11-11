#!/usr/bin/env bash
set -euo pipefail
desktop_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
apps="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
mkdir -p "$apps"
path=${desktop_dir//\\/\\\\};path=${path//\"/\\\"};path=${path//\$/\\\$};path=${path//\`/\\\`};path=${path//%/%%}
entry="$apps/nomad-qt.desktop"
printf '%s\n' '[Desktop Entry]' 'Type=Application' 'Name=NOMAD Qt Control' \
    "Exec=/bin/bash \"$path/start-linux.sh\"" 'Terminal=true' 'Icon=applications-engineering' 'Categories=Utility;' > "$entry"
chmod 755 "$entry"
echo '앱 메뉴에 NOMAD Qt Control을 추가했습니다. SSH 로그인은 열린 터미널에서 입력하세요.'
