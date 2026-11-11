#!/usr/bin/env bash
set -euo pipefail
NOMAD_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
apps="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
mkdir -p "$apps"
# Desktop Entry의 Exec 인자용 이스케이프입니다. 셸 코드를 만들지 않습니다.
path=${NOMAD_DIR//\\/\\\\}
path=${path//\"/\\\"}
path=${path//\$/\\\$}
path=${path//\`/\\\`}
path=${path//%/%%}
entry="$apps/nomad-gui.desktop"
printf '%s\n' '[Desktop Entry]' 'Type=Application' 'Name=NOMAD GUI' 'Comment=NOMAD 차량 조종' \
    "Exec=/bin/bash \"$path/start.sh\" --desktop" 'Terminal=true' 'Icon=applications-engineering' 'Categories=Utility;' > "$entry"
chmod 755 "$entry"
if command -v xdg-user-dir >/dev/null; then
    desktop=$(xdg-user-dir DESKTOP)
    if [[ -d "$desktop" && "$desktop" != "$HOME" ]]; then cp "$entry" "$desktop/NOMAD-GUI.desktop"; chmod 755 "$desktop/NOMAD-GUI.desktop"; fi
fi
echo '앱 메뉴에 NOMAD GUI를 추가했습니다. 바탕화면 파일은 환경에 따라 실행 허용을 한 번 선택하세요.'
echo '압축을 푼 폴더를 이동했다면 이 스크립트를 다시 실행하세요.'
