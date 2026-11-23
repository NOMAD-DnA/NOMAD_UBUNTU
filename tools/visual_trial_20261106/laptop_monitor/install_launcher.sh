#!/usr/bin/env bash
set -euo pipefail
nomad_app_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if ! /usr/bin/python3 -c 'import PyQt5, matplotlib, numpy' >/dev/null 2>&1; then
  printf '먼저 설치: sudo apt install python3-pyqt5 python3-matplotlib python3-numpy\n' >&2
  exit 1
fi
/usr/bin/python3 - "$nomad_app_dir" <<'PY'
import os
from pathlib import Path
import sys

root = Path(sys.argv[1]).resolve()
apps = Path(os.environ.get('XDG_DATA_HOME', str(Path.home() / '.local/share'))) / 'applications'
apps.mkdir(parents=True, exist_ok=True)

def quote(value):
    return '"' + str(value).replace('\\', '\\\\').replace('"', '\\"').replace('`', '\\`').replace('$', '\\$').replace('%', '%%') + '"'

entry = apps / 'nomad-live-trial.desktop'
entry.write_text('[Desktop Entry]\nType=Application\nName=NOMAD 실시간 시험\n'
                 'Comment=보드 찾기 · 실시간 시험 · 종료\n'
                 'Exec=/bin/bash ' + quote(root / 'run_app.sh') + '\n'
                 'Icon=' + str(root / 'nomad-trial.svg') + '\n'
                 'Terminal=false\nCategories=Science;Education;\nStartupNotify=true\n', encoding='utf-8')
entry.chmod(0o755)
print('앱 목록에 등록했습니다: NOMAD 실시간 시험')
print(entry)
PY
