#!/usr/bin/env bash
set -euo pipefail
app_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ $(uname -s) != Linux ]]; then
  printf '이 실행기는 Linux 노트북용입니다.\n' >&2
  exit 1
fi
python_bin=${NOMAD_APP_PYTHON:-/usr/bin/python3}
if [[ ! -x $python_bin ]]; then
  printf 'Python 3을 찾지 못했습니다. 필요 명령: sudo apt install python3 python3-pyqt5 python3-matplotlib python3-numpy openssh-client iproute2 util-linux\n' >&2
  exit 1
fi
if ! "$python_bin" -c 'from PyQt5 import QtCore, QtGui, QtWidgets; import matplotlib, numpy' >/dev/null 2>&1; then
  printf '화면 실행 의존성이 없습니다. 필요 명령: sudo apt install python3-pyqt5 python3-matplotlib python3-numpy\n' >&2
  exit 1
fi
for command in ssh ip getent setsid; do
  if ! command -v "$command" >/dev/null 2>&1; then
    printf '%s 명령이 없습니다. 필요 명령: sudo apt install openssh-client iproute2 libc-bin util-linux\n' "$command" >&2
    exit 1
  fi
done
exec "$python_bin" "$app_dir/launcher.py" "$@"
