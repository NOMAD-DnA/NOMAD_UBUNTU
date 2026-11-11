#!/usr/bin/env bash
# 공유폴더 원본 → 이 노트북의 ~/nomad_gui 로 GUI를 설치·업데이트한다.
# 핫스팟을 켜 공유폴더가 끊겨도 로컬 사본은 계속 동작한다. 실행 중인 GUI는 바꾸지 않는다.
set -euo pipefail
src=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec /usr/bin/python3 -I "$src/laptop_monitor/local_install.py" install --source "$src" "$@"
