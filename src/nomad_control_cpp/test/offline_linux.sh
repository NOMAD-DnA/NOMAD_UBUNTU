#!/usr/bin/env bash
# 로컬 가짜 장치 검사만 실행합니다. SSH/실제 VESC 연결은 만들지 않습니다.
set -euo pipefail
project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$project"
count=0
for file in scripts/*.sh linux/*.sh desktop/*.sh test/offline_linux.sh; do
    bash -n "$file"
    count=$((count+1))
done
echo "$count Bash scripts parsed successfully"
if [[ ${1:-} == --syntax-only ]]; then exit 0; fi
cmake -S . -B build-sanitize -G Ninja -DNOMAD_BUILD_ROS2=OFF -DCMAKE_BUILD_TYPE=Debug \
    '-DCMAKE_CXX_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer' \
    -DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined
cmake --build build-sanitize -j2
ctest --test-dir build-sanitize --output-on-failure
cmake -S desktop -B build-desktop-linux -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-desktop-linux -j2
ctest --test-dir build-desktop-linux --output-on-failure
python3 test/test_linux_launcher.py
