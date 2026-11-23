#!/bin/bash
# 2026-11-17 VIO 시험기: Pi5 융합기를 window_xy 시험판으로 전환(on)/원래대로(off)/상태(status).
# Pi5의 ~/nomad_fusion_test_kit/fusion_test_swap.sh 를 실행한다(키트·시험판 빌드는 Pi5에 미리 설치).
set -e
test -x ~/nomad_fusion_test_kit/fusion_test_swap.sh || { echo "Pi5에 시험판 키트가 없습니다(~/nomad_fusion_test_kit)."; exit 2; }
bash ~/nomad_fusion_test_kit/fusion_test_swap.sh status
