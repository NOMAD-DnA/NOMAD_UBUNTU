# 플래닝

```bash
./scripts/run_module.sh planning  # 기존 메뉴에서 GPP/LPP 선택
```

입력: 융합 pose·전역/지역 costmap·목표·센서 상태·Control 피드백.
출력: `/nomad/planning/drive_command`의 목표 속도·조향각·정지 요청.
설정: `config/planning.yaml`; 통합 override: `../nomad_perception/config/terrain_planning.yaml`.
GPP/LPP 전략은 `gpp/`, `lpp/`; 후진 복구는 `recovery_supervisor.py`.
후진은 기록된 복귀 경로와 최신 센서가 필요하다. 현재8번 맵에서 복구 이력 부족 정지는 미해결이다.
알고리즘·복구·인터페이스 계약은 [상세](docs/archive/README_details.md).
