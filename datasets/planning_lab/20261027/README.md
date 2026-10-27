# 2026-10-27 플래닝 시험 데이터

사용자 요청에 따라 오늘 추가한 코드·환경과 함께 보존한 시점 스냅샷이다.
`planning_lab.tar.xz`는 당시 `sessions/planning_lab`의 전체 파일을 포함한다.
`manifest.json`은 파일별 SHA256·크기, 압축 파일 SHA256, 큐/결과 개수를 기록한다.

- 메인 GUI 큐: **106개**, 모두 종료 상태
- 큐 상태: 도착 58, 전진 불가 17, 안전 정지 16, 차량 외곽 겹침 8, 진척 없음 7
- 메인 결과 폴더: 113개 — 큐 기록 외 초기 CLI 검증 포함
- 초기/중지 검증까지 전체 결과: **116개**
- 실패한 시험과 원시 trajectory/events/파트별 로그, 실행별 맵·설정도 그대로 보존

초기 `verification_summary.json`은 당시 6개 검증의 보고서다. 이후 누적된 106개 전체가 성공했다는 뜻이 아니다.
`ownership.json`, `pids.json`의 PID는 과거 진단 기록이며 현재 프로세스 제어에 사용하면 안 된다.

## 확인과 복원

저장소 루트에서:

```bash
python3 scripts/restore_scenario_lab.py --verify-only
python3 scripts/restore_scenario_lab.py
./scripts/run_scenario_lab.sh --root sessions/planning_lab_archives/20261027
```

복원은 새 폴더에만 가능하며 기존 실험실 데이터는 덮어쓰지 않는다.
다른 위치가 필요하면 `--output 새폴더`를 지정한다.
체크섬 검증 후 GUI 큐의 실행 폴더 경로만 새 위치로 변경한다.
로그·manifest·과거 생성 SDF의 원래 절대 경로는 출처 증거로 보존한다.
새 시험은 현재 저장소의 맵 생성기로 다시 만들어 실행하므로 다른 PC에서도 이전 절대 경로를 실행에 사용하지 않는다.

일반 사용법과 측정 정의: [플래닝 시나리오 실험실](../../../docs/플래닝_시나리오_실험실.md).
