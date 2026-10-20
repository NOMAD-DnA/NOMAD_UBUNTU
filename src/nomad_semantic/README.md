# 의미분할

```bash
./.nomad/install_semantic.sh  # 최초: 추론 환경 설치·패키지 빌드 (x86_64 CUDA)
./scripts/run_semantic.sh            # RGB → labels/confidence/colors/overlay, 최대 10Hz
```

입력 `/oak/rgb/image_raw`, 출력 `/nomad/semantic/{labels,confidence,colors,overlay}`.
출력은 입력 RGB의 촬영 시각·frame을 유지한다. 지도 생성은 `nomad_perception` 담당.

- 모델: `models/nomad_b0/model.onnx`, 클래스·전처리: 옆 `metadata.json`.
- 모델 교체: `./scripts/run_semantic.sh -p model:=/경로/model.onnx` (metadata.json 필수).
- CPU 확인: `./scripts/run_semantic.sh -p backend:=cpu`. CUDA 실패 시 자동 CPU 전환하지 않는다.
- Jetson 전용 설치·성능은 미검증. 모델 사용 조건은 `models/nomad_b0/PROVENANCE.md`.

### 입력·발행 지연 진단

워크스페이스에서 다음 명령으로 GUI를 새로 실행하면 지도와 의미분할이 같은 폴더에
각각 `map_*.jsonl`, `semantic_*.jsonl`을 기록한다. 이미 실행 중인 노드에는 적용되지 않는다.

```bash
NOMAD_MAP_DIAGNOSTICS_DIR="$HOME/nomad_ws/log/map_diagnostics" ./scripts/run_gui.sh
```

기본 비활성. `diagnostics_directory` 파라미터로도 지정할 수 있다.
RGB 콜백 수신, 최신 입력 교체/만료, 추론 시작/종료, 각 출력의 발행 시작/완료 및
메시지 크기를 기록한다. 정수 `stamp_ns`가 원본 촬영 시각이며 지도 로그와 매칭하는 키다.
`wall_ns`는 실제 시각, `mono_ns`는 같은 호스트에서 경과 시간 비교용이다.
`publish_complete`는 publish 함수 반환이며 DDS 전달·수신 완료를 보장하지 않는다.
수신 기록은 콜백 실행 시점이다. 영상 본문은 기록하지 않는다.
기존 nomad_logging의 비동기 기록기를 공유하며 8192건 큐가 가득 차면 진단만 버린다.
종료 시 `diagnostics_closed.dropped`를 확인한다. 강제 종료 시 마지막 기록은 유실될 수 있다.
