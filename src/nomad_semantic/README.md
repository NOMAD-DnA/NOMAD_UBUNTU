# 의미분할

```bash
./.nomad/install_semantic.sh  # 최초: 추론 환경 설치·패키지 빌드 (x86_64 CUDA)
./run_semantic.sh            # RGB → labels/confidence/colors/overlay, 최대 10Hz
```

입력 `/oak/rgb/image_raw`, 출력 `/nomad/semantic/{labels,confidence,colors,overlay}`.
출력은 입력 RGB의 촬영 시각·frame을 유지한다. 지도 생성은 `nomad_perception` 담당.

- 모델: `models/nomad_b0/model.onnx`, 클래스·전처리: 옆 `metadata.json`.
- 모델 교체: `./run_semantic.sh -p model:=/경로/model.onnx` (metadata.json 필수).
- CPU 확인: `./run_semantic.sh -p backend:=cpu`. CUDA 실패 시 자동 CPU 전환하지 않는다.
- Jetson 전용 설치·성능은 미검증. 모델 사용 조건은 `models/nomad_b0/PROVENANCE.md`.
