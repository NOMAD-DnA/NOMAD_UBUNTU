# NOMAD SegFormer-B0

- 기반: https://huggingface.co/nvidia/segformer-b0-finetuned-ade-512-512
- NOMAD 2번 Gazebo 맵 정답 1,000장 학습 / 200장 검증, 10클래스.
- 입력: BGR float32, 1×3×512×512. 출력: 10클래스 logits.
- 동일맵 검증 mIoU 87.96%. 다른 맵·실물 정확도는 별도 검증 필요.
- NVIDIA SegFormer 라이선스 적용: 연구·평가용 비상업적 사용. LICENSE-SegFormer 원문 포함.
