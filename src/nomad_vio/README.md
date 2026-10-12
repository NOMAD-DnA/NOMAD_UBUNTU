# 위치추정

```bash
./scripts/run_module.sh localization                   # Basalt + GNSS 융합·공통 출력
./scripts/run_module.sh localization method:=openvins   # OpenVINS로 교체
```

설정: `config/vio/`, 융합: `../nomad_localization/config/fusion.yaml`.
공통 출력 `/nomad/localization/odometry`는 odom→base_link 기준, 상태 `/nomad/localization/status`.
Gazebo GT 동적TF를 끄고 초기정렬 이동 후 READY를 확인한다. GNSS 단독은 자세를 측정하지 않는다.
자동 재시작은 기본 off. 단독 VIO·공통 어댑터·설치 계약은 [상세](docs/archive/README_details.md).
