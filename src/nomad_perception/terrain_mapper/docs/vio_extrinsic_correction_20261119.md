# 매퍼 내부 VIO 외부파라미터 보정 (2026-11-19)

PC에서 구현하고 ROS 2 Jazzy Release 빌드와 CTest 3개를 통과했습니다. Jetson ARM 빌드, 설치와 실주행은 아직 수행하지 않았습니다.

## 적용한 보정

Pi5의 `/home/user/nomad_ws/config/oak_basalt_calib.json`과 Jetson의 `/home/nvidia/nomad_ws/tools/visual_trial_20261106/config/oak_basalt_calib.json`을 읽어 비교했다. 두 파일의 첫 번째 `T_imu_cam`은 각각 IMU에서 본 rectified CAM_B의 위치와 방향이다. 두 카메라 항목 모두 같은 왼쪽 강체 변환으로 일치하는 것을 검사한다. 회전 차이는 2.49도다. 앞선 원시 가속도와 바닥 법선 비교에서 나온 약 2.9도를 사용하지 않는다.

`T_new_imu_from_old_imu = T_vio_imu_from_rect0 * inverse(T_reference_imu_from_rect0)`

기존 factory 기반 `IMU <- CAM_B`와 `IMU <- CAM_A` 변환에 위 변환을 왼쪽에서 한 번 곱한다. 그 후 기존 코드가 이 변환에서 LiDAR 및 ZED 장착 변환과 의미 투영 변환을 유도한다. OAK 점군에만 별도로 회전을 적용하면 다른 센서와 상대 배치가 어긋나므로, 매퍼 내부의 공통 외부파라미터 생성 지점에 적용한다.

VIO 파일에서는 카메라 중심의 위치를 유지하면서 방향을 바꾸었다. 이에 정확히 맞추기 위해 보정 강체 변환에는 약 `[-0.000001246, -0.000015627, -0.000275698] m`의 병진도 포함한다. 이는 카메라 중심을 유지하는 좌표 변환 항이고 새로 측정한 센서 위치가 아니다.

코드 내부 상수는 이 시점에 수집한 보정 파일에 묶인다. 이후 VIO 외부파라미터를 다시 바꾸면 매퍼의 상수도 함께 갱신해야 한다. 이 코드는 Pi5 설정을 실시간으로 읽거나 자동으로 추정하지 않는다.

## 변경 범위

매퍼 패키지의 외부파라미터 보정 헤더, 이를 호출하는 factory 처리 위치, 시작 로그와 검사 등록만 바꾼다. 코드 변경은 보정 헤더, mapper.cpp, 전용 검사와 CMake 등록에 한정한다.

VIO와 센서 발행기, 원시 점군 토픽, 공개 자세 및 TF, semantic/core/cost 소스, GUI, active 설정은 수정하지 않는다. 매퍼가 만드는 지도 좌표의 OAK와 LiDAR 및 ZED 배치는 함께 영향을 받는다. 거리 제한과 높이 제거 기준은 보존한다.

## 검증 범위

전용 C++ 검사는 두 카메라의 실제 보정 파일 일치, 회전 크기와 강체 성질, 카메라 사이 상대 변환, LiDAR 및 ZED 투영 왕복, 알려진 평지와 10도 및 20도 경사 복원을 확인한다. 기존 수평거리 제한과 높이 필터 검사도 실행한다. 보정 33개, 거리 제한 39개, 높이 필터 81개 검사가 통과했다.

이 검증은 좌표 변환 구현 검증이다. 실제 깊이 오차나 실제 주행에서 평지 오판이 사라졌다는 증거가 아니다. 센서와 전체 지도 및 차량 제어를 시작하지 않는다.

## 근거

카메라와 IMU 사이 회전 및 병진을 명시적 외부파라미터로 다루는 근거는 Furgale, Rehder, Siegwart의 2013년 논문 III-B절을 확인했다. 위 보정 행렬의 구성은 이 프로젝트의 두 실제 변환 사이 좌표 변환으로 직접 유도한 것이다. 논문은 이 장치의 2.49도 보정값이나 보정 효과를 입증하지 않는다.

[Unified Temporal and Spatial Calibration for Multi-Sensor Systems 원문](https://vigir.missouri.edu/~gdesouza/Research/Conference_CDs/IEEE_IROS_2013/media/files/0240.pdf)
