# 외부 C++ 점군 브리지

기존 OAK·VIO 코드를 수정하지 않고 현재 좌우 rectified 영상만 구독한다. OpenCV StereoSGBM으로 호스트에서 깊이를 추정하므로 OAK 장치의 StereoDepth 깊이 출력과 알고리즘이 다르다. 실물 정확도는 별도 검증이 필요하다.

- 입력: `/oak/left/image_rect`, `/oak/right/image_rect` — 원래 동일 촬영 stamp로 정확히 짝짓기.
- 출력: `/oak/depth/points` — 1Hz, 상단206행 제외, 8픽셀 샘플, XYZ float32/12bytes, 미터, `oak_cam_a_optical_frame`.
- 변환: 좌측 rectified 좌표 → 공장 rectification 역회전 → CAM_B→CAM_C→CAM_A. 촬영 stamp 유지.
- 공장 보정: `/oak/calibration/factory` — reliable/transient_local, 저장된 공장 원문 JSON. 장치를 열거나 EEPROM을 쓰지 않는다.
- 좌우/IMU 발행률은 기존 설정 그대로. 입력 큐와 매칭 큐, 작업 큐를 제한하고 OpenCV 기본1스레드 사용.
- 0.2~10m의 유효점만 발행하고 빈 점군·오래된 입력은 버린다. 원시 영상·깊이·점군 프레임을 저장하지 않는다.

의존성: native ROS2 rclcpp/sensor_msgs/std_msgs, OpenCV core/imgproc/calib3d, Boost property_tree 헤더. DepthAI SDK는 사용하지 않는다.

보정 원본 근거:

- 공장 원문: `/home/user/Documents/ChatGPT/lig 프로젝트/log/YDLIDAR_G2/타PC_전달자료_보존_20261019/vio실 테스트/20261019_134128_높이맵_경사맵/calibration/factory_eeprom.json`
- 같은 폴더 `metadata.json`의 device_id=`1944301091BA147E00`. CAM_B의 K/D가 최신 주행가능성 수신부의 기대값과 일치한다.
- 현재 rectified K: Pi5 `/home/user/nomad_ws/config/oak_basalt_calib.json`을 부모 작업에서 읽기 보존한 `../config/oak_basalt_calib.json`. 이전 metadata의 undistorted/unrectified 출력 K를 사용하지 않는다.

독립 빌드 예시(아직 보드에서 실행한 명령이 아님):

```bash
cmake -S external_cloud -B external_cloud/build -DCMAKE_BUILD_TYPE=Release
cmake --build external_cloud/build --parallel 2
external_cloud/build/external_stereo_cloud --ros-args \
  -p factory_json:=/absolute/path/factory_eeprom.json \
  -p vio_calib_json:=/absolute/path/oak_basalt_calib.json
```

센서 없는 기하·합성 검사는 `cloud_geometry_check <factory.json> <vio_calib.json>`이다.

2026-11-06 실제 확인: 노트북 GCC13.3/native Jazzy/OpenCV4.6에서 두 실행파일 빌드 성공. 공장 baseline 0.0749524m, CAM_A 변환·역변환·거리 범위 거부와 알려진24픽셀 시차의 합성1530/1530점 통과. 실행파일 `ldd`에 Python·누락 라이브러리 없음. 보드 빌드·실센서 정확도는 이 확인에 포함하지 않는다.

실제 로컬 검증 명령:

```bash
source /opt/ros/jazzy/setup.bash
cmake -S 관련자료/통합_실시간시험_20261106/external_cloud -B /tmp/nomad_external_cloud_build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/nomad_external_cloud_build --parallel 2
/tmp/nomad_external_cloud_build/cloud_geometry_check \
  'log/YDLIDAR_G2/타PC_전달자료_보존_20261019/vio실 테스트/20261019_134128_높이맵_경사맵/calibration/factory_eeprom.json' \
  관련자료/통합_실시간시험_20261106/config/oak_basalt_calib.json
```

보정 SHA256:

```text
factory_eeprom.json f1966c7ed4e25fd278612ef1451809c3d9710b2c8e1c5f882a578a465e052603
metadata.json a299a9bc6e9ffc01d8e5f0f2370abe5f8b32e5695b0f0b22dd1305f1e9d201a3
현재 oak_basalt_calib.json a88c89639ed88fdf5d2d4cf05a592349fb9ff52bdbf3cab628887ccedc4487dd
```

DepthAI rectified 출력·기하 근거는 [공식 StereoDepth 문서](https://depthai-docs-website.readthedocs.io/projects/api/en/latest/components/nodes/stereo_depth/)와 보존한 공장 EEPROM이다. 실제 계산은 입력 영상의 현재 rectified K와 공장 rectification R을 사용한다.
