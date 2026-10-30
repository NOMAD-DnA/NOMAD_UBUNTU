# NOMAD 실차 PC 운영 GUI

PC에서 전방 영상과 주행 그리드맵을 받아 표시하고, 지도 셀을 선택한 뒤 버튼을 눌러 목표점을 보낸다. 이 패키지는 차량 구동 노드를 실행하지 않는다.

## ROS 토픽 계약

| 방향 | 기본 토픽 | 타입 | 조건 |
|---|---|---|---|
| 수신 | `/oak/rgb/image_raw` | `sensor_msgs/msg/Image` | `rgb8`, `bgr8`, `mono8` 지원. SensorData best effort QoS |
| 수신 | `/nomad/navigation_costmap` | `nav_msgs/msg/OccupancyGrid` | `odom` 프레임, 회전 없는 origin, reliable + transient local QoS |
| 수신(선택) | `/nomad/localization/odometry` | `nav_msgs/msg/Odometry` | `odom` 프레임 차량 위치 표시 |
| 발행 | `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 버튼 클릭 시 1회, `odom` 프레임, 선택 셀 중심 XY, 방향 quaternion `w=1` |

지도값 `-1`(미관측) 또는 `80` 이상(차단)은 목표 선택을 막는다. 지도 클릭은 발행하지 않으며, 목표 토픽을 구독하는 노드가 연결된 뒤 발행 버튼을 따로 눌러야 한다. 발행 표시는 플래너의 목표 수락이나 차량 주행을 보장하지 않는다.

토픽 이름은 CLI 옵션 `--image`, `--grid`, `--odom`, `--goal`로 바꿀 수 있다. 기본값은 `nomad_bringup/config/topics.yaml`과 일치한다. PC와 Jetson은 동일한 `ROS_DOMAIN_ID`를 사용해야 한다.

## PC 빌드·실행

Ubuntu 24.04/ROS 2 Jazzy 기준. `python3-pyqt5`를 설치한 뒤 워크스페이스 루트에서:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select nomad_operator_gui --symlink-install
ROS_DOMAIN_ID=<Jetson과_같은_번호> ./scripts/run_real_operator_gui.sh
```

Jetson이 아직 데이터를 발행하지 않으면 화면에는 수신 대기 상태가 표시된다. 시험에는 실제 차량 명령 없이 영상/지도 토픽만 발행하면 된다.
