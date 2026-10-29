# 공통 차량 운동학

`nomad_bringup/config/vehicle.yaml`이 현재 차량 설정이다. `drive_layout=front_drive_rear_steer`는 앞축 속도 m/s, 등가 조향 rad(전진 좌회전 양수)를 사용한다. 실제 뒷바퀴 각도는 반대 부호다. base_link는 차체 기준이며 적분 기준인 비조향 앞축은 +wheelbase/2에 있다. 따라서 reference_offset=-wheelbase/2다.

`advance`는 비조향축을 정확한 원호로 적분하고 같은 차체 기준으로 되돌린다. `Geometry`는 실제 좌우 관절각과 등가 조향을 상호 변환한다. 구동·조향 설정, 음수 offset, 물리 안쪽바퀴 한계를 함께 검증한다. 임시 블록 기하이며 실차 캘리브레이션이 아니다. 기존 전륜 조향은 vehicle_legacy.yaml로 명시한다.
