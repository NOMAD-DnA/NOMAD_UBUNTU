# 실제 분할·깊이 2.5D 지도

```bash
cd ~/nomad_ws
./scripts/run_terrain_map.sh  # 맵 선택 → Gazebo·조종 창·높이/경사/클래스 지도
```

20cm 격자, NN입력·지도갱신10Hz목표(실제처리율은상태로확인).
사용자가 조종하며, 프로그램은 자동으로 운전하지 않는다. 조종창 W/S속도증감,A/D조향,Space정지.
지도창 툴바로 확대/이동, 셀클릭으로높이(m)·경사(도)·클래스이름확인. 회색은미관측/해당정보무효.

실제SegFormer10클래스 + 같은촬영시각의깊이/신뢰도 → pinhole점군 → 촬영시각TF → odom격자.
현재Gazebo RGB-D동일시야·정합된센서설정용. 실제미정합RGB/깊이를바로넣지않는다.
지면후보는Landscape(ID8), minconfidence0.5,깊이0.4–8m,stride4.
경사는 셀 중심 반경 0.3m의 원본 지면 점을 5cm XY 빈별 균등 가중한 TLS 평면으로 계산한다.
셀 높이를 이용한 경사 재계산은 비활성화. 점 수·공간 분포가 부족하면 미확정으로 두거나 기존에 검증된 원본 점 평면을 유지한다.
높이 표시값은 해당 셀 지면 점의 중앙값이며, 경사 계산의 입력이 아니다. ID8점만기존지면추정기에별도정책으로전달하며,실제모델ID는변경없이클래스지도에보존.
NN지면오분류시높이/경사도잘못될수있다. 신뢰도값은정확도보장이아님.

높이/경사/클래스는같은격자원점/크기/촬영stamp. 지도범위80m(최대160m),시작위치꼭짓점불필요.
관측10초클래스분포에서대표값갱신,과거셀값유지. 미관측높이는NaN,클래스-1.
출력 `/nomad/terrain/{grid,height,slope,classes,status}`. grid는관측여부와격자메타데이터이며주행비용아님.
GT는위치·자세TF용으로만사용. 의미라벨GT/세계지면정답을지도입력으로사용하지않는다.
최종비용결합·Planning/Control·기본인지에는연결하지않음.

종료Ctrl+C. `sessions/live_terrain/실행시간/map.npz`에배열/원점/해상도,`status.json`에처리상태저장.
실행설정: launch `resolution:=0.2`, `semantic_script:=...`, `viewer:=false`, `teleop:=false`, `headless:=true`, `world_file:=...`.
분할모델은기존 `~/ws/semantic_ws` 설치가필요. 단일같은ROS환경의노트북시험, Xavier/보드통신검증아님.
