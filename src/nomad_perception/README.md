# 인지·지도

```bash
./scripts/run_module.sh semantic  # 별도 터미널: RGB 분할
./scripts/run_module.sh map       # 깊이·분할·융합 pose → 지도·코스트맵
```

핵심은 `terrain_layers.py`(지면·클래스), `navigation_costmap.py`(비용·관측기억), `navigation_terrain.py`(ROS IO).
설정: `config/navigation_terrain.yaml`. 입력 RGB/깊이/라벨은 같은 capture·TF로 정렬한다.
출력 `/nomad/navigation_costmap`, `/nomad/navigation_local_costmap`, `/nomad/perception/status`.
20cm셀·120cm 이차 장애물 비용. 미관측은 -1, 통행0–79, 차단100.
세부 계약·기존 비교 도구는 [기존 문서](docs/archive/README_details.md).
