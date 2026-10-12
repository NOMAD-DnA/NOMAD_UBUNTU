# 실제 인지·융합 자율주행

```bash
./run_terrain_navigation.sh viewer:=false  # 맵·GPP/LPP 선택, B0+VIO/GNSS+지도+제어+RViz
./run_teleop.sh                            # 초기정렬 이동 후 정지·종료
```

위치추정·인지 READY와3초 정지가 확인되면 Planning/Control이 시작된다. RViz에서 목표 지정.
각 파트만 켜는 명령은 루트 `실행안내.md`의 `run_module.sh`를 따른다.

- 위치: Basalt/OpenVINS + GNSS → Ceres → odom/base_link. GT 동적TF off.
- 인지: B0 label·confidence와 동일 capture 깊이 → 촬영시각 TF → 원본 지면점 TLS·20cm 지도.
- 비용: 의미30%·경사제곱70%,0–79. 확인차단100,미관측-1.
- 장애물주변:1.2m 반경,추가비용100*(1-(d/1.2)^2),기존값과max. unknown은열지않는다.
- 초기전방3m 고정 출발가정/실관측전역기억·지역30초. `config/navigation_terrain.yaml`에서조절.
- 플래닝 override: `config/terrain_planning.yaml`. 원본 Planning/Control의 전략·인터페이스는 유지.

전체 실행 중 단독 노드를 중복 실행하지 않는다. 현재 막다른 길 후진복구 이력 부족과 지도지연은 미해결.
과거시연·시험조건은 `docs/archive/NAVIGATION_TERRAIN_before_refactor.md`에 보존하며현재설정으로해석하지않는다.
