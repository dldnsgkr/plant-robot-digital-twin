# Digital Twin 시스템 아키텍처 — 통신 구조 설계

## 1. 구성 요소 매핑 (과제 §1-2 Digital Twin 4요소)

| 요소 | 구현 | 데이터 |
|---|---|---|
| Virtual Plant | `plant_world.sdf` (Gazebo Harmonic) + `gas_field.py`(확산 물리) + `plant_process.py`(설비 프로세스) | 물리·렌더링·플룸 농도장·게이지 압력 |
| Robot Twin | Go2 URDF + `gz_ros2_control` + 게이트/복구/지형 노드 | 관절·IMU·LiDAR·카메라·odometry |
| Inspection Data | `gauge_reader`, `thermal_fusion`, `gas/concentration` | 판독값·과열 판정·농도 |
| Monitoring System | `dashboard/index.html` (rosbridge) + Foxglove | 위치·상태·점검 결과·농도장 시각화 |

## 2. 토픽 그래프 (주요 데이터 흐름)

```
[Gazebo]
  ├─ (ros_gz_bridge) → /lidar/points /imu /front_camera /depth_camera /clock
  ├─ (OdometryPublisher) → /model/go2/odometry
  └─ (gz_ros2_control) ← /joint_group_position_controller/commands

Module1: /lidar/points → elevation_map → /elevation_map/raw → gait_controller
         /imu → fall_recovery → /gait_enable → gait_controller
         /cmd_vel → gait_controller → 관절 명령
Module2: /front_camera → gauge_reader → /inspection/gauge_value·overlay
         /depth_camera+odometry → thermal_camera_sim → /thermal/image
         /thermal/image+/front_camera → thermal_fusion → /inspection/thermal_fused·max_temp
Module3: odometry → gas_field → /gas/concentration·field·wind
         /gas/* → source_seeker → /cmd_vel, /gas/source_estimate·found
         odometry → rth → /robot/battery, /rth_active, /cmd_vel

통합:    mission_controller → /mission/state·event, /gas/alarm, /cmd_vel
관제:    rosbridge(9090) → 웹 대시보드 / foxglove_bridge(8765) → Foxglove
```

## 3. 설계 결정 (동료평가·심층 인터뷰 답변)

**Topic/Service/Action 배분**
- 연속 스트림(센서·명령·상태)은 전부 **Topic**: 결합도 최소, 구독자 자유 확장
  (같은 odometry를 MPC·열화상·가스·RTH·대시보드 5곳이 독립 구독).
- **Service**는 시뮬레이터 제어(스폰·set_pose)처럼 1회성 요청-응답에만 사용
  (gz 서비스). 로봇 운용 경로에는 블로킹 호출을 두지 않음.
- 장시간 작업(미션 단계)은 Action 대신 상태 토픽+이벤트 토픽으로 구현 —
  단일 로봇·단일 조정자 구조에서는 Action 서버의 복잡성 대비 이득이 없다고
  판단. 다중 로봇 확장 시(보너스②) Nav2 Action 인터페이스로 전환 지점 명시.

**cmd_vel 중재 (자원 충돌 방지)**
- 우선순위: fall_recovery(관절 직접 점유) > rth(/rth_active) > source_seeker
  (알람 게이트) > mission_controller. 각 상위 노드가 활성 신호를 발행하면
  하위가 스스로 침묵하는 **협조적 중재** — 중앙 mux 없이 결합도 낮게 유지.

**병목·부하 관리**
- 이미지류(1280×720 RGB, depth)는 시뮬 내부 노드만 원본 구독. 관제로는
  결과값(판독치·온도·농도)만 전달 — 대시보드 대역폭을 수 KB/s로 유지.
- 대시보드 rosbridge 구독에 throttle_rate(200~1000ms) 적용, 농도장은
  0.5m 격자 1Hz로 다운샘플.
- 게이트 200Hz 제어 루프와 10~15Hz 인지 루프를 노드 분리로 격리 —
  비전 처리 지연이 보행 명령 주기에 영향을 주지 않음.

**동기화**
- 전 노드 `use_sim_time` + /clock 브리지로 시뮬 시간 일원화.
- 열화상 정합은 타임스탬프 무관하게 depth 프레임 기하를 기준으로 계산
  (같은 센서 헤드에서 파생) — 카메라 간 시간차 정합 오차 원천 제거.

## 4. Sim2Real 관점 (심층 인터뷰)

- 시뮬 전용 요소를 인터페이스 뒤로 격리: odometry(→실기 상태추정기로 교체),
  gas_field(→실측 가스센서), plant_process(→실제 설비), OdometryPublisher/
  set_pose(시뮬만). 나머지 노드는 토픽 계약이 동일해 그대로 이식 가능.
- 게이트 파라미터(마찰·접촉)는 물리 엔진 의존 — stage2 TUNING.md에 기록된
  플랜트-모델 불일치 실험이 Sim2Real 격차의 실증 사례.
