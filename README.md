# 플랜트 4족 보행 로봇 Digital Twin

가상 발전소 환경(복도·공장·계단·험지)에서 4족 보행 로봇이 자율 주행하며
아날로그 계기 판독, 열화상 점검, 가스 누출원 추적을 수행하는 Digital Twin 시스템.

- 과제 요구사항: [docs/REQUIREMENTS.md](docs/REQUIREMENTS.md)
- 프로젝트 계획: [docs/PLAN.md](docs/PLAN.md)

## 기술 스택

| 영역 | 스택 |
|---|---|
| Robot OS | ROS 2 Jazzy + Nav2 |
| 시뮬레이터 | Gazebo Harmonic (Docker arm64 네이티브, GPU 불필요) |
| 언어 | Python (주력), C++ (MPC stage3) |
| AI/비전 | PyTorch, OpenCV |
| 시각화 | Foxglove Studio, noVNC, 웹 대시보드(rosbridge) |
| 실행 환경 | macOS + Docker (보너스 RL/클라우드 관제만 EC2 스팟) |

## 환경 설정 (Phase 0)

요구 사항: Docker Desktop, (권장) [Foxglove Studio](https://foxglove.dev/download) Mac 앱

```bash
cd docker
docker compose build          # 최초 1회, 10~20분 소요
docker compose up -d          # 컨테이너 기동
docker compose exec sim bash  # 개발 셸 진입
```

동작 확인 (자동 스모크 테스트):

```bash
docker compose run --rm --no-deps sim bash /ws/src/plant_dt/docker/smoke_test.sh
```

수동 확인:

```bash
# 컨테이너 셸 안에서 — Gazebo Harmonic 서버(headless) 실행
gz sim -s -r empty.sdf &
gz topic -l                            # Gazebo 토픽이 보이면 정상

# Foxglove 연결용 브리지
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
# → Mac의 Foxglove Studio에서 ws://localhost:8765 접속
```

Gazebo GUI가 필요하면 브라우저에서 http://localhost:8080/vnc.html 접속 후
컨테이너 셸에서 `gz sim -g` 실행.

## 레포지토리 구조

```
module1_locomotion/   # 험지 돌파 제어 (Elevation Map, MPC 3단계, Fall Recovery)
module2_inspection/   # 게이지 독해, 열화상 퓨전, 순찰 경로
module3_gas_safety/   # 가스 확산 시뮬, Source Seeking, 자율 복귀
simulation/           # Gazebo 월드·모델·launch (복도 45×3.5×4m, 공장 25×18×7m)
dashboard/            # Digital Twin 관제 대시보드
bonus/                # RL 보행, 다중 로봇, 클라우드 관제
docker/               # 개발 컨테이너 구성
docs/                 # 계획서·요구사항·보고서
```

## 모듈별 실행

> 각 Phase 완료 시 이 섹션에 실행 명령을 추가한다.

| 모듈 | 상태 | 실행 |
|---|---|---|
| Phase 0 환경 구축 | 완료 | 위 "환경 설정" 참조, 검증: `docker/smoke_test.sh` |
| Phase 1 Virtual Plant | 완료 | `ros2 launch /ws/src/plant_dt/simulation/launch/plant_sim.launch.py` (컨테이너 안) → Foxglove로 ws://localhost:8765 접속 |
| Module 1 Locomotion (stage1) | 완료 | 컨테이너에서 launch 후 `python3 module1_locomotion/stage1_gait_controller/gait_controller.py`, `.../terrain_mapping/elevation_map.py`, `.../fall_recovery/fall_recovery.py` 실행, `/cmd_vel`로 조종 |
| Module 1 MPC stage2 (LIP-MPC) | 완료 | launch 후 `python3 module1_locomotion/stage2_simple_mpc_py/mpc_node.py` — 게이트와 병행 실행, 튜닝 기록은 stage2_simple_mpc_py/TUNING.md |
| Module 1 MPC stage3 (C++ Convex) | 예정 | - |
| Module 2 Inspection AI | 완료 | launch 후 gauge_reader/plant_process/image_degrader, thermal_camera_sim/thermal_fusion 실행. 정확도: `eval_accuracy.py`, 순찰 경로: `patrol_planner.py` |
| Module 3 Gas/Safety | 완료 | 통합 launch에 포함, 상태: module3_gas_safety/STATUS.md |
| Phase 5 통합+관제 | 완료 | `ros2 launch .../plant_dt.launch.py mission:=true` + 브라우저로 dashboard/index.html (rosbridge :9090) |
