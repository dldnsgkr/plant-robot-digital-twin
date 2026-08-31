# 플랜트 4족 보행 로봇 Digital Twin

가상 발전소 환경(복도·공장·계단·험지)에서 4족 보행 로봇이 자율 주행하며
아날로그 계기 판독, 열화상 점검, 가스 누출원 추적을 수행하는 Digital Twin 시스템.

- 과제 요구사항: [docs/REQUIREMENTS.md](docs/REQUIREMENTS.md)
- 프로젝트 계획: [docs/PLAN.md](docs/PLAN.md)

## 기술 스택

| 영역 | 스택 |
|---|---|
| Robot OS | ROS 2 Humble + Nav2 |
| 시뮬레이터 | Gazebo Classic 11 (Docker, GPU 불필요) |
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

동작 확인:

```bash
# 컨테이너 셸 안에서 — Gazebo 서버(headless) 실행
gzserver --verbose /usr/share/gazebo-11/worlds/empty.world &
ros2 topic list                        # /clock 등이 보이면 정상

# Foxglove 연결용 브리지
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
# → Mac의 Foxglove Studio에서 ws://localhost:8765 접속
```

Gazebo GUI가 필요하면 브라우저에서 http://localhost:8080/vnc.html 접속 후
컨테이너 셸에서 `gzclient` 실행.

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
| Phase 0 환경 구축 | 진행 중 | 위 "환경 설정" 참조 |
| Phase 1 Virtual Plant | 예정 | - |
| Module 1 Locomotion | 예정 | - |
| Module 2 Inspection AI | 예정 | - |
| Module 3 Gas/Safety | 예정 | - |
| 통합 시나리오 | 예정 | - |
