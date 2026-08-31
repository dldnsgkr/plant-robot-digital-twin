# 플랜트 4족 보행 로봇 Digital Twin — 프로젝트 계획서

> 작성일: 2026-08-31
> 목표 수행 시간: 필수 200h + 보너스 약 60h
> 최종 결과물: 미션 요구사항을 모두 충족하는 GitHub Repository

---

## 1. 프로젝트 개요

가상의 발전소 환경(복도·공장·계단·험지·고온 구역)에서 4족 보행 로봇을 자율 주행시키고,
탑재 센서로 아날로그 계기 판독·열화상 점검·가스 누출원 추적을 수행하는
**Digital Twin 기반 로봇 관제 시스템**을 구축한다.

시스템 데이터 흐름:

```
Virtual Plant → Robot Twin → Simulation Sensor Data → Inspection AI → Monitoring System
```

| 구성 요소 | 역할 | 구현 위치 |
|---|---|---|
| Virtual Plant | 발전소 3D 환경 (복도 45×3.5×4m, 공장 25×18×7m) | `simulation/` |
| Robot Twin | 4족 로봇 URDF + 센서 모델 + 보행 제어 | `module1_locomotion/` |
| Inspection Data | 게이지 수치, 열화상, 가스 농도 | `module2_inspection/`, `module3_gas_safety/` |
| Monitoring System | 로봇 위치·설비 상태·점검 결과 시각화 | `dashboard/` (보너스 ①과 통합) |

---

## 2. 핵심 의사결정: 실행 환경 및 스택 선택

로컬 개발 머신이 **macOS**이므로 ROS2 + 시뮬레이터를 네이티브로 돌릴 수 없다.
이 선택이 이후 모든 과제물의 방향을 결정하므로 가장 먼저 확정해야 한다.

### 2.1 환경 시나리오 비교

| | A. Docker only (Mac) | B. 하이브리드 ★추천 | C. EC2 전용 서버 |
|---|---|---|---|
| 구성 | Docker로 ROS2 Humble + Gazebo, GUI는 웹(noVNC/Foxglove) | 평소 Docker+Gazebo(무료), RL 학습·클라우드 관제 때만 EC2 GPU 스팟 | g5.xlarge(A10G)에 Isaac Sim 상주, NICE DCV 원격 데스크톱 |
| 시뮬레이터 | Gazebo | Gazebo 메인 (+EC2에서 RL 학습) | Isaac Sim |
| 예상 비용 | 0원 | 약 3~10만원 (스팟 ~$0.3-0.5/h × 30-60h + 스토리지) | 약 30만원+ (온디맨드 ~$1/h × 200h + 100GB EBS) |
| 필수 요건 충족 | 전부 가능 | 전부 가능 | 전부 가능 |
| 보너스 4종 | RL은 CPU 학습(매우 느림), 클라우드 관제는 모킹 | 4종 전부 정상 수행 가능 | 4종 전부 가능, 비용 최대 |
| 리스크 | RL 품질 저하, 렌더링 품질 한계 | EC2 사용 구간 관리 필요 | 인스턴스 끄는 것 잊으면 요금 폭탄, 원격 GUI 지연 |

**결정 규칙(합의됨): Docker로 가면 Gazebo, 아니면 Isaac Sim.**
→ B안은 Gazebo 메인이며, 필수 200시간 전체를 무료 구간(Docker)에서 소화한다.

### 2.2 선택에 따른 과제물 방향성 분기 (중요)

같은 요구사항이라도 환경 선택에 따라 **구현 방식과 결과물의 형태가 달라진다.**
아래 표는 각 산출물이 어떻게 갈라지는지 정리한 것이다.

| 산출물 | Gazebo 경로 (A/B) | Isaac Sim 경로 (C) |
|---|---|---|
| **3D 맵 포맷** | SDF/World 파일 + Mesh(DAE/STL). 제공 에셋(PBL_AssetPackage)을 Blender에서 변환 | USD(OpenUSD) 씬. 에셋을 USD로 컨버팅, RTX 렌더링 활용 |
| **로봇 모델** | URDF + `gz_ros2_control` 플러그인 (과제 필수 산출물이 URDF이므로 그대로 제출 가능) | URDF를 Isaac이 USD로 임포트. **제출용 URDF는 별도 유지 필요** |
| **Elevation Map (M1)** | Depth 카메라/LiDAR → `elevation_mapping_cupy`(CPU 모드) 또는 자체 그리드맵 노드 | Isaac 내장 LiDAR(RTX Lidar) → 동일 파이프라인, GPU 가속 가능 |
| **MPC 보행 제어 (M1)** | CHAMP 스택 위에 컨트롤러 교체 방식. Gazebo 물리(ODE/DART)에 맞춰 튜닝 | Isaac PhysX에 맞춰 튜닝. 접촉 파라미터가 달라 **게인 값 이식 불가, 재튜닝 필요** |
| **열화상 카메라 (M2)** | Gazebo에 열화상 센서가 없음 → **커스텀 플러그인으로 합성**: 발열 오브젝트에 온도 태그를 달고 카메라 시점에서 온도맵 렌더링 | Isaac Sim도 순정 열화상은 없으나 semantic/emissive 렌더링으로 더 사실적인 합성 가능 |
| **저조도·흔들림 (M2)** | 조명 엔티티 제어 + 이미지에 노이즈/블러 후처리 주입으로 재현 | RTX 실시간 조명으로 자연스럽게 재현 (별도 후처리 불필요) |
| **가스 확산 (M3)** | 시뮬레이터 무관하게 **자체 ROS2 노드로 가우시안 플룸 모델** 구현 → 로봇 위치 기반 농도 퍼블리시. RViz Marker로 농도장 시각화 | 동일 노드 재사용 가능. Isaac에서 파티클로 시각 효과 추가 가능 |
| **RL 보행 (보너스 ③)** | A안: CPU에서 MuJoCo/Genesis로 소규모 학습 (품질 한계) / B안: EC2 GPU에서 Isaac Lab 또는 Genesis 학습 → **ONNX로 정책 내보내 Gazebo에 이식(Sim2Sim)** | Isaac Lab 네이티브 학습 → 같은 물리 엔진이라 이식 마찰 최소 |
| **클라우드 관제 (보너스 ④)** | A안: 로컬 docker-compose로 클라우드 구조만 모킹 / B안: EC2에 rosbridge+대시보드 배포, 로컬 시뮬과 WebSocket 연결 (진짜 원격 관제) | 시뮬 자체가 클라우드에 있으므로 관제 분리 구성이 자연스러움 |
| **REPORT.md 서술** | "무료·재현 가능한 환경 구축" + Sim2Sim 이식을 Sim2Real 논의로 연결 | "고정밀 물리·렌더링 검증" 중심 서술 |

**핵심 함의:**

- **Gazebo 경로의 추가 개발량**: 열화상 합성 플러그인, 저조도/흔들림 후처리 노드를 직접 만들어야 한다 (약 +10h). 대신 비용 0원, 재현성(누구나 Docker로 실행) 확보 — README 실행 가이드 품질이 올라간다.
- **Isaac 경로의 추가 부담**: USD 변환 파이프라인, 원격 GUI, 비용 관리. 대신 Module 2의 시각 품질과 RL 연계가 좋다.
- **B안(하이브리드)은 두 경로의 장점을 취한다**: 필수 요건은 Gazebo에서 완성하고, RL 학습·클라우드 관제만 EC2를 쓴다. RL 정책을 Gazebo로 이식하는 과정 자체가 심층 인터뷰의 **Sim2Real 질문에 대한 실전 답변**이 된다 ("물리 엔진 간 차이를 도메인 랜덤라이제이션으로 극복" 서술 가능).

### 2.3 확정 스택 (B안 기준)

| 영역 | 선택 | 근거 |
|---|---|---|
| OS/실행 | macOS + Docker (Ubuntu 24.04 컨테이너, arm64 네이티브), 보너스 구간만 EC2 g5.xlarge 스팟 | 비용 최소화, 재현성 |
| ROS | ROS 2 Jazzy + Nav2 | Phase 0 검증 결과: Humble은 arm64에 Gazebo 스택 미제공 (아래 결정 기록) |
| 시뮬레이터 | Gazebo Harmonic (gz-sim8) | arm64 네이티브 공식 지원, Jazzy 표준 페어링 |
| 로봇 모델 | Unitree Go2 URDF (오픈소스) + gz_ros2_control 기반 보행 스택 (CHAMP는 Jazzy 포팅 검증 후 채택) | 제출물 URDF 요건 직결 |
| 언어 | Python 주력, stage3 MPC만 C++ | 과제 제약사항 |
| AI/비전 | PyTorch, OpenCV, (RL: Isaac Lab 또는 Genesis on EC2) | 과제 지정 |
| GUI 확인 | Foxglove Studio(Mac 네이티브, WebSocket 접속) + noVNC(Gazebo GUI 필요 시) | Mac에서 X11 포워딩보다 안정적 |
| 대시보드 | rosbridge_suite + React(또는 Foxglove 커스텀 패널) + WebSocket | 보너스 ①과 필수 Monitoring System 통합 |

> **Phase 0 결정 기록 (2026-08-31)**: 당초 Humble + Gazebo Classic으로 계획했으나 실측 결과
> ① `osrf/ros:humble-desktop`은 amd64 전용이라 Apple Silicon에서 에뮬레이션 실행(빈 월드 RTF 0.5),
> ② Humble arm64 저장소에는 Gazebo Classic도 `ros-gz-sim`도 빌드가 없음.
> 반면 **Jazzy + Harmonic은 arm64에서 전체 스택(ros-gz-sim, gz-ros2-control, Nav2, Foxglove, rosbridge) 제공** → 채택.
> 파급: CHAMP 등 Humble 기준 오픈소스는 Jazzy 포팅 검증이 필요 (Phase 2 리스크 표 참조).

> **분기 지점 기록**: 이후 어떤 Phase에서든 EC2 상시 사용(C안)으로 전환하고 싶어지면,
> §2.2 표의 "Isaac Sim 경로" 열을 따라 산출물 방향을 바꾸면 된다.
> 전환 비용이 가장 큰 것은 맵(SDF→USD 재작업)과 보행 튜닝(재튜닝)이다.
> **Phase 1 완료 이후의 전환은 비추천.**

---

## 3. 레포지토리 구조 (최종 제출 형태)

```
plant-robot-digital-twin/
├── module1_locomotion/
│   ├── stage1_champ_tuning/     # 1번 과정: CHAMP 기반 + 지형적응·게이트전환 튜닝 (완주 보장선)
│   ├── stage2_simple_mpc_py/    # 2번 과정: SRBD 모델 간이 MPC를 Python으로 직접 구현
│   ├── stage3_convex_mpc_cpp/   # 3번 과정: Convex MPC(MIT Cheetah 방식) C++ 구현 (도전)
│   ├── terrain_mapping/         # Elevation Map 생성 + 발 디딤 위치 결정
│   └── fall_recovery/           # 넘어짐 감지 + 자동 기립 시퀀스
├── module2_inspection/
│   ├── gauge_reader/            # 아날로그 게이지 검출·바늘 각도→수치 변환 (오차 5% 이내)
│   ├── thermal_fusion/          # 열화상+RGB 정합 및 과열 시각화
│   └── patrol_planner/          # Viewpoint 기반 최적 순찰 경로 생성
├── module3_gas_safety/
│   ├── gas_simulation/          # 가우시안 플룸 확산 + 바람 효과 노드
│   ├── source_seeking/          # 경사 하강 기반 누출원 추적
│   └── return_to_home/          # 배터리/통신 조건 트리거 자율 복귀
├── simulation/
│   ├── worlds/                  # 복도(45×3.5×4m)·공장(25×18×7m) SDF, 장애물(복도3+공장5)
│   ├── models/                  # 로봇 URDF, 공장기계, 가스탱크, 팔레트 등
│   └── launch/                  # 통합 실행 launch 파일
├── dashboard/                   # Monitoring System + 보너스① 웹 대시보드
├── bonus/
│   ├── rl_locomotion/           # 보너스③ PPO 학습 (EC2) + ONNX 정책 이식
│   ├── multi_robot/             # 보너스② 2대 협업 순찰
│   └── cloud_control/           # 보너스④ EC2 원격 관제 배포 구성
├── docker/                      # Dockerfile, docker-compose.yml (로컬/EC2 겸용)
├── docs/
│   ├── PLAN.md                  # (본 문서)
│   └── REPORT.md                # 프로젝트 보고서
└── README.md                    # 환경 설정·실행 가이드
```

MPC "1번→2번→3번 과정"은 순차 진행하되, **stage1 완성 시점에 과제 필수 요건은 이미 충족**되도록 설계한다.
stage2/3는 실패해도 제출물에 영향이 없고, 성공하면 보고서·심층 인터뷰의 강력한 소재가 된다.

---

## 4. 단계별 로드맵

### Phase 0 — 개발 환경 구축 (15h)
- Docker 이미지 작성: ROS2 Humble + Gazebo + Nav2 + CHAMP 의존성 (linux/arm64 확인)
- Foxglove/noVNC 접속 확인, 에셋 패키지 다운로드(`PBL_AssetPackage.zip`) 및 포맷 확인
- **완료 기준**: Mac에서 `docker compose up` 한 번으로 빈 Gazebo 월드 + RViz 대체 뷰가 뜬다

### Phase 1 — Virtual Plant 구축 (25h)
- 복도(45×3.5m, 높이 4m)·공장(25×18m, 높이 7m) 월드 모델링, 제공 에셋 배치
- 험지 장애물: 복도 3개(파이프 잔해 등), 공장 5개 + 공장기계 1개 + 계단 구간
- Unitree Go2 URDF 스폰 + 센서(LiDAR/Depth/RGB) 장착
- **완료 기준**: 최소 모델링 조건 전부 충족, 로봇이 월드에 스폰되어 센서 토픽 발행

### Phase 2 — Module 1: Locomotion (55h)
- Terrain Mapping: Depth/LiDAR → Elevation Map(노이즈 필터링 포함) → 발 디딤 위치 결정 (12h)
- stage1: CHAMP 보행 + 험지 파라미터 튜닝 + Trot/Crawl 게이트 전환 로직 (15h)
- stage2: Python 간이 MPC (SRBD, 예측 구간 튜닝 실험 기록) (12h)
- stage3: C++ Convex MPC (도전, 타임박스 엄수) (10h)
- Fall Recovery: IMU 기반 넘어짐 판정 + 관절 시퀀스 기립 (6h)
- **완료 기준**: 복도 험지·계단 통과 성공률 측정 및 기록, 넘어짐 시 자동 기립

### Phase 3 — Module 2: Inspection AI (35h)
- 열화상 합성 플러그인 + 저조도/모션블러 주입 노드 (Gazebo 경로 추가분) (8h)
- 게이지 독해: 검출(Hough/경량 CNN) → 바늘 각도 → 수치 변환, 오차 5% 검증 스크립트 (15h)
- 열화상-RGB 정합(호모그래피/외부 파라미터 캘리브레이션) + 과열 오버레이 (7h)
- Viewpoint 기반 순찰 경로 최적화 (인식 비용함수 + TSP/A*) (5h)
- **완료 기준**: 이동 중 게이지 판독 오차 ≤5%, 과열 배관 검출 데모

### Phase 4 — Module 3: Gas/Safety (30h)
- 가우시안 플룸 확산 노드(바람 벡터 파라미터) + 농도장 시각화 (10h)
- Source Seeking: 농도 구배 추정 + 경사 상승 이동, 국소 최적 탈출 전략(랜덤 재탐색/나선 탐색) (12h)
- Return to Home: 배터리(<20%)/통신 음영 트리거 → Nav2 최단 경로 복귀 (8h)
- **완료 기준**: 임의 누출원 위치에서 추적 성공, 비상 트리거 시 충전소 복귀

### Phase 5 — 통합 + Monitoring System + 보너스① 대시보드 (30h)
- ROS2 통신 구조 설계 문서화: Topic/Service/Action 배분, QoS 설정 (동료평가 통합 질문 대비)
- rosbridge + 웹 대시보드: 로봇 위치, 게이지 수치, 과열 상태, 가스 농도맵 실시간 표시
- 3개 모듈 통합 launch 스크립트 + 통합 시나리오 데모 (진입→점검→가스추적→복귀)
- **완료 기준**: 스크립트 하나로 전체 시나리오 실행, 대시보드에 실시간 반영

### Phase 6 — 보너스 ②③④ (60h)
- ③ RL 보행: EC2 g5.xlarge 스팟 기동 → Isaac Lab/Genesis PPO 학습 → ONNX 정책을 Gazebo로 Sim2Sim 이식, MPC와 성능 비교 (30h, **EC2 사용 구간**)
- ② 다중 로봇: 네임스페이스 분리 2대 스폰 + 구역 분할 협업 순찰 (15h)
- ④ 클라우드 관제: 대시보드+rosbridge를 EC2에 배포, 로컬 시뮬 ↔ 클라우드 WebSocket 연동 (15h, **EC2 사용 구간**)
- **비용 통제 규칙**: EC2는 작업 시작 시 기동·종료 시 중지 스크립트로만 조작, 스팟 + 예산 알람($30) 설정

### Phase 7 — 문서화 (15h)
- REPORT.md: 모듈별 알고리즘 흐름도, 문제 해결 기록, 평가 지표(넘어짐 횟수, 인식 오차율, 추적 성공률) 그래프
- README.md: Docker 원커맨드 실행 가이드, 모듈별 실행 예시
- 동료평가·심층 인터뷰 질문 목록에 대한 답변 초안 정리 (docs/에 별도 파일)

---

## 5. 리스크와 대응

| 리스크 | 영향 | 대응 |
|---|---|---|
| CHAMP 등 4족 오픈소스 스택의 Jazzy 미지원 | Phase 2 stage1 계획 변경 | 커뮤니티 포크 검증 → 실패 시 gz_ros2_control + 자체 게이트 생성기(보행 궤적은 stage2 MPC 코드와 공유)로 대체 |
| Apple Silicon에서 Gazebo 컨테이너 성능 저하 | 시뮬 실시간 계수 하락 | 물리 스텝 조정, headless 실행 + Foxglove, 필요 시 Rosetta/이미지 교체. 최악의 경우 EC2 CPU 인스턴스로 시뮬 이전(§2.2 분기표 참조) |
| stage3 C++ MPC 미완성 | 없음 (stage1이 요건 충족) | 타임박스 10h 초과 시 중단하고 보고서에 시도·한계 기술 |
| 제공 에셋 포맷 비호환 | Phase 1 지연 | Blender 변환 파이프라인 우선 검증, 불가 시 자체 모델링(과제 허용) |
| RL 정책의 Gazebo 이식 실패 | 보너스③ 품질 | 도메인 랜덤라이제이션 강화, 실패 시 학습 곡선·Isaac 내 결과만으로 보고 |
| EC2 비용 초과 | 예산 | 스팟 + 자동 중지 스크립트 + $30 예산 알람 |

---

## 6. 진행 순서 요약

```
Phase 0 → 1 → 2 → 3 → 4 → 5 → (6 보너스) → 7
                 └ 각 Phase 완료 기준 통과 후 다음 진행
```

다음 액션: **Phase 0 착수** — `docker/` 디렉토리에 Dockerfile + docker-compose.yml 작성,
에셋 패키지 다운로드 및 검증.
