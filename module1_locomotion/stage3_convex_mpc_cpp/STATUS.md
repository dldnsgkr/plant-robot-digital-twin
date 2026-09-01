# stage3: C++ Convex MPC — 구현 상태 (도전 과제, 타임박스 종료)

## 달성 (실측 검증)

- **토크 제어 인프라**: URDF에 position+effort 이중 명령 인터페이스,
  launch 인자 `controller:=joint_group_effort_controller`로 전환.
- **SRBD Convex MPC (C++/Eigen/EiquadprogFast)**: Di Carlo 2018 방식 —
  13-상태 SRBD를 예측구간 N=10(0.4s)으로 응축, GRF 시퀀스를
  마찰원뿔(μ=0.6)·수직력(5~130N)·스윙 등식 제약 QP로 최적화.
  25Hz에서 1400회+ 연속 solve, 실패 0. 첫 컴파일 성공.
- **200Hz 토크 매핑**: 지지 τ = −JᵀRᵀf + 관절감쇠, 스윙 IK-PD.
  해석 Jacobian은 유한차분 대비 오차 2e-11로 검증.
- **순수 토크 제어 기립·유지**: 2단계 PD 기립(TUCK→STAND, 1s 연속 안정
  판정) 후 MPC가 z=0.33, 수평 자세를 25초+ 유지. GRF를 비대칭 분배
  (fz=[27,23,63,35] ≈ mg — CoM 오프셋 반영)하는 능동 균형 실증.
- 상태 공급 검증: odometry twist가 몸좌표임을 실측 확정.

## 한계 (정직 기록 — 심층 인터뷰 소재)

1. **기립 신뢰성의 초기조건 의존**: effort 활성화 순간 무토크 붕괴에서
   출발하므로, 붕괴 형태가 험하면(뒤집힘) PD 기립이 실패한다.
   파이썬 fall_recovery의 supine 펄스킥 로직 미이식 — 이식이 해법.
2. **수평 지속 외란(30~40N) 취약**: 붕괴가 시작되면 회복하지 못한다.
   원인 규명: 무너진 자세(z<0.2)에서는 발이 몸에 붙어 fz가 만드는 자세
   모멘트 팔이 커지고, 자세 가중(60) > 위치 가중(120·z만 큼) 구조상
   **QP가 '들어올리기'보다 'fz 최소화'를 최적으로 선택**한다 — 관측:
   붕괴 직후 fz=[5,5,5,5] 고착. 붕괴 감지 → PD 재기립 폴백을 추가했다.
   근본 해법: 자세/위치 가중의 상태 의존 스케줄링 또는 z 하한 소프트
   제약, 그리고 CoM 오프셋(베이스 원점 대비 +x 수 cm) 모델 반영.
3. **trot 보행 미검증**: 게이트 스케줄·스윙 PD 경로는 구현되어 있으나
   기립 강건성 확보가 선행 조건이라 검증하지 못함.

## 빌드·실행

```bash
# 컨테이너 안에서
cd /ws && colcon build --packages-select stage3_convex_mpc \
  --build-base /ws/src/plant_dt/.build --install-base /ws/src/plant_dt/.install
ros2 launch /ws/src/plant_dt/simulation/launch/plant_sim.launch.py \
  controller:=joint_group_effort_controller &
source /ws/src/plant_dt/.install/setup.bash
ros2 run stage3_convex_mpc convex_mpc_node   # mode:=trot 파라미터로 보행 시도
```

## stage2와의 연결

stage2 TUNING.md의 결론 — "위치제어 지지에서는 LIP(CoP) 모델이 성립하지
않아 토크 제어 + 접촉 스케줄 GRF 최적화가 필요하다" — 를 그대로 구현한
것이 본 stage3다. 기립·유지 실증으로 그 방향성이 옳았음을 보였고,
남은 한계는 가중 설계·기립 강건성이라는 다음 단계 과제로 좁혀졌다.
