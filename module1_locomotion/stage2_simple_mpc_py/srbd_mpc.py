#!/usr/bin/env python3
"""stage2: SRBD/LIP 기반 간이 MPC 코어.

모델 (축별 분리 LIP — Linear Inverted Pendulum):
    상태 x = [p_err, v_err]  (기준 궤적 대비 CoM 위치/속도 오차)
    입력 u = CoP 오프셋 (지지발 배치로 실현)
    동역학:  v̇ = ω²(p - u),  ω² = g/h   (h: CoM 높이)

MPC (condensed QP, 무제약 해 + 입력 포화):
    X = Sx·x0 + Su·U
    J = Xᵀ Q̄ X + Uᵀ R̄ U   →   U* = -(SuᵀQ̄Su + R̄)⁻¹ SuᵀQ̄Sx x0
    |u| ≤ u_max 는 해에 대한 클리핑으로 처리 (간이 구현 — 보고서에 한계 명시)

예측 구간 5초(요구사항 '다음 5초간 예측') 기준, dt=0.1 → N=50.
튜닝 실험: python3 srbd_mpc.py  →  N 스윕 결과 표 출력 (docs 기록용)
"""
import time

import numpy as np

G = 9.81


class LipMpc:
    def __init__(self, com_h=0.32, horizon_n=50, dt=0.1,
                 q_p=40.0, q_v=8.0, r_u=1.0, u_max=0.10):
        self.dt = dt
        self.n = horizon_n
        self.u_max = u_max
        w2 = G / com_h
        # 이산화 (오일러): x+ = A x + B u
        a = np.array([[1.0, dt], [w2 * dt, 1.0]])
        b = np.array([[0.0], [-w2 * dt]])

        # condensed 행렬: X = Sx x0 + Su U
        nx, nu = 2, 1
        sx = np.zeros((nx * self.n, nx))
        su = np.zeros((nx * self.n, nu * self.n))
        ak = np.eye(nx)
        for i in range(self.n):
            ak = ak @ a
            sx[i * nx:(i + 1) * nx, :] = ak
            for j in range(i + 1):
                ajb = np.linalg.matrix_power(a, i - j) @ b
                su[i * nx:(i + 1) * nx, j:j + 1] = ajb

        qbar = np.kron(np.eye(self.n), np.diag([q_p, q_v]))
        rbar = np.eye(self.n) * r_u
        # 되먹임 이득 (U* = -K x0) 을 미리 계산 — 실시간 해석해
        self.k_gain = np.linalg.solve(su.T @ qbar @ su + rbar,
                                      su.T @ qbar @ sx)

    def solve(self, p_err, v_err):
        """최적 CoP 오프셋 시퀀스의 첫 입력을 반환 (receding horizon)."""
        u_seq = -self.k_gain @ np.array([p_err, v_err])
        return float(np.clip(u_seq[0], -self.u_max, self.u_max))


def _simulate_push(mpc, v0, sim_t=6.0, sim_dt=0.02, delay_steps=2,
                   com_h=0.32, noise=0.005):
    """외란(초기 속도 v0) 후 폐루프 응답. 입력 지연 + 측정 노이즈 포함."""
    w2 = G / com_h
    p, v = 0.0, v0
    u_hist = [0.0] * delay_steps
    max_p, settled_at = 0.0, None
    rng = np.random.default_rng(0)
    for i in range(int(sim_t / sim_dt)):
        u = mpc.solve(p + rng.normal(0, noise), v + rng.normal(0, noise))
        u_hist.append(u)
        u_apply = u_hist.pop(0)          # 액추에이션 지연
        v += w2 * (p - u_apply) * sim_dt
        p += v * sim_dt
        max_p = max(max_p, abs(p))
        if settled_at is None and abs(p) < 0.01 and abs(v) < 0.02 \
                and i * sim_dt > 0.3:
            settled_at = i * sim_dt
        if abs(p) > 0.5:                 # 전도 간주
            return max_p, None
    return max_p, settled_at


def horizon_experiment():
    """예측 구간 N 스윕 — 보고서/심층 인터뷰용 근거 데이터."""
    print("LIP-MPC 예측 구간 튜닝 (dt=0.1, 외란: 초기속도 0.35 m/s, "
          "입력지연 40ms, 노이즈 5mm)")
    print(f"{'N':>4} {'구간(s)':>7} {'최대변위(m)':>11} {'정착시간(s)':>11} "
          f"{'풀이시간(us)':>12}")
    for n in (3, 5, 10, 20, 50, 80):
        mpc = LipMpc(horizon_n=n)
        t0 = time.perf_counter()
        for _ in range(200):
            mpc.solve(0.05, 0.1)
        us = (time.perf_counter() - t0) / 200 * 1e6
        max_p, settle = _simulate_push(mpc, v0=0.35)
        settle_s = "%.2f" % settle if settle else "발산/전도"
        print(f"{n:>4} {n*0.1:>7.1f} {max_p:>11.3f} {settle_s:>11} {us:>12.1f}")
    print("\n결론 기준: 구간이 LIP 시정수(√(h/g)≈0.18s)의 수 배를 넘으면 "
          "성능이 포화하고, 짧으면 (포화 입력 하에서) 회복 여유를 과소평가한다.")


if __name__ == "__main__":
    horizon_experiment()
