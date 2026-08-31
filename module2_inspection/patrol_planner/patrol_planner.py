#!/usr/bin/env python3
"""Viewpoint 기반 최적 순찰 경로 생성.

각 점검 대상에 대해 '인식이 가장 잘 되는 관측점(Viewpoint)'을 비용 최소화로
선정하고, 선정된 관측점들을 TSP(최근접 삽입 + 2-opt)로 순회 순서 최적화한다.

관측점 비용 (동료평가 '가중치·비용 계산' 답변):
  cost = w_ang·(정면 법선과의 각도)²      # 사각(斜角) 판독은 원근 왜곡 유발
       + w_dst·(거리 - d_opt)²            # 너무 멀면 해상도↓, 가까우면 FOV 밖
       + ∞ (장애물 충돌 or 시선 가림)     # 유효성 하드 제약
  d_opt: 다이얼이 카메라 프레임의 ~25%를 차지하는 거리 (해상도-시야 균형)

출력: patrol_route.yaml (Nav2 waypoint 목록) + 콘솔 요약표
사용: python3 patrol_planner.py
"""
import math

# ---- 점검 대상: (이름, x, y, 바라볼 높이, 법선 방향[rad], 최적 거리) ----
TARGETS = [
    ("gauge_panel", 38.0, 1.62, 0.6, -math.pi / 2, 1.4),   # 복도 북벽, -y 방향
    ("machine_pipe_joint", -7.5, -5.4, 1.5, 0.0, 2.0),     # 공장기계, +x 방향
    ("machine_motor", -7.1, -6.0, 0.6, 0.0, 1.6),
    ("gas_tank", -9.0, 6.0, 1.5, 0.0, 2.2),
]
HOME = ("charging_station", 11.0, 7.5)

# ---- 월드 장애물 (plant_world.sdf 기준, 2D 원/사각 근사) ----
OBSTACLES = [
    # (cx, cy, 반경) 원 근사
    ("factory_barrel", 8.0, 2.0, 0.45),
    ("gas_tank_body", -9.0, 6.0, 1.35),
    ("factory_crate", 5.0, 5.0, 0.55),
    ("pallet1", -6.0, 4.0, 0.8),
    ("pallet2", 3.0, -5.0, 0.8),
    ("fallen_pipe", -3.0, -3.0, 1.4),
    ("machine", -9.0, -6.0, 2.0),
    ("stairs", 10.6, -7.0, 1.8),
]
# 건물 내부 영역 (공장 ∪ 복도), 벽 여유 0.5m
FACTORY = (-12.0, 12.0, -8.5, 8.5)
CORRIDOR = (13.0, 57.0, -1.25, 1.25)

W_ANG = 2.0     # rad² 당 가중치
W_DST = 1.0     # m² 당 가중치


def in_free_space(x, y, clearance=0.4):
    inside = (FACTORY[0] < x < FACTORY[1] and FACTORY[2] < y < FACTORY[3]) or \
             (CORRIDOR[0] < x < CORRIDOR[1] and CORRIDOR[2] < y < CORRIDOR[3])
    if not inside:
        return False
    return all(math.hypot(x - ox, y - oy) > r + clearance
               for _, ox, oy, r in OBSTACLES)


def sight_blocked(x0, y0, x1, y1):
    """관측점→대상 시선이 장애물 원을 지나는가 (선분-원 교차)."""
    for name, ox, oy, r in OBSTACLES:
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        if L2 < 1e-9:
            continue
        t = max(0.0, min(1.0, ((ox - x0) * dx + (oy - y0) * dy) / L2))
        # 대상 바로 옆 장애물(자기 자신)은 시선 끝단 10%를 제외하고 검사
        if t > 0.9:
            continue
        px, py = x0 + t * dx, y0 + t * dy
        if math.hypot(ox - px, oy - py) < r:
            return True
    return False


def best_viewpoint(name, tx, ty, tz, normal, d_opt):
    """법선 주변 ±60°, 거리 0.8~3.0m 격자에서 비용 최소 관측점."""
    best, best_cost = None, float("inf")
    for da in [math.radians(a) for a in range(-60, 61, 10)]:
        for d in [0.8 + 0.2 * i for i in range(12)]:
            ang = normal + da
            vx = tx + d * math.cos(ang)
            vy = ty + d * math.sin(ang)
            if not in_free_space(vx, vy):
                continue
            if sight_blocked(vx, vy, tx, ty):
                continue
            cost = W_ANG * da * da + W_DST * (d - d_opt) ** 2
            if cost < best_cost:
                # 로봇이 대상을 바라보는 방위
                yaw = math.atan2(ty - vy, tx - vx)
                best, best_cost = (vx, vy, yaw), cost
    return best, best_cost


def tsp_order(points, start):
    """최근접 이웃 + 2-opt."""
    def dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    unvisited = list(range(len(points)))
    order = []
    cur = start
    while unvisited:
        nxt = min(unvisited, key=lambda i: dist(cur, points[i]))
        order.append(nxt)
        cur = points[nxt]
        unvisited.remove(nxt)

    def tour_len(o):
        total = dist(start, points[o[0]])
        for i in range(len(o) - 1):
            total += dist(points[o[i]], points[o[i + 1]])
        return total + dist(points[o[-1]], start)   # 귀환 포함

    improved = True
    while improved:
        improved = False
        for i in range(len(order) - 1):
            for j in range(i + 1, len(order)):
                cand = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
                if tour_len(cand) < tour_len(order) - 1e-6:
                    order = cand
                    improved = True
    return order, tour_len(order)


def main():
    print("== Viewpoint 선정 ==")
    vps, names = [], []
    for name, tx, ty, tz, normal, d_opt in TARGETS:
        vp, cost = best_viewpoint(name, tx, ty, tz, normal, d_opt)
        if vp is None:
            print("  %s: 유효 관측점 없음!" % name)
            continue
        d = math.hypot(vp[0] - tx, vp[1] - ty)
        print("  %-18s → vp(%6.2f, %6.2f) yaw %5.1f° 거리 %.2fm (비용 %.3f)"
              % (name, vp[0], vp[1], math.degrees(vp[2]), d, cost))
        vps.append(vp)
        names.append(name)

    start = (HOME[1], HOME[2])
    order, length = tsp_order(vps, start)
    print("\n== 순찰 순서 (TSP, 충전소 출발·복귀) ==")
    print("  %s → %s → %s" % (HOME[0], " → ".join(names[i] for i in order), HOME[0]))
    print("  총 이동 거리(직선 근사): %.1f m" % length)

    out = __file__.rsplit("/", 1)[0] + "/patrol_route.yaml"
    with open(out, "w") as f:
        f.write("# 자동 생성: patrol_planner.py — Nav2 waypoint follower 입력\n")
        f.write("waypoints:\n")
        for i in order:
            vp = vps[i]
            f.write("  - name: %s\n    x: %.2f\n    y: %.2f\n    yaw: %.3f\n"
                    % (names[i], vp[0], vp[1], vp[2]))
        f.write("  - name: %s\n    x: %.2f\n    y: %.2f\n    yaw: 0.0\n"
                % (HOME[0], HOME[1], HOME[2]))
    print("저장: %s" % out)


if __name__ == "__main__":
    main()
