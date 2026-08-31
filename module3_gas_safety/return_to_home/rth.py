#!/usr/bin/env python3
"""자율 복귀 (Return to Home) — 배터리/통신 트리거 + A* 경로 + 게걸음 추종.

트리거 (요구사항):
  1. 배터리 잔량 < 20%  (이동 시 0.25%/s, 대기 시 0.05%/s 소모 모델)
  2. 통신 음영 구역 진입 예상 (복도 깊숙한 x>48 구역에 5초 이상 체류)

경로 계획:
  월드 기하(벽 + 장애물, 0.4m 팽창)를 0.25m 격자로 이산화 → A*(8방향).
  로봇의 기하학적 최단 경로가 장애물을 우회하도록 보장.

추종:
  waypoint를 축 정렬 게걸음으로 폐루프 추종 — 이 게이트는 대각(vx+vy 동시)
  이동에서 기생 결합이 커서, 검증된 두 프리미티브(순수 전진 / 순수 측방)만
  번갈아 사용한다(몸좌표 오차의 큰 축 우선, 1.5s 유지로 채터링 방지).

토픽: /robot/battery(Float32), /rth_active(Bool — source_seeker 양보),
      /cmd_vel, 도킹 완료 시 /robot/docked(Bool)
"""
import heapq
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32

HOME = (11.0, 7.5)
RES = 0.25
INFLATE = 0.4
V_WALK = 0.22

# 월드 기하 (plant_world.sdf — patrol_planner와 동일 모델)
OBSTACLES = [
    (8.0, 2.0, 0.45), (-9.0, 6.0, 1.35), (5.0, 5.0, 0.55), (-6.0, 4.0, 0.8),
    (3.0, -5.0, 0.8), (-3.0, -3.0, 1.4), (-9.0, -6.0, 2.0), (10.6, -7.0, 1.8),
]
FACTORY = (-12.0, 12.0, -8.5, 8.5)
CORRIDOR = (13.0, 57.0, -1.25, 1.25)
# 격자 범위 (공장+복도 포함)
GX0, GX1, GY0, GY1 = -12.5, 58.0, -9.0, 9.0
NX = int((GX1 - GX0) / RES)
NY = int((GY1 - GY0) / RES)


def free(x, y):
    # 복도 시작을 공장 경계와 겹치게(-1.2) 두어 출입구가 격자에서 연결되게 함
    inside = (FACTORY[0] < x < FACTORY[1] and FACTORY[2] < y < FACTORY[3]) or \
             (CORRIDOR[0] - 1.2 < x < CORRIDOR[1] and CORRIDOR[2] < y < CORRIDOR[3])
    if not inside:
        return False
    return all(math.hypot(x - ox, y - oy) > r + INFLATE
               for ox, oy, r in OBSTACLES)


def astar(start, goal):
    """0.25m 격자 8방향 A*. waypoint 목록(월드 좌표) 반환."""
    def cell(p):
        return (int((p[0] - GX0) / RES), int((p[1] - GY0) / RES))

    def world(c):
        return (GX0 + (c[0] + 0.5) * RES, GY0 + (c[1] + 0.5) * RES)

    s, g = cell(start), cell(goal)
    openq = [(0.0, s)]
    came, cost = {s: None}, {s: 0.0}
    moves = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
             (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    while openq:
        _, cur = heapq.heappop(openq)
        if cur == g:
            path = []
            while cur:
                path.append(world(cur))
                cur = came[cur]
            path.reverse()
            # 경로 솎아내기 (0.75m 간격)
            out = [path[0]]
            for p in path[1:]:
                if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > 0.75:
                    out.append(p)
            out.append(goal)
            return out
        for dx, dy, w in moves:
            nxt = (cur[0] + dx, cur[1] + dy)
            if not (0 <= nxt[0] < NX and 0 <= nxt[1] < NY):
                continue
            if not free(*world(nxt)):
                continue
            nc = cost[cur] + w * RES
            if nxt not in cost or nc < cost[nxt]:
                cost[nxt] = nc
                came[nxt] = cur
                h = math.hypot(world(nxt)[0] - goal[0], world(nxt)[1] - goal[1])
                heapq.heappush(openq, (nc + h, nxt))
    return None


class Rth(Node):
    def __init__(self):
        super().__init__("return_to_home")
        self.declare_parameter("start_pct", 100.0)
        self.declare_parameter("drain_move", 0.25)   # %/s
        self.declare_parameter("drain_idle", 0.05)
        self.battery = self.get_parameter("start_pct").value
        self.pose = None
        self.speed = 0.0
        self.active = False
        self.docked = False
        self.path = None
        self.wp_i = 0
        self.shadow_since = None

        self.pub_bat = self.create_publisher(Float32, "/robot/battery", 5)
        self.pub_act = self.create_publisher(Bool, "/rth_active", 5)
        self.pub_dock = self.create_publisher(Bool, "/robot/docked", 5)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/model/go2/odometry", self._on_odom, 10)
        self.create_timer(0.1, self._follow)
        self.create_timer(1.0, self._battery_tick)
        self.get_logger().info("return_to_home 시작 (battery %.0f%%)" % self.battery)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)
        self.speed = math.hypot(msg.twist.twist.linear.x,
                                msg.twist.twist.linear.y)

    def _battery_tick(self):
        if self.docked:
            self.battery = min(100.0, self.battery + 2.0)
            self.pub_bat.publish(Float32(data=self.battery))
            return
        drain = self.get_parameter(
            "drain_move" if self.speed > 0.05 else "drain_idle").value
        self.battery = max(0.0, self.battery - drain)
        self.pub_bat.publish(Float32(data=self.battery))

        now = self.get_clock().now().nanoseconds * 1e-9
        # 통신 음영 (복도 깊숙) 체류 감지
        in_shadow = self.pose is not None and self.pose[0] > 48.0
        if in_shadow and self.shadow_since is None:
            self.shadow_since = now
        elif not in_shadow:
            self.shadow_since = None

        low_batt = self.battery < 20.0
        shadow = self.shadow_since is not None and now - self.shadow_since > 5.0
        if not self.active and self.pose is not None and (low_batt or shadow):
            reason = "배터리 %.0f%%" % self.battery if low_batt else "통신 음영"
            self.path = astar((self.pose[0], self.pose[1]), HOME)
            if self.path is None:
                self.get_logger().error("A* 경로 실패!")
                return
            self.active = True
            self.wp_i = 0
            self.get_logger().warn("자율 복귀 시작 (%s) — waypoint %d개"
                                   % (reason, len(self.path)))
        self.pub_act.publish(Bool(data=self.active and not self.docked))

    def _follow(self):
        if not self.active or self.docked or self.pose is None:
            return
        x, y, yaw = self.pose
        # 룩어헤드: 현재 위치에서 0.3m 이내 waypoint는 통과 처리
        while self.wp_i < len(self.path) - 1 and \
                math.hypot(self.path[self.wp_i][0] - x,
                           self.path[self.wp_i][1] - y) < 0.5:
            self.wp_i += 1
        tx, ty = self.path[self.wp_i]
        dx, dy = tx - x, ty - y
        d_goal = math.hypot(HOME[0] - x, HOME[1] - y)
        if d_goal < 0.5:
            self.docked = True
            self.pub_cmd.publish(Twist())
            self.pub_dock.publish(Bool(data=True))
            self.get_logger().info("충전 스테이션 도킹 완료 (오차 %.2fm)" % d_goal)
            return
        # 몸좌표 오차 → 축 정렬 이동 (1.5s 유지)
        cy, sy = math.cos(yaw), math.sin(yaw)
        ex = cy * dx + sy * dy
        ey = -sy * dx + cy * dy
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - getattr(self, "_axis_t", 0.0) > 1.5:
            self._axis = "x" if abs(ex) > abs(ey) else "y"
            self._axis_t = now
        cmd = Twist()
        if self._axis == "x":
            # 후진은 요 불안정이 커서 저속으로 제한
            cmd.linear.x = min(V_WALK, max(-0.1, V_WALK * (1 if ex > 0 else -1)))
        else:
            cmd.linear.y = 0.15 * (1 if ey > 0 else -1)
        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = Rth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
