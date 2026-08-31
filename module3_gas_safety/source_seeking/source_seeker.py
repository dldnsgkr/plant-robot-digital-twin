#!/usr/bin/env python3
"""가스 누출원 탐색 (Source Seeking) — 경사 상승 + 국소최적 탈출 + 장애물 회피.

알고리즘:
  1. 이동 궤적의 슬라이딩 윈도(8s)에서 (x, y, C) 최소제곱 평면 적합
     C ≈ a + gx·x + gy·y  →  농도 구배 ∇C = (gx, gy)
  2. CLIMB: ∇C 방향으로 요를 P제어하며 전진 (경사 상승법).
     농도가 충분하면(플룸 내부) 풍상(upwind) 방향을 혼합 — 플룸 추적의
     표준 'surge' 행동으로, 난류 속 구배 배회를 줄이고 소스로 직행
  3. CAST: 구배가 약하거나(평탄/국소최적) 신뢰도가 낮으면 확장 나선 탐색
     — 난류로 인한 순간 구배 부호 반전에 강인, 국소최적 탈출 전략
  4. FOUND: 농도가 임계(45ppm) 이상 2초 지속 → 정지, 위치 보고
  5. 장애물 회피: LiDAR 포인트(몸높이 대역, 전방 ±70°) 반발 조향을
     구배 조향에 중첩 (potential field)

토픽: /gas/concentration, /model/go2/odometry, /lidar/points → /cmd_vel
      발견 시 /gas/source_estimate (Point) + /gas/found (Bool)
"""
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32

FOUND_PPM = 40.0
SURGE_PPM = 12.0      # 이 이상이면 플룸 내부로 보고 풍상 직진
GRAD_MIN = 0.6        # 이보다 약한 구배는 난류 노이즈로 간주 (ppm/m)
V_WALK = 0.22


class SourceSeeker(Node):
    def __init__(self):
        super().__init__("source_seeker")
        self.declare_parameter("enabled", True)
        # 통합 시나리오: 가스 알람(/gas/alarm)을 받아야 탐색 시작
        self.declare_parameter("wait_alarm", False)
        self.alarmed = False
        self.samples = deque(maxlen=40)   # (x, y, C) @5Hz → 8s 윈도
        self.pose = None                  # (x, y, yaw)
        self.conc = 0.0
        self.found_since = None
        self.done = False
        self.rth = False
        self.cast_t = 0.0
        self.repulse = 0.0                # 장애물 반발 조향량
        self.front_block = False
        self.wind = None                  # 풍향계 (surge upwind용)
        # 최고 농도 메모리 (백트래킹): 플룸 이탈 시 복귀할 기준점
        self.best_c = 0.0
        self.best_pos = None
        self.best_t = 0.0

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_est = self.create_publisher(Point, "/gas/source_estimate", 1)
        self.pub_found = self.create_publisher(Bool, "/gas/found", 1)
        self.create_subscription(Float32, "/gas/concentration", self._on_gas, 10)
        self.create_subscription(Odometry, "/model/go2/odometry", self._on_odom, 10)
        self.create_subscription(PointCloud2, "/lidar/points", self._on_cloud, 1)
        self.create_subscription(Bool, "/rth_active", self._on_rth, 1)
        self.create_subscription(Vector3, "/gas/wind", self._on_wind, 1)
        self.create_subscription(Bool, "/gas/alarm", self._on_alarm, 1)

    def _on_alarm(self, msg):
        if msg.data and not self.alarmed:
            self.alarmed = True
            self.get_logger().info("가스 알람 수신 — 누출원 탐색 시작")
        self.create_timer(0.1, self._step)
        self._dbg = ("-", 0.0)
        self.create_timer(2.0, self._debug)
        self.get_logger().info("source_seeker 시작")

    def _debug(self):
        if self.pose:
            self.get_logger().info(
                "dbg mode=%s tgt=%.2f yaw=%.2f pos=(%.1f,%.1f) C=%.1f rep=%.2f blk=%s"
                % (self._dbg[0], self._dbg[1], self.pose[2], self.pose[0],
                   self.pose[1], self.conc, self.repulse, self.front_block))

    def _on_rth(self, msg):
        self.rth = msg.data

    def _on_wind(self, msg):
        self.wind = (msg.x, msg.y)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def _on_gas(self, msg):
        self.conc = msg.data
        if self.pose:
            self.samples.append((self.pose[0], self.pose[1], msg.data))

    def _on_cloud(self, msg):
        """몸높이 대역·전방 포인트로 반발 조향량 계산 (자기 몸 제외)."""
        pts = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) == 0:
            self.repulse, self.front_block = 0.0, False
            return
        keep = (pts[:, 2] > -0.22) & (pts[:, 2] < 0.35) & \
               (np.abs(pts[:, 1]) < 1.2) & (pts[:, 0] > 0.35) & (pts[:, 0] < 1.6)
        pts = pts[keep]
        if len(pts) == 0:
            self.repulse, self.front_block = 0.0, False
            return
        d = np.hypot(pts[:, 0], pts[:, 1])
        w = np.clip(1.2 - d, 0, None) ** 2
        # 장애물이 왼쪽(+y)에 있으면 오른쪽(-wz)으로 조향
        self.repulse = float(-1.8 * np.sum(np.sign(pts[:, 1] + 1e-6) * w)
                             / max(len(pts), 1))
        self.front_block = bool(np.any((d < 0.55) & (np.abs(pts[:, 1]) < 0.35)))

    def _gradient(self):
        """윈도 최소제곱 평면 적합 → (gx, gy) 또는 None (스프레드 부족)."""
        if len(self.samples) < 15:
            return None
        arr = np.array(self.samples)
        xy = arr[:, :2]
        if xy.std(axis=0).max() < 0.15:   # 위치 스프레드 부족 → 적합 불가
            return None
        A = np.c_[np.ones(len(arr)), xy]
        sol, *_ = np.linalg.lstsq(A, arr[:, 2], rcond=None)
        return sol[1], sol[2]

    def _step(self):
        if self.done or self.rth or self.pose is None \
                or not self.get_parameter("enabled").value:
            return
        if self.get_parameter("wait_alarm").value and not self.alarmed:
            return
        cmd = Twist()

        # 발견 판정: 임계 농도 2초 지속
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.conc > FOUND_PPM:
            if self.found_since is None:
                self.found_since = now
            elif now - self.found_since > 2.0:
                self.done = True
                self.pub_cmd.publish(Twist())   # 정지
                self.pub_est.publish(Point(x=self.pose[0], y=self.pose[1]))
                self.pub_found.publish(Bool(data=True))
                self.get_logger().info(
                    "누출원 발견! (%.1f, %.1f) C=%.1fppm"
                    % (self.pose[0], self.pose[1], self.conc))
                return
        else:
            self.found_since = None

        # 최고 농도 메모리 갱신 (서서히 감쇠 — 돌풍 피크 과신 방지)
        self.best_c *= 0.999
        if self.conc > self.best_c:
            self.best_c = self.conc
            self.best_pos = (self.pose[0], self.pose[1])
            self.best_t = now

        # 백트래킹: 최고 기록 대비 크게 하락한 채 12초 경과 → 기준점 복귀
        if (self.best_pos is not None and self.conc < 0.6 * self.best_c
                and now - self.best_t > 12.0 and self.best_c > 12.0):
            dx = self.best_pos[0] - self.pose[0]
            dy = self.best_pos[1] - self.pose[1]
            if math.hypot(dx, dy) > 0.5:
                gn = math.hypot(dx, dy)
                cy, sy = math.cos(self.pose[2]), math.sin(self.pose[2])
                cmd.linear.x = V_WALK * (cy * dx + sy * dy) / gn
                cmd.linear.y = V_WALK * (-sy * dx + cy * dy) / gn
                self._dbg = ("RETURN", math.atan2(dy, dx))
                self._avoid_and_publish(cmd)
                return
            else:
                self.best_t = now   # 도착 — 타이머 리셋 후 재탐색

        grad = self._gradient()
        gmag = math.hypot(*grad) if grad else 0.0

        # 모드 히스테리시스: 돌풍으로 농도가 문턱 주변에서 요동해도
        # SURGE에 한번 들어가면 8초 유지 (방향 플래핑 방지)
        if self.conc > SURGE_PPM and self.wind is not None:
            self.surge_until = now + 8.0
        in_plume = getattr(self, "surge_until", 0.0) > now and self.wind is not None
        weak_signal = self.conc > 5.0 and grad and gmag > GRAD_MIN
        if in_plume or weak_signal:
            if in_plume:
                # SURGE: 풍상 전진 + 횡풍 센터링 — 구배의 횡풍 성분만 사용해
                # 플룸 중심선을 유지 (풍상 성분 노이즈는 무시, 이탈 방지)
                w = math.hypot(*self.wind)
                ux, uy = -self.wind[0] / w, -self.wind[1] / w
                cxw, cyw = -uy, ux                  # 횡풍 단위벡터
                s = 0.0
                if grad:
                    s = max(-0.8, min(0.8, (grad[0] * cxw + grad[1] * cyw) / 3.0))
                gx = ux + s * cxw
                gy = uy + s * cyw
            else:
                # CLIMB: 약한 신호 — 농도 구배 상승으로 플룸 탐색
                gx, gy = grad
            # 헤딩 조향: 목표 방위로 회전하며 전진
            target = math.atan2(gy, gx)
            err = math.atan2(math.sin(target - self.pose[2]),
                             math.cos(target - self.pose[2]))
            cmd.linear.x = V_WALK * max(0.0, math.cos(err))
            cmd.angular.z = max(-0.4, min(0.4, 1.0 * err))
            self.cast_t = 0.0
            self._dbg = ("SURGE" if in_plume else "CLIMB", target)
        else:
            # CAST: 확장 나선 (갈수록 큰 원으로 플룸 탐색)
            self.cast_t += 0.1
            cmd.linear.x = V_WALK
            cmd.angular.z = 0.4 / (1.0 + 0.1 * self.cast_t)
            self._dbg = ("CAST", 0.0)

        self._avoid_and_publish(cmd)

    def _avoid_and_publish(self, cmd):
        """장애물 회피 중첩: 반발은 게걸음(vy)으로, 전방 차단 시 옆걸음 탈출."""
        if self.front_block:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.15 if self.repulse >= 0 else -0.15
        else:
            cmd.linear.y += max(-0.12, min(0.12, self.repulse * 0.25))
        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = SourceSeeker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
