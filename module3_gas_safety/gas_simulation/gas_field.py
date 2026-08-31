#!/usr/bin/env python3
"""가스 누출·기류 확산 시뮬레이터 (가우시안 플룸 모델).

모델 (정상상태 가우시안 플룸의 2D 지면 근사):
  누출원 S에서 바람 벡터 U 방향으로 플룸이 흘러가며,
  풍하 거리 d 에 따라 횡방향 퍼짐 σ(d) = σ0 + α·d 로 넓어지고
  농도는 1/(U·σ) 로 희석된다:
      C(p) = Q / (U·σ(d)) · exp(-c²/(2σ(d)²)) ,  (d: 풍하, c: 횡방향 성분)
  풍상(d<0)에는 소량의 등방 확산만 존재. 난류는 시변 게인 노이즈로 재현.

출력:
  /gas/concentration (Float32)  — 로봇 위치의 농도 (ppm) = 로봇 가스센서
  /gas/field (OccupancyGrid)    — 공장 영역 농도장 (관제 시각화, 1Hz)
  /gas/source_truth (Point)     — 누출원 위치 ground truth (평가용)
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32

SOURCE = (-9.0, 6.0)        # 가스탱크 위치
WIND = (0.8, -0.55)         # 공장 내부로 흐르는 기류 (m/s)
Q = 120.0                   # 방출 강도 (ppm·m²/s 스케일)
SIGMA0, ALPHA = 0.8, 0.35   # 초기 퍼짐, 풍하 성장률


class GasField(Node):
    def __init__(self):
        super().__init__("gas_field")
        self.t = 0.0
        self.robot = None
        self.pub_c = self.create_publisher(Float32, "/gas/concentration", 10)
        self.pub_f = self.create_publisher(OccupancyGrid, "/gas/field", 1)
        self.pub_s = self.create_publisher(Point, "/gas/source_truth", 1)
        self.pub_w = self.create_publisher(Vector3, "/gas/wind", 1)  # 풍향계 시뮬
        self.create_subscription(Odometry, "/model/go2/odometry", self._on_odom, 10)
        self.create_timer(0.2, self._sense)     # 가스센서 5Hz
        self.create_timer(1.0, self._field)
        self.get_logger().info("gas_field 시작 (source=%s, wind=%s)"
                               % (SOURCE, WIND))

    def concentration(self, x, y, t):
        u = math.hypot(*WIND)
        ux, uy = WIND[0] / u, WIND[1] / u
        dx, dy = x - SOURCE[0], y - SOURCE[1]
        d = dx * ux + dy * uy          # 풍하 거리
        c = -dx * uy + dy * ux         # 횡방향 거리
        if d > 0:
            sig = SIGMA0 + ALPHA * d
            base = Q / (u * sig) * math.exp(-c * c / (2 * sig * sig)) \
                * math.exp(-0.02 * d)                    # 침적/희석 감쇠
        else:
            r2 = dx * dx + dy * dy
            base = Q / u * 0.25 * math.exp(-r2 / (2 * SIGMA0 ** 2))
        # 난류: 시변 저주파 게인 (0.8~1.2)
        gust = 1.0 + 0.2 * math.sin(0.7 * t + 0.13 * x) * math.cos(0.5 * t + 0.11 * y)
        return max(base * gust, 0.0)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.robot = (p.x, p.y)

    def _sense(self):
        self.t += 0.2
        if self.robot is None:
            return
        c = self.concentration(*self.robot, self.t)
        c += abs(np.random.normal(0, 0.5))              # 센서 노이즈
        self.pub_c.publish(Float32(data=float(c)))
        self.pub_s.publish(Point(x=SOURCE[0], y=SOURCE[1]))
        # 풍향계: 실제 풍향 + 약간의 측정 노이즈
        self.pub_w.publish(Vector3(
            x=WIND[0] + float(np.random.normal(0, 0.08)),
            y=WIND[1] + float(np.random.normal(0, 0.08))))

    def _field(self):
        # 공장 영역 (25×18, 0.5m 격자) 농도장 시각화
        res, w, h = 0.5, 50, 36
        grid = OccupancyGrid()
        grid.header.frame_id = "world"
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = res
        grid.info.width = w
        grid.info.height = h
        grid.info.origin.position.x = -12.5
        grid.info.origin.position.y = -9.0
        data = []
        for j in range(h):
            for i in range(w):
                c = self.concentration(-12.5 + (i + 0.5) * res,
                                       -9.0 + (j + 0.5) * res, self.t)
                data.append(min(int(c), 100))
        grid.data = data
        self.pub_f.publish(grid)


def main():
    rclpy.init()
    node = GasField()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
