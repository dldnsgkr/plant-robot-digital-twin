#!/usr/bin/env python3
"""elevation_map 검증 헬퍼: 맵 통계와 벽/지면 높이 샘플을 출력."""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

N = 80
RES = 0.05
HALF = 2.0


class Check(Node):
    def __init__(self):
        super().__init__("check_elevmap")
        self.create_subscription(Float32MultiArray, "/elevation_map/raw", self.cb, 1)
        self.create_subscription(Float32MultiArray, "/footholds", self.cb_feet, 1)
        self.done = False
        self.feet = None

    def cb_feet(self, msg):
        self.feet = list(msg.data)

    def cb(self, msg):
        m = np.array(msg.data).reshape(N, N)
        valid = ~np.isnan(m)
        print("유효 셀: %d / %d (%.0f%%)" % (valid.sum(), N * N, 100 * valid.sum() / N / N))
        if valid.sum() == 0:
            self.done = True
            return

        def cell(x, y):
            i, j = int((x + HALF) / RES), int((y + HALF) / RES)
            v = m[i, j]
            return "nan" if math.isnan(v) else "%.3f" % v

        print("지면 샘플 h(x=1.0,y=0):", cell(1.0, 0.0), " h(x=-1.0,y=0):", cell(-1.0, 0.0))
        print("벽 샘플  h(x=0,y=+1.7):", cell(0.0, 1.7), " h(x=0,y=-1.7):", cell(0.0, -1.7))
        ground = m[valid & (np.abs(m) < 0.1)]
        print("지면대 셀 수: %d, 평균 %.3f, 표준편차 %.3f"
              % (len(ground), ground.mean() if len(ground) else 0,
                 ground.std() if len(ground) else 0))
        if self.feet:
            print("footholds:", ["%.2f" % v for v in self.feet])
        self.done = True


def main():
    rclpy.init()
    n = Check()
    import time
    t0 = time.time()
    while not n.done and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.5)


if __name__ == "__main__":
    main()
