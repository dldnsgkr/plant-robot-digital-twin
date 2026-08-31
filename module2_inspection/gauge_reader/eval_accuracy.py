#!/usr/bin/env python3
"""게이지 판독 정확도 평가 — 오차 5% 이내(풀스케일 10bar 기준 0.5bar) 검증.

plant_process(sweep/고정값)와 gauge_reader가 실행 중인 상태에서:
  ground truth(/plant/gauge_pressure)와 판독값(/inspection/gauge_value)을
  짝지어 수집하고 오차 통계를 출력한다.

사용: python3 eval_accuracy.py [수집시간(초, 기본 40)]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class Eval(Node):
    def __init__(self):
        super().__init__("gauge_eval")
        self.truth = None
        self.pairs = []
        self.create_subscription(Float32, "/plant/gauge_pressure", self._t, 10)
        self.create_subscription(Float32, "/inspection/gauge_value", self._v, 10)

    def _t(self, msg):
        self.truth = msg.data

    def _v(self, msg):
        if self.truth is not None:
            self.pairs.append((self.truth, msg.data))


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
    rclpy.init()
    n = Eval()
    t0 = time.time()
    while time.time() - t0 < dur:
        rclpy.spin_once(n, timeout_sec=0.2)

    if not n.pairs:
        print("FAIL: 판독값 수집 0건 (gauge_reader가 다이얼을 못 찾는 상태)")
        return
    errs = [abs(t - v) for t, v in n.pairs]
    errs.sort()
    fs_pct = [e / 10.0 * 100 for e in errs]        # 풀스케일(10bar) 기준 %
    mean_e = sum(errs) / len(errs)
    p95 = errs[int(len(errs) * 0.95) - 1]
    print("수집 %d건 | 평균오차 %.3f bar (FS %.1f%%) | 95%%ile %.3f bar (FS %.1f%%)"
          % (len(n.pairs), mean_e, mean_e / 10 * 100, p95, p95 / 10 * 100))
    ok = p95 / 10 * 100 <= 5.0
    print("판정: %s (목표: 95%%ile 오차 <= FS 5%%)" % ("PASS" if ok else "FAIL"))
    for t, v in n.pairs[::max(1, len(n.pairs) // 8)]:
        print("  truth %.2f → read %.2f (err %.2f)" % (t, v, abs(t - v)))


if __name__ == "__main__":
    main()
