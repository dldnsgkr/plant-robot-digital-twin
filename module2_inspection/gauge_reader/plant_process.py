#!/usr/bin/env python3
"""플랜트 프로세스 시뮬레이터 — 게이지 바늘 구동 + ground truth 발행.

압력값(0~10 bar)을 시뮬레이션하고:
  1. gauge_needle 모델의 pose를 gz set_pose 서비스로 회전 (y축 피치)
  2. /plant/gauge_pressure (Float32) 로 실제 값 발행 → 판독 오차 검증 기준

각도 규약 (make_dial.py 와 일치):
  뷰어 각 θ = 225° - 27°·v, 실측 매핑 θ_view = -φ  →  φ = 27°·v - 225°
"""
import math
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

GAUGE_POS = (38.0, 1.575, 0.6)


class PlantProcess(Node):
    def __init__(self):
        super().__init__("plant_process")
        # mode: 'sine'(연속 변화) | 'sweep'(0.5bar 계단, 판독 정확도 평가용)
        self.declare_parameter("mode", "sine")
        self.declare_parameter("fixed_value", -1.0)  # >=0 이면 고정값
        self.t = 0.0
        self.value = 5.0
        self.pub = self.create_publisher(Float32, "/plant/gauge_pressure", 10)
        self.create_timer(0.5, self._step)
        self.get_logger().info("plant_process 시작")

    def _set_needle(self, v):
        phi = math.radians(27.0 * v - 225.0)
        qy, qw = math.sin(phi / 2), math.cos(phi / 2)
        req = ('name: "gauge_needle", position: {x: %g, y: %g, z: %g}, '
               'orientation: {x: 0, y: %.6f, z: 0, w: %.6f}'
               % (*GAUGE_POS, qy, qw))
        subprocess.run(
            ["gz", "service", "-s", "/world/plant/set_pose",
             "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
             "--timeout", "1000", "--req", req],
            capture_output=True, timeout=3)

    def _step(self):
        self.t += 0.5
        fixed = self.get_parameter("fixed_value").value
        mode = self.get_parameter("mode").value
        if fixed >= 0:
            self.value = fixed
        elif mode == "sweep":
            self.value = (self.t / 4) % 10.5   # 2초마다 0.5bar 증가
        else:
            self.value = 5.0 + 3.5 * math.sin(self.t / 20)
        self._set_needle(self.value)
        self.pub.publish(Float32(data=float(self.value)))


def main():
    rclpy.init()
    node = PlantProcess()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
