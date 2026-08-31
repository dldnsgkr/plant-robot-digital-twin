#!/usr/bin/env python3
"""Go2 게이트 생성기 (stage1) — 트로트/크롤 보행 + 해석적 다리 IK.

/cmd_vel (Twist) 를 받아 12관절 목표각을
/joint_group_position_controller/commands (Float64MultiArray) 로 발행한다.

보행 원리:
  - 위상 오실레이터가 다리별 위상(0~1)을 돌리고, 위상에 따라 각 발이
    지지(stance: 몸 아래에서 뒤로 밀기) / 유각(swing: 사이클로이드로 앞으로 복귀)을 반복
  - 발 목표 위치(힙 기준 3D)를 해석적 IK로 관절각 (hip roll, thigh pitch, calf pitch) 변환
  - 게이트 전환: trot(대각 2쌍, duty 0.5) ↔ crawl(한 다리씩, duty 0.75, 험지용 저속 안정 보행)

실행:
  ros2 run … 또는 python3 gait_controller.py
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}' -r 10
  게이트 전환: ros2 param set /gait_controller gait crawl
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64MultiArray

# ---- Go2 기구 파라미터 (urdf 실측) ----
L_HIP = 0.0955     # 힙 롤축 → 다리 평면 y 오프셋
L_THIGH = 0.213
L_CALF = 0.213
HIP_X = 0.1934     # base → 힙 x 오프셋
HIP_Y = 0.0465     # base → 힙 y 오프셋

BODY_HEIGHT = 0.30  # 지지 시 힙 아래 발 깊이
STAND = (0.0, 0.8, -1.6)

# 다리 순서는 컨트롤러 yaml의 관절 순서와 동일해야 한다
LEGS = ("FL", "FR", "RL", "RR")
SIDE = {"FL": +1, "FR": -1, "RL": +1, "RR": -1}   # y 부호 (왼쪽 +)

# 게이트 정의: {다리: 위상 오프셋}, duty = 지지 구간 비율
GAITS = {
    "trot":  {"offset": {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5},
              "duty": 0.5, "period": 0.5, "swing_h": 0.06},
    "crawl": {"offset": {"FL": 0.0, "RR": 0.25, "FR": 0.5, "RL": 0.75},
              "duty": 0.75, "period": 1.2, "swing_h": 0.08},
}


def leg_ik(x, y, z, side):
    """힙 원점 기준 발 목표 (x앞+, y왼+, z위+) → (hip, thigh, calf) 관절각."""
    l_off = side * L_HIP
    # 힙 롤: 발을 다리 시상면으로 보내는 회전
    r_sq = y * y + z * z - l_off * l_off
    r = math.sqrt(max(r_sq, 1e-6))          # 시상면 내 힙-발 깊이
    q1 = math.atan2(z, y) + math.atan2(r, l_off)
    # 시상면 2링크 IK (q2: 수직 아래 기준, +는 발이 뒤로)
    d = math.sqrt(x * x + r * r)
    d = min(d, L_THIGH + L_CALF - 1e-4)
    cos_knee = (L_THIGH**2 + L_CALF**2 - d * d) / (2 * L_THIGH * L_CALF)
    q3 = math.acos(max(-1.0, min(1.0, cos_knee))) - math.pi
    psi = math.acos(max(-1.0, min(1.0,
        (L_THIGH**2 + d * d - L_CALF**2) / (2 * L_THIGH * d))))
    q2 = math.atan2(-x, r) + psi
    return q1, q2, q3


class GaitController(Node):
    def __init__(self):
        super().__init__("gait_controller")
        self.declare_parameter("gait", "trot")
        self.declare_parameter("rate_hz", 100.0)

        self.cmd = Twist()
        self.phase = 0.0
        self.active = False   # cmd_vel이 0이면 기립 유지
        self.enabled = True   # fall_recovery가 복구 중이면 false

        self.pub = self.create_publisher(
            Float64MultiArray, "/joint_group_position_controller/commands", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(Bool, "/gait_enable", self._on_enable, 10)

        rate = self.get_parameter("rate_hz").value
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self._step)
        self.get_logger().info("gait_controller 시작 (gait=%s)"
                               % self.get_parameter("gait").value)

    def _on_cmd(self, msg):
        self.cmd = msg

    def _on_enable(self, msg):
        self.enabled = msg.data
        if not msg.data:
            self.active = False
            self.phase = 0.0

    def _foot_target(self, leg, gait, phase):
        """다리별 위상 → 힙 기준 발 목표 좌표."""
        duty = gait["duty"]
        vx = self.cmd.linear.x
        vy = self.cmd.linear.y
        wz = self.cmd.angular.z
        # 회전 성분: 힙 위치에 따른 접선 속도 기여
        hx = HIP_X if leg[0] == "F" else -HIP_X
        hy = SIDE[leg] * (HIP_Y + L_HIP)
        vx_leg = vx - wz * hy
        vy_leg = vy + wz * hx

        t_stance = gait["period"] * duty
        step_x = vx_leg * t_stance / 2.0     # 발 스트로크 절반
        step_y = vy_leg * t_stance / 2.0

        p = (phase + gait["offset"][leg]) % 1.0
        if p < duty:                          # 지지: 앞→뒤로 밀기
            s = p / duty                      # 0→1
            x = step_x * (1 - 2 * s)
            y = step_y * (1 - 2 * s)
            z = -BODY_HEIGHT
        else:                                 # 유각: 뒤→앞 복귀 + 사이클로이드 들어올림
            s = (p - duty) / (1 - duty)
            x = step_x * (2 * s - 1)
            y = step_y * (2 * s - 1)
            z = -BODY_HEIGHT + gait["swing_h"] * math.sin(math.pi * s)
        return x, SIDE[leg] * L_HIP + y, z

    def _step(self):
        if not self.enabled:   # 복구 중에는 fall_recovery가 관절을 점유
            return
        gait = GAITS[self.get_parameter("gait").value]
        moving = (abs(self.cmd.linear.x) + abs(self.cmd.linear.y)
                  + abs(self.cmd.angular.z)) > 1e-3

        if moving:
            self.active = True
            self.phase = (self.phase + self.dt / gait["period"]) % 1.0
        elif self.active:
            # 정지 명령: 위상을 0으로 되돌리고 기립 자세로 복귀
            self.active = False
            self.phase = 0.0

        angles = []
        for leg in LEGS:
            if self.active:
                x, y, z = self._foot_target(leg, gait, self.phase)
                q = leg_ik(x, y, z, SIDE[leg])
            else:
                q = STAND
            angles.extend(q)

        self.pub.publish(Float64MultiArray(data=angles))


def main():
    rclpy.init()
    node = GaitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
