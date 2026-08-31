#!/usr/bin/env python3
"""LIP-MPC 균형 제어 노드.

입력:
  /model/go2/odometry (OdometryPublisher 플러그인 브리지) → 포즈 + 몸좌표 속도
  /cmd_vel                                              → 기준 속도

처리 (20Hz):
  몸 좌표계 속도 오차(+누수 적분 위치 오차)
  → 축별 LIP-MPC 풀이 → CoP 오프셋(= 지지발 배치 오프셋)

출력:
  /mpc/cop_offset (Vector3, 몸 좌표계) — gait_controller가 지지발 목표에 가산
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

from srbd_mpc import LipMpc


class MpcNode(Node):
    def __init__(self):
        super().__init__("lip_mpc")
        self.declare_parameter("horizon_n", 10)   # TUNING.md 실험으로 선정
        self.declare_parameter("dt", 0.1)

        n = self.get_parameter("horizon_n").value
        dt = self.get_parameter("dt").value
        self.mpc_x = LipMpc(horizon_n=n, dt=dt, u_max=0.10)
        self.mpc_y = LipMpc(horizon_n=n, dt=dt, u_max=0.07)

        self.cmd = Twist()
        self.vel_b = None         # 몸 좌표계 속도 (Odometry twist, LPF)
        self.p_err = [0.0, 0.0]   # 속도 오차 적분(위치 오차 대용, 누수 적분)

        self.pub = self.create_publisher(Vector3, "/mpc/cop_offset", 10)
        self.create_subscription(Odometry, "/model/go2/odometry", self._on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_timer(0.05, self._step)
        self.get_logger().info("lip_mpc 시작 (N=%d, dt=%.2f)" % (n, dt))

    def _on_cmd(self, msg):
        self.cmd = msg

    def _on_odom(self, msg):
        # Odometry twist는 child(몸) 좌표계 — 그대로 사용, 가벼운 LPF만
        vx, vy = msg.twist.twist.linear.x, msg.twist.twist.linear.y
        if self.vel_b is None:
            self.vel_b = [vx, vy]
        else:
            a = 0.35
            self.vel_b[0] += a * (vx - self.vel_b[0])
            self.vel_b[1] += a * (vy - self.vel_b[1])

    def _step(self):
        if self.vel_b is None:
            return
        ex_v = self.vel_b[0] - self.cmd.linear.x
        ey_v = self.vel_b[1] - self.cmd.linear.y
        # 위치 오차 대용: 속도 오차의 누수 적분 (기준 경로 없이 드리프트 방지)
        leak = 0.95
        self.p_err[0] = leak * self.p_err[0] + ex_v * 0.05
        self.p_err[1] = leak * self.p_err[1] + ey_v * 0.05

        ux = self.mpc_x.solve(self.p_err[0], ex_v)
        uy = self.mpc_y.solve(self.p_err[1], ey_v)
        self.pub.publish(Vector3(x=ux, y=uy, z=0.0))


def main():
    rclpy.init()
    node = MpcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
