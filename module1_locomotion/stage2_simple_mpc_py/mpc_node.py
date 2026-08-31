#!/usr/bin/env python3
"""LIP-MPC 균형 제어 노드.

입력:
  /world/plant/dynamic_pose/info (TFMessage 브리지) → go2 베이스 포즈
  /imu                                              → 요(방위) 확인용
  /cmd_vel                                          → 기준 속도

처리 (20Hz):
  월드 포즈 → 유한차분+LPF 로 CoM 속도 추정 → 몸 좌표계 속도 오차
  → 축별 LIP-MPC 풀이 → CoP 오프셋(= 지지발 배치 오프셋)

출력:
  /mpc/cop_offset (Vector3, 몸 좌표계) — gait_controller가 지지발 목표에 가산
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from tf2_msgs.msg import TFMessage

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
        self.pose = None          # (t, x, y, yaw)
        self.vel = [0.0, 0.0]     # 월드 프레임 LPF 속도
        self.p_err = [0.0, 0.0]   # 속도 오차 적분(위치 오차 대용, 누수 적분)

        self.pub = self.create_publisher(Vector3, "/mpc/cop_offset", 10)
        self.create_subscription(
            TFMessage, "/world/plant/dynamic_pose/info", self._on_poses, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_timer(0.05, self._step)
        self.get_logger().info("lip_mpc 시작 (N=%d, dt=%.2f)" % (n, dt))

    def _on_cmd(self, msg):
        self.cmd = msg

    def _on_poses(self, msg):
        for tr in msg.transforms:
            if tr.child_frame_id != "go2":
                continue
            t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            q = tr.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            x, y = tr.transform.translation.x, tr.transform.translation.y
            if self.pose is not None and t > self.pose[0]:
                dt = t - self.pose[0]
                if dt < 0.5:
                    a = 0.25  # 속도 LPF
                    self.vel[0] += a * ((x - self.pose[1]) / dt - self.vel[0])
                    self.vel[1] += a * ((y - self.pose[2]) / dt - self.vel[1])
            self.pose = (t, x, y, yaw)
            return

    def _step(self):
        if self.pose is None:
            return
        yaw = self.pose[3]
        c, s = math.cos(yaw), math.sin(yaw)
        # 몸 좌표계 속도
        vx_b = c * self.vel[0] + s * self.vel[1]
        vy_b = -s * self.vel[0] + c * self.vel[1]
        ex_v = vx_b - self.cmd.linear.x
        ey_v = vy_b - self.cmd.linear.y
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
