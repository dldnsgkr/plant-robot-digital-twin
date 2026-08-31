#!/usr/bin/env python3
"""열화상 카메라 시뮬레이터 — Depth 기하 기반 온도 영상 합성.

Gazebo에는 열화상 센서가 없으므로(계획서 §2.2 Gazebo 경로 추가분),
Depth 카메라의 픽셀별 3D 점을 월드 좌표로 복원해 발열원과의 거리로
온도를 합성한다. 픽셀 단위 기하를 쓰므로 가림(occlusion)이 자연스럽게
반영된다 — 발열원 앞에 장애물이 있으면 그 픽셀의 3D 점이 발열원과 멀어
뜨겁게 나오지 않는다.

카메라 장착 (URDF 실측): depth = base + (0.335, +0.03, 0.03)
출력: /thermal/image (mono16, 온도×100, 320×240 — 실제 열화상의 저해상도 재현)
      /thermal/colormap (bgr8, 시각화용)
"""
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

# 발열원: (x, y, z, ΔT[°C], σ[m])  — 공장기계 모터/배관 이음매
HOTSPOTS = [
    (-7.1, -6.0, 0.6, 50.0, 0.30),   # 모터 (75°C)
    (-7.5, -5.4, 1.5, 65.0, 0.25),   # 배관 이음매 (90°C, 과열!)
]
AMBIENT = 25.0

W, H = 640, 480
HFOV = 1.396
FX = W / (2 * math.tan(HFOV / 2))
CX, CY = W / 2, H / 2
CAM_OFF = (0.335, 0.03, 0.03)        # base → depth 카메라


class ThermalSim(Node):
    def __init__(self):
        super().__init__("thermal_camera_sim")
        self.base = None  # (x, y, z, yaw)
        self.pub16 = self.create_publisher(Image, "/thermal/image", 2)
        self.pub_c = self.create_publisher(Image, "/thermal/colormap", 2)
        self.create_subscription(Image, "/depth_camera", self._on_depth, 2)
        self.create_subscription(Odometry, "/model/go2/odometry", self._on_odom, 10)
        self.get_logger().info("thermal_camera_sim 시작 (발열원 %d개)" % len(HOTSPOTS))

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        self.base = (p.x, p.y, p.z, yaw)

    def _on_depth(self, msg):
        if not getattr(self, "_logged", False):
            self._logged = True
            self.get_logger().info("depth 수신 시작 (%dx%d, base=%s)"
                                   % (msg.width, msg.height, self.base is not None))
        if self.base is None:
            return
        depth = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        # 픽셀 → 카메라 몸체 좌표 (x전방, y좌, z상)
        u = np.arange(W) - CX
        v = np.arange(H) - CY
        uu, vv = np.meshgrid(u, v)
        d = np.where(np.isfinite(depth), depth, 1e3)
        fwd = d
        left = -uu / FX * d
        up = -vv / FX * d
        # 월드 좌표 (기립 가정: 롤/피치 무시)
        bx, by, bz, yaw = self.base
        cy_, sy = math.cos(yaw), math.sin(yaw)
        ox = bx + CAM_OFF[0] * cy_ - CAM_OFF[1] * sy
        oy = by + CAM_OFF[0] * sy + CAM_OFF[1] * cy_
        oz = bz + CAM_OFF[2]
        wx = ox + fwd * cy_ - left * sy
        wy = oy + fwd * sy + left * cy_
        wz = oz + up

        temp = np.full((H, W), AMBIENT, np.float32)
        for hx, hy, hz, dt, sig in HOTSPOTS:
            r2 = (wx - hx) ** 2 + (wy - hy) ** 2 + (wz - hz) ** 2
            temp += dt * np.exp(-r2 / (2 * sig * sig))
        temp += np.random.normal(0, 0.4, temp.shape)   # 열화상 센서 노이즈

        # 320×240 저해상도 (실제 열화상 재현)
        t_small = cv2.resize(temp, (320, 240), interpolation=cv2.INTER_AREA)
        raw = (np.clip(t_small, 0, 300) * 100).astype(np.uint16)
        out = Image(height=240, width=320, encoding="mono16", step=640,
                    data=raw.tobytes())
        out.header = msg.header
        self.pub16.publish(out)

        vis = cv2.applyColorMap(
            np.clip((t_small - 20) / 80 * 255, 0, 255).astype(np.uint8),
            cv2.COLORMAP_INFERNO)
        outc = Image(height=240, width=320, encoding="bgr8", step=960,
                     data=vis.tobytes())
        outc.header = msg.header
        self.pub_c.publish(outc)


def main():
    rclpy.init()
    node = ThermalSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
