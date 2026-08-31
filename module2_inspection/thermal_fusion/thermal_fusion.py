#!/usr/bin/env python3
"""열화상-RGB 퓨전 노드 — 외부 파라미터 기반 픽셀 정합 + 과열 시각화.

두 카메라는 장착 위치가 다르다 (RGB: base+(0.335, 0, 0.03),
열화상(=depth 기하): base+(0.335, +0.03, 0.03) — 3cm 횡방향 시차).

정합(Registration) 수학:
  열화상 픽셀 (u,v) + 그 픽셀의 깊이 d
  → 열화상 카메라 광학좌표 3D 점 p_t = ((u-cx)/fx·d, (v-cy)/fy·d, d)
  → RGB 카메라 좌표로 변환: p_r = p_t + t,  t = (+0.03, 0, 0) (광학 x=오른쪽,
     열화상 카메라가 왼쪽에 있으므로 점은 RGB 프레임에서 오른쪽으로 이동)
  → RGB 픽셀로 재투영: u' = fx_r·x/z + cx_r
  단순 호모그래피(평면 가정)와 달리 픽셀별 깊이를 쓰므로 근거리·원거리가
  동시에 정확하다 (시차는 깊이에 반비례).

출력: /inspection/thermal_fused (bgr8) — RGB 위에 과열 영역 오버레이
      /inspection/max_temp (Float32), 60°C 초과 시 OVERHEAT 라벨
"""
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

# 열화상(depth 기하, 원본 640×480을 320×240로 낮춘 것)
TW, TH = 320, 240
T_FX = TW / (2 * math.tan(1.396 / 2))
# RGB 1280×720
RW, RH = 1280, 720
R_FX = RW / (2 * math.tan(1.396 / 2))
BASELINE = 0.03      # 열화상이 RGB보다 왼쪽으로 3cm
OVERHEAT_C = 60.0


class ThermalFusion(Node):
    def __init__(self):
        super().__init__("thermal_fusion")
        self.thermal = None
        self.depth = None
        self.pub_img = self.create_publisher(Image, "/inspection/thermal_fused", 2)
        self.pub_max = self.create_publisher(Float32, "/inspection/max_temp", 10)
        self.create_subscription(Image, "/thermal/image", self._on_thermal, 2)
        self.create_subscription(Image, "/depth_camera", self._on_depth, 2)
        self.create_subscription(Image, "/front_camera", self._on_rgb, 2)
        self.get_logger().info("thermal_fusion 시작")

    def _on_thermal(self, msg):
        self.thermal = np.frombuffer(msg.data, np.uint16).reshape(
            msg.height, msg.width).astype(np.float32) / 100.0

    def _on_depth(self, msg):
        self.depth = np.frombuffer(msg.data, np.float32).reshape(
            msg.height, msg.width)

    def _on_rgb(self, msg):
        if self.thermal is None or self.depth is None:
            return
        rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        # --- 정합: 열화상 각 픽셀을 깊이 기반으로 RGB 픽셀에 매핑 ---
        d = cv2.resize(self.depth, (TW, TH), interpolation=cv2.INTER_NEAREST)
        d = np.where(np.isfinite(d) & (d > 0.05), d, np.nan)
        u = (np.arange(TW) - TW / 2)
        v = (np.arange(TH) - TH / 2)
        uu, vv = np.meshgrid(u, v)
        x = uu / T_FX * d + BASELINE          # RGB 프레임으로 평행이동
        y = vv / T_FX * d
        ur = (x / d * R_FX + RW / 2)
        vr = (y / d * R_FX + RH / 2)

        heat = np.zeros((RH, RW), np.float32)
        valid = np.isfinite(ur) & np.isfinite(vr) & \
            (ur >= 0) & (ur < RW - 1) & (vr >= 0) & (vr < RH - 1)
        ui = ur[valid].astype(int)
        vi = vr[valid].astype(int)
        np.maximum.at(heat, (vi, ui), self.thermal[valid])
        # 저해상도 → 고해상도 매핑의 구멍 메움
        heat = cv2.dilate(heat, np.ones((5, 5), np.uint8))
        heat = cv2.GaussianBlur(heat, (9, 9), 0)

        # --- 과열 오버레이 ---
        hot = heat > 40.0
        alpha = np.clip((heat - 40.0) / 40.0, 0, 0.65)[..., None]
        cmap = cv2.applyColorMap(
            np.clip((heat - 20) / 80 * 255, 0, 255).astype(np.uint8),
            cv2.COLORMAP_INFERNO)
        fused = (bgr * (1 - alpha) + cmap * alpha).astype(np.uint8)

        max_t = float(np.nanmax(heat)) if hot.any() else \
            float(self.thermal[np.isfinite(self.thermal)].max(initial=25.0))
        self.pub_max.publish(Float32(data=max_t))
        if max_t > OVERHEAT_C:
            ys, xs = np.nonzero(heat > OVERHEAT_C)
            if len(xs):
                x0, x1 = xs.min(), xs.max()
                y0, y1 = ys.min(), ys.max()
                cv2.rectangle(fused, (x0 - 8, y0 - 8), (x1 + 8, y1 + 8),
                              (0, 0, 255), 3)
                cv2.putText(fused, "OVERHEAT %.0fC" % max_t,
                            (x0 - 8, max(20, y0 - 16)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        out = Image(height=RH, width=RW, encoding="bgr8", step=RW * 3,
                    data=fused.tobytes())
        out.header = msg.header
        self.pub_img.publish(out)


def main():
    rclpy.init()
    node = ThermalFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
