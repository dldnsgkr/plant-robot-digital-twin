#!/usr/bin/env python3
"""아날로그 게이지 판독 노드 (OCR/Computer Vision).

파이프라인:
  1. 전처리: CLAHE(저조도 대비 향상) + 가우시안 블러(노이즈 완화)
  2. 다이얼 검출: HoughCircles — 가장 큰 원을 게이지로 선택
  3. 바늘 검출: 원 내부에서 (a) 빨강 HSV 마스크 → 실패 시 (b) 어두운 픽셀 마스크,
     중심 기준 방사 픽셀들의 각도 히스토그램 최빈 방향
  4. 각도→수치: v = (225° - θ)/27  (make_dial.py 규약, θ는 이미지 오른쪽 기준 CCW)
  5. 시간 필터: 최근 5프레임 중앙값 → 흔들림 순간 오독 제거

토픽:
  입력  /front_camera (Image) — image_degrader 사용 시 /front_camera/degraded
  출력  /inspection/gauge_value (Float32), /inspection/gauge_overlay (Image)
"""
import math
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class GaugeReader(Node):
    def __init__(self):
        super().__init__("gauge_reader")
        self.declare_parameter("input_topic", "/front_camera")
        self.history = deque(maxlen=5)

        topic = self.get_parameter("input_topic").value
        self.pub_val = self.create_publisher(Float32, "/inspection/gauge_value", 10)
        self.pub_img = self.create_publisher(Image, "/inspection/gauge_overlay", 2)
        self.create_subscription(Image, topic, self._on_image, 2)
        self.get_logger().info("gauge_reader 시작 (입력: %s)" % topic)

    # ---------- CV 파이프라인 ----------
    @staticmethod
    def detect_dial(gray):
        """가장 큰 원(게이지 다이얼) 검출 → (cx, cy, r) 또는 None."""
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=200,
            param1=120, param2=60, minRadius=40, maxRadius=400)
        if circles is None:
            return None
        cx, cy, r = max(circles[0], key=lambda c: c[2])
        return int(cx), int(cy), int(r)

    @staticmethod
    def needle_angle(bgr, cx, cy, r):
        """바늘 방향(도, 이미지 오른쪽 기준 CCW) 또는 None."""
        h, w = bgr.shape[:2]
        y0, y1 = max(0, cy - r), min(h, cy + r)
        x0, x1 = max(0, cx - r), min(w, cx + r)
        roi = bgr[y0:y1, x0:x1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        # (a) 빨강 바늘 (저조도에서도 Hue는 유지됨 — S/V 하한을 낮게)
        m1 = cv2.inRange(hsv, (0, 60, 40), (12, 255, 255))
        m2 = cv2.inRange(hsv, (168, 60, 40), (180, 255, 255))
        mask = m1 | m2
        if mask.sum() < 40 * 255:
            # (b) 폴백: 중심부 어두운 픽셀 (테두리·눈금 제외 반경 0.75r)
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            mask = cv2.inRange(gray, 0, 70)
        ys, xs = np.nonzero(mask)
        if len(xs) < 30:
            return None
        dx = xs - (cx - x0)
        dy = ys - (cy - y0)
        rad = np.hypot(dx, dy)
        keep = (rad > 0.15 * r) & (rad < 0.85 * r)   # 중심축·외곽 링 제외
        if keep.sum() < 20:
            return None
        ang = np.degrees(np.arctan2(-dy[keep], dx[keep]))  # 이미지 y 반전
        # 각도 히스토그램 최빈 방향 (바늘은 한쪽으로만 뻗음)
        hist, edges = np.histogram(ang, bins=72, range=(-180, 180),
                                   weights=rad[keep])
        i = int(hist.argmax())
        sel = (ang >= edges[i] - 10) & (ang <= edges[i] + 15)
        return float(np.average(ang[sel], weights=rad[keep][sel]))

    @staticmethod
    def angle_to_value(theta):
        """θ(도) → bar. 0bar=225°, 10bar=-45° (시계방향 270° 스윕)."""
        if theta < -90:               # -180~-90 구간은 225~315°로 랩
            theta += 360
        return (225.0 - theta) / 27.0

    # ---------- ROS ----------
    def _on_image(self, msg):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(gray)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        dial = self.detect_dial(gray)
        value = None
        if dial:
            cx, cy, r = dial
            theta = self.needle_angle(bgr, cx, cy, r)
            if theta is not None:
                v = self.angle_to_value(theta)
                if -0.3 <= v <= 10.3:
                    self.history.append(v)
                    value = float(np.median(self.history))

        if value is not None:
            self.pub_val.publish(Float32(data=value))

        # 오버레이 (관제 화면·데모용)
        if dial:
            cv2.circle(bgr, (cx, cy), r, (0, 255, 0), 2)
            label = "%.2f bar" % value if value is not None else "READ FAIL"
            cv2.putText(bgr, label, (cx - r, cy - r - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        out = Image(height=bgr.shape[0], width=bgr.shape[1], encoding="bgr8",
                    step=bgr.shape[1] * 3, data=bgr.tobytes())
        out.header = msg.header
        self.pub_img.publish(out)


def main():
    rclpy.init()
    node = GaugeReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
