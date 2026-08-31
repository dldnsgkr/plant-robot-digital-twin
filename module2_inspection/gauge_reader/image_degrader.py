#!/usr/bin/env python3
"""카메라 열화 주입 노드 — 저조도 + 모션블러 + 센서 노이즈 재현.

Gazebo 렌더링은 항상 깨끗하므로, 실환경 제약(이동 중 흔들림, 어두운 플랜트
내부)을 후처리로 주입해 인식 파이프라인의 강건성을 검증한다.

/front_camera → [감마·게인 저조도] → [방향성 모션블러] → [가우시안 노이즈]
             → /front_camera/degraded
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Degrader(Node):
    def __init__(self):
        super().__init__("image_degrader")
        self.declare_parameter("darkness", 0.35)   # 0=원본, 1=완전 암전
        self.declare_parameter("blur_len", 9)      # 모션블러 커널 길이(px)
        self.declare_parameter("noise_std", 8.0)   # 가우시안 노이즈 σ
        self.t = 0
        self.pub = self.create_publisher(Image, "/front_camera/degraded", 2)
        self.create_subscription(Image, "/front_camera", self._cb, 2)
        self.get_logger().info("image_degrader 시작")

    def _cb(self, msg):
        img = np.frombuffer(msg.data, np.uint8).reshape(
            msg.height, msg.width, -1).astype(np.float32)
        self.t += 1

        # 저조도: 게인 감소 + 감마 (어두운 영역 디테일 손실 재현)
        d = self.get_parameter("darkness").value
        img = (img / 255.0) ** (1 + d * 1.5) * 255.0 * (1 - d)

        # 모션블러: 보행 진동을 흉내내 방향이 매 프레임 회전
        klen = int(self.get_parameter("blur_len").value)
        if klen >= 3:
            k = np.zeros((klen, klen), np.float32)
            ang = (self.t * 37) % 180
            c = klen // 2
            dx, dy = np.cos(np.radians(ang)), np.sin(np.radians(ang))
            for i in range(klen):
                x = int(c + (i - c) * dx)
                y = int(c + (i - c) * dy)
                k[np.clip(y, 0, klen - 1), np.clip(x, 0, klen - 1)] = 1
            img = cv2.filter2D(img, -1, k / max(k.sum(), 1))

        # 센서 노이즈
        std = self.get_parameter("noise_std").value
        img = img + np.random.normal(0, std, img.shape)

        out_arr = np.clip(img, 0, 255).astype(np.uint8)
        out = Image(height=msg.height, width=msg.width, encoding=msg.encoding,
                    step=msg.step, data=out_arr.tobytes())
        out.header = msg.header
        self.pub.publish(out)


def main():
    rclpy.init()
    node = Degrader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
