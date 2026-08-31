#!/usr/bin/env python3
"""/front_camera 이미지 1장을 파일로 저장 (검증 헬퍼).

사용: python3 save_camera.py <출력경로.png> [토픽]
"""
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Saver(Node):
    def __init__(self, out, topic):
        super().__init__("cam_saver")
        self.out = out
        self.done = False
        self.create_subscription(Image, topic, self.cb, 1)

    def cb(self, msg):
        if self.done:
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding in ("rgb8",):
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imwrite(self.out, img)
        print("저장:", self.out, img.shape)
        self.done = True


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/cam.png"
    topic = sys.argv[2] if len(sys.argv) > 2 else "/front_camera"
    rclpy.init()
    n = Saver(out, topic)
    import time
    t0 = time.time()
    while not n.done and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.5)


if __name__ == "__main__":
    main()
