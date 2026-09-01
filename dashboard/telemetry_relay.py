#!/usr/bin/env python3
"""관제 텔레메트리 릴레이 — 대시보드 2차 고도화 지원 노드.

역할:
  1. 라이브 피드: /front_camera(1280×720)·/thermal/colormap(320×240)을
     다운스케일 JPEG(2Hz)로 압축해 CompressedImage로 발행 — rosbridge가
     base64로 전달하므로 브라우저 <img>에 바로 표시 가능. 원본 영상은
     대역폭 설계상 관제로 보내지 않는다는 원칙을 유지(수 KB/s 썸네일만).
  2. 이벤트 히스토리: 미션 이벤트·알람 전이·운전원 확인(ack)을 누적하고
     전체 이력을 JSON으로 transient_local(latched) 발행 — 관제 화면을
     늦게 열어도 과거 이벤트가 그대로 복원된다.
  3. 알람 전이 기록: 가스>30ppm, 온도>60°C, 배터리<20% 의 on/off 전이를
     이력에 남긴다 (알람 이력 요건).
  4. /dashboard/ack (String: GAS|OVERHEAT|BATTERY) 수신 → '운전원 확인'
     이력 추가.
"""
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32, String

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)

ALARMS = {
    "GAS": ("/gas/concentration", lambda v: v > 30.0, "가스 농도"),
    "OVERHEAT": ("/inspection/max_temp", lambda v: v > 60.0, "설비 과열"),
    "BATTERY": ("/robot/battery", lambda v: v < 20.0, "배터리 부족"),
}


class TelemetryRelay(Node):
    def __init__(self):
        super().__init__("telemetry_relay")
        self.events = []
        self.alarm_on = {k: False for k in ALARMS}
        self.last_jpg = {"cam": 0.0, "thermal": 0.0}

        self.pub_cam = self.create_publisher(
            CompressedImage, "/dashboard/cam/compressed", 2)
        self.pub_th = self.create_publisher(
            CompressedImage, "/dashboard/thermal/compressed", 2)
        self.pub_ev = self.create_publisher(String, "/dashboard/events", LATCHED)

        self.create_subscription(Image, "/front_camera",
                                 lambda m: self._img(m, "cam"), 2)
        self.create_subscription(Image, "/thermal/colormap",
                                 lambda m: self._img(m, "thermal"), 2)
        self.create_subscription(String, "/mission/event",
                                 lambda m: self._add("info", m.data), 10)
        self.create_subscription(Bool, "/gas/found",
                                 lambda m: m.data and self._once(
                                     "found", "alarm", "가스 누출원 발견 보고"), 5)
        self.create_subscription(Bool, "/robot/docked",
                                 lambda m: m.data and self._once(
                                     "dock", "info", "충전 스테이션 도킹 완료"), 5)
        for key, (topic, _cond, _label) in ALARMS.items():
            self.create_subscription(
                Float32, topic,
                lambda m, k=key: self._alarm(k, m.data), 10)
        self.create_subscription(String, "/dashboard/ack", self._ack, 5)

        self._add("info", "관제 텔레메트리 릴레이 기동")
        self.get_logger().info("telemetry_relay 시작")

    # ---- 이벤트 이력 ----
    def _add(self, typ, text):
        self.events.append({
            "t": time.strftime("%H:%M:%S"), "type": typ, "text": text})
        self.events = self.events[-80:]
        self.pub_ev.publish(String(data=json.dumps(
            self.events, ensure_ascii=False)))

    def _once(self, flag, typ, text):
        if not getattr(self, "_f_" + flag, False):
            setattr(self, "_f_" + flag, True)
            self._add(typ, text)

    def _alarm(self, key, value):
        on = ALARMS[key][1](value)
        if on != self.alarm_on[key]:
            self.alarm_on[key] = on
            label = ALARMS[key][2]
            if on:
                self._add("alarm", "⚠ 알람 발생: %s (%.1f)" % (label, value))
                self.get_logger().warn("알람 발생: %s (%.1f)" % (label, value))
            else:
                self._add("info", "알람 해제: %s" % label)

    def _ack(self, msg):
        self._add("ack", "✓ 운전원 확인(ACK): %s" % msg.data)

    # ---- 영상 썸네일 ----
    def _img(self, msg, kind):
        now = time.time()
        if now - self.last_jpg[kind] < 0.5:          # 2Hz 제한
            return
        self.last_jpg[kind] = now
        img = np.frombuffer(msg.data, np.uint8).reshape(
            msg.height, msg.width, -1)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if kind == "cam":
            img = cv2.resize(img, (384, 216), interpolation=cv2.INTER_AREA)
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 60])
        if not ok:
            return
        out = CompressedImage(format="jpeg", data=jpg.tobytes())
        out.header = msg.header
        (self.pub_cam if kind == "cam" else self.pub_th).publish(out)


def main():
    rclpy.init()
    node = TelemetryRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
