#!/usr/bin/env python3
"""아날로그 압력계 눈금판 텍스처 생성 (0~10 bar, 270° 스윕).

게이지 규약 (gauge_reader와 공유):
  - 눈금 시작(0 bar): 시계 7시 30분 방향 = 텍스처 기준 225°
  - 눈금 끝(10 bar):  시계 4시 30분 방향 = -45° (즉 시계방향으로 270° 스윕)
  - 값 v[bar] 의 바늘 각도(텍스처 좌표, 반시계+): θ = 225° - 27°·v
"""
import math

import cv2
import numpy as np

S = 512
c = S // 2
img = np.full((S, S, 3), 245, np.uint8)          # 밝은 문자판
cv2.circle(img, (c, c), c - 4, (30, 30, 30), 6)  # 테두리

for v in range(0, 11):
    ang = math.radians(225 - 27 * v)
    ca, sa = math.cos(ang), math.sin(ang)
    # 주 눈금
    p1 = (int(c + ca * (c - 30)), int(c - sa * (c - 30)))
    p2 = (int(c + ca * (c - 70)), int(c - sa * (c - 70)))
    cv2.line(img, p1, p2, (20, 20, 20), 8)
    # 숫자
    pt = (int(c + ca * (c - 110)) - 22, int(c - sa * (c - 110)) + 18)
    cv2.putText(img, str(v), pt, cv2.FONT_HERSHEY_SIMPLEX, 1.6, (20, 20, 20), 4)
    # 보조 눈금
    if v < 10:
        for m in range(1, 5):
            a2 = math.radians(225 - 27 * (v + m / 5))
            q1 = (int(c + math.cos(a2) * (c - 30)), int(c - math.sin(a2) * (c - 30)))
            q2 = (int(c + math.cos(a2) * (c - 52)), int(c - math.sin(a2) * (c - 52)))
            cv2.line(img, q1, q2, (60, 60, 60), 3)

cv2.putText(img, "bar", (c - 35, c + 90), cv2.FONT_HERSHEY_SIMPLEX,
            1.4, (60, 60, 60), 3)
cv2.circle(img, (c, c), 14, (30, 30, 30), -1)    # 중심축

cv2.imwrite(__file__.rsplit("/", 1)[0] + "/dial.png", img)
print("dial.png 생성 완료 (512x512)")
