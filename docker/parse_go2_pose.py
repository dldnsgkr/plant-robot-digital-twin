#!/usr/bin/env python3
"""gz dynamic_pose/info 출력에서 go2 베이스 포즈를 추출 (테스트 헬퍼)."""
import re
import sys

t = sys.stdin.read()
m = re.search(r'name: .go2.\s*.*?position \{(.*?)\}\s*orientation \{(.*?)\}', t, re.S)
if not m:
    print("no pose")
    sys.exit(1)
pos = dict(re.findall(r'(\w): ([-\d.e]+)', m.group(1)))
ori = dict(re.findall(r'(\w): ([-\d.e]+)', m.group(2)))
print("x=%.2f y=%.2f z=%.2f qx=%.2f qy=%.2f" % (
    float(pos.get('x', 0)), float(pos.get('y', 0)), float(pos.get('z', 0)),
    float(ori.get('x', 0)), float(ori.get('y', 0))))
