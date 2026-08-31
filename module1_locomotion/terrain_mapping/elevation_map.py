#!/usr/bin/env python3
"""로봇 중심 Elevation Map 생성 + 발 디딤 위치(foothold) 평가 노드.

입력:  /lidar/points (PointCloud2, base 프레임) + /imu (자세 보정)
출력:  /elevation_map      (OccupancyGrid — Foxglove/RViz 시각화용, 0~0.5m → 0~100)
       /elevation_map/raw  (Float32MultiArray — 게이트 생성기가 발 높이 조정에 사용)
       /footholds          (Float32MultiArray — 다리별 권장 디딤 위치 [x,y,z]×4)

파이프라인:
  1. 포인트클라우드 → IMU 롤/피치로 중력 정렬 (기울어진 몸에서도 지면 기준 높이)
  2. 4×4m, 5cm 격자에 셀별 최대 높이 기록 (max: 장애물 상단이 디딤 기준)
  3. 노이즈 필터: 시간 EMA(α=0.4) + 관측 수 2회 미만 셀 무시
  4. foothold 평가: 다리별 명목 착지점 주변 15cm에서
     '평탄도(주변 셀 높이 분산) + 명목점과의 거리' 비용 최소 셀 선택
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray

MAP_SIZE = 4.0     # m (로봇 중심 정사각)
RES = 0.05         # m/셀
N = int(MAP_SIZE / RES)  # 80
BODY_HEIGHT = 0.30
LIDAR_OFFSET = np.array([0.15, 0.0, 0.1])

# 명목 발 착지점 (base 기준): 힙 바로 아래
NOMINAL_FEET = {
    "FL": (+0.1934, +0.142), "FR": (+0.1934, -0.142),
    "RL": (-0.1934, +0.142), "RR": (-0.1934, -0.142),
}


class ElevationMap(Node):
    def __init__(self):
        super().__init__("elevation_map")
        self.height = np.zeros((N, N), dtype=np.float32)   # 지면 기준 높이
        self.count = np.zeros((N, N), dtype=np.int32)
        self.roll = 0.0
        self.pitch = 0.0

        self.pub_grid = self.create_publisher(OccupancyGrid, "/elevation_map", 1)
        self.pub_raw = self.create_publisher(Float32MultiArray, "/elevation_map/raw", 1)
        self.pub_feet = self.create_publisher(Float32MultiArray, "/footholds", 1)
        self.create_subscription(PointCloud2, "/lidar/points", self._on_cloud, 1)
        self.create_subscription(Imu, "/imu", self._on_imu, 10)
        self.create_timer(0.5, self._publish)
        self.get_logger().info("elevation_map 시작 (%.0f×%.0fm, %.0fcm 격자)"
                               % (MAP_SIZE, MAP_SIZE, RES * 100))

    def _on_imu(self, msg):
        q = msg.orientation
        self.roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                               1 - 2 * (q.x * q.x + q.y * q.y))
        s = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))
        self.pitch = math.asin(s)

    def _on_cloud(self, msg):
        pts = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True)
        if len(pts) == 0:
            return
        pts = pts[np.isfinite(pts).all(axis=1)]  # 무반사(inf) 빔 제거
        if len(pts) == 0:
            return
        pts = pts + LIDAR_OFFSET

        # 중력 정렬: 롤/피치 역회전 (yaw는 로봇 중심 맵이므로 불필요)
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        pts = pts @ (ry @ rx).T

        # 자기 몸 마스킹: 몸통·스윙 중인 다리가 장애물로 잡히면
        # '발 들기 → 자기 발 감지 → 더 들기' 양성 피드백이 생긴다
        self_mask = (np.abs(pts[:, 0]) < 0.45) & (np.abs(pts[:, 1]) < 0.30)
        pts = pts[~self_mask]
        if len(pts) == 0:
            return

        # 지면 기준 높이 (base가 지면에서 BODY_HEIGHT 위에 있다고 가정)
        heights = pts[:, 2] + BODY_HEIGHT
        # 관심 범위 밖(천장 등) 제거
        keep = (heights > -0.3) & (heights < 1.5)
        pts, heights = pts[keep], heights[keep]

        ix = ((pts[:, 0] + MAP_SIZE / 2) / RES).astype(int)
        iy = ((pts[:, 1] + MAP_SIZE / 2) / RES).astype(int)
        keep = (ix >= 0) & (ix < N) & (iy >= 0) & (iy < N)
        ix, iy, heights = ix[keep], iy[keep], heights[keep]

        # 셀별 최대 높이 → 시간 EMA로 융합 (센서 노이즈 완화)
        frame = np.full((N, N), -np.inf, dtype=np.float32)
        np.maximum.at(frame, (ix, iy), heights)
        seen = frame > -np.inf
        alpha = 0.4
        first = seen & (self.count == 0)
        self.height[first] = frame[first]
        upd = seen & (self.count > 0)
        self.height[upd] = (1 - alpha) * self.height[upd] + alpha * frame[upd]
        self.count[seen] = np.minimum(self.count[seen] + 1, 8)  # 감쇠 대비 상한

    def _flatness_cost(self, cx, cy):
        """셀 주변 3×3 높이 분산 (작을수록 평탄)."""
        x0, x1 = max(0, cx - 1), min(N, cx + 2)
        y0, y1 = max(0, cy - 1), min(N, cy + 2)
        patch = self.height[x0:x1, y0:y1]
        valid = self.count[x0:x1, y0:y1] >= 2
        if valid.sum() < 3:
            return 1.0  # 정보 부족 — 회피
        return float(np.var(patch[valid]))

    def _publish(self):
        # 로봇 중심 맵은 이동 시 구식 데이터가 어긋나므로 관측 카운트를 감쇠시켜
        # 최근 관측이 없는 셀을 1~2초 내 무효화한다
        self.count = np.maximum(self.count - 1, 0)
        valid = self.count >= 2

        grid = OccupancyGrid()
        grid.header.frame_id = "base"
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = RES
        grid.info.width = N
        grid.info.height = N
        grid.info.origin.position.x = -MAP_SIZE / 2
        grid.info.origin.position.y = -MAP_SIZE / 2
        data = np.full((N, N), -1, dtype=np.int8)
        h = np.clip(self.height / 0.5 * 100, 0, 100).astype(np.int8)
        data[valid] = h[valid]
        # OccupancyGrid는 row-major (y가 행)
        grid.data = data.T.flatten().tolist()
        self.pub_grid.publish(grid)

        raw = Float32MultiArray()
        raw.data = np.where(valid, self.height, np.nan).flatten().tolist()
        self.pub_raw.publish(raw)

        # 다리별 foothold: 명목점 주변 15cm 탐색, 비용 = 분산 + 0.5×거리²
        feet = []
        for leg, (nx, ny) in NOMINAL_FEET.items():
            cx = int((nx + MAP_SIZE / 2) / RES)
            cy = int((ny + MAP_SIZE / 2) / RES)
            best = (nx, ny, 0.0)
            best_cost = float("inf")
            r = int(0.15 / RES)
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    x, y = cx + dx, cy + dy
                    if not (0 <= x < N and 0 <= y < N) or not valid[x, y]:
                        continue
                    dist2 = (dx * RES) ** 2 + (dy * RES) ** 2
                    cost = self._flatness_cost(x, y) + 0.5 * dist2
                    if cost < best_cost:
                        best_cost = cost
                        best = (x * RES - MAP_SIZE / 2,
                                y * RES - MAP_SIZE / 2,
                                float(self.height[x, y]))
            feet.extend(best)
        self.pub_feet.publish(Float32MultiArray(data=feet))


def main():
    rclpy.init()
    node = ElevationMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
