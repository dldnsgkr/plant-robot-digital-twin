#!/usr/bin/env python3
"""통합 미션 오케스트레이터 — 순찰→점검→가스탐색→자율복귀 시나리오.

상태 흐름:
  GOTO_GAUGE   : 복도 게이지 관측점(patrol_planner 산출 지점)으로 A* 주행
  INSPECT      : 정지 후 12초간 게이지 판독 수집 → 결과 기록
  GOTO_FACTORY : 공장 내부로 이동
  SEEK         : /gas/alarm 발령 → source_seeker에 제어권 양보,
                 /gas/found 또는 타임아웃까지 대기
  WAIT_RTH     : 배터리 소모로 rth가 복귀·도킹할 때까지 대기
  DONE

관제 연동: /mission/state(String), /mission/event(String) 발행.
seeker(/rth_active 구독)·rth와 같은 cmd_vel 조정 규약을 따른다:
SEEK/WAIT_RTH 상태에서는 미션이 cmd_vel을 내지 않는다.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String

sys.path.insert(0, __file__.rsplit("/", 2)[0]
                + "/module3_gas_safety/return_to_home")
from rth import astar  # noqa: E402  (월드 기하·A* 공유)

GAUGE_VP = (38.0, 0.22)     # patrol_planner가 산출한 게이지 관측점
FACTORY_POINT = (5.0, -2.0)
V_WALK = 0.22


class Mission(Node):
    def __init__(self):
        super().__init__("mission_controller")
        self.state = "GOTO_GAUGE"
        self.pose = None
        self.path = None
        self.wp_i = 0
        self.t_state = 0.0
        self.gauge_readings = []
        self.found = False
        self.docked = False
        self.rth_active = False

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_state = self.create_publisher(String, "/mission/state", 5)
        self.pub_event = self.create_publisher(String, "/mission/event", 5)
        self.pub_alarm = self.create_publisher(Bool, "/gas/alarm", 1)
        self.create_subscription(Odometry, "/model/go2/odometry", self._on_odom, 10)
        self.create_subscription(Float32, "/inspection/gauge_value", self._on_gauge, 5)
        self.create_subscription(Bool, "/gas/found", self._on_found, 1)
        self.create_subscription(Bool, "/robot/docked", self._on_dock, 1)
        self.create_subscription(Bool, "/rth_active", self._on_rth, 1)
        self.create_timer(0.1, self._step)
        self.create_timer(1.0, lambda: self.pub_state.publish(
            String(data=self.state)))
        self._event("미션 시작: 게이지 점검 지점으로 이동")

    def _event(self, text):
        self.get_logger().info(text)
        self.pub_event.publish(String(data=text))

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def _on_gauge(self, msg):
        if self.state == "INSPECT":
            self.gauge_readings.append(msg.data)

    def _on_found(self, msg):
        self.found = self.found or msg.data

    def _on_dock(self, msg):
        self.docked = self.docked or msg.data

    def _on_rth(self, msg):
        self.rth_active = msg.data

    # ---- 주행 (rth와 동일한 축 정렬 게걸음 규약) ----
    def _drive_to(self, goal):
        """goal로 주행. 도착 시 True."""
        x, y, yaw = self.pose
        if math.hypot(goal[0] - x, goal[1] - y) < 0.6:
            self.pub_cmd.publish(Twist())
            self.path = None
            return True
        if self.path is None:
            self.path = astar((x, y), goal)
            self.wp_i = 0
            if self.path is None:
                self._event("경로 계획 실패!")
                return False
        while self.wp_i < len(self.path) - 1 and \
                math.hypot(self.path[self.wp_i][0] - x,
                           self.path[self.wp_i][1] - y) < 0.5:
            self.wp_i += 1
        tx, ty = self.path[self.wp_i]
        dx, dy = tx - x, ty - y
        # 헤딩 조향: 목표 방위로 회전하며 전진
        target = math.atan2(dy, dx)
        err = math.atan2(math.sin(target - yaw), math.cos(target - yaw))
        cmd = Twist()
        cmd.linear.x = V_WALK * max(0.0, math.cos(err))
        cmd.angular.z = max(-0.4, min(0.4, 1.0 * err))
        self.pub_cmd.publish(cmd)
        return False

    def _step(self):
        if self.pose is None:
            return
        self.t_state += 0.1

        if self.rth_active and self.state not in ("WAIT_RTH", "DONE"):
            self._event("자율 복귀 발동 — 미션 제어권 양보")
            self.state, self.t_state = "WAIT_RTH", 0.0
            return

        if self.state == "GOTO_GAUGE":
            if self._drive_to(GAUGE_VP):
                self._event("게이지 관측점 도착 — 게이지 방향 정렬")
                self.state, self.t_state = "ALIGN", 0.0

        elif self.state == "ALIGN":
            # 게이지(38, 1.575)를 바라보도록 제자리 회전
            target = math.atan2(1.575 - self.pose[1], 38.0 - self.pose[0])
            err = math.atan2(math.sin(target - self.pose[2]),
                             math.cos(target - self.pose[2]))
            if abs(err) < 0.12 or self.t_state > 20.0:
                self.pub_cmd.publish(Twist())
                self._event("정렬 완료 — 판독 시작")
                self.gauge_readings = []
                self.state, self.t_state = "INSPECT", 0.0
            else:
                cmd = Twist()
                cmd.angular.z = max(-0.35, min(0.35, 1.0 * err))
                self.pub_cmd.publish(cmd)

        elif self.state == "INSPECT":
            if self.t_state > 12.0:
                if self.gauge_readings:
                    med = sorted(self.gauge_readings)[len(self.gauge_readings) // 2]
                    self._event("게이지 판독 완료: %.2f bar (%d건)"
                                % (med, len(self.gauge_readings)))
                else:
                    self._event("게이지 판독 실패 (수신 0건)")
                self._event("공장 구역으로 이동")
                self.state, self.t_state = "GOTO_FACTORY", 0.0

        elif self.state == "GOTO_FACTORY":
            if self._drive_to(FACTORY_POINT):
                self._event("공장 도착 — 가스 알람 발령, 누출원 탐색 위임")
                self.state, self.t_state = "SEEK", 0.0

        elif self.state == "SEEK":
            self.pub_alarm.publish(Bool(data=True))
            if self.found:
                self._event("누출원 탐지 보고 수신")
                self.state, self.t_state = "WAIT_RTH", 0.0
            elif self.t_state > 150.0:
                self._event("탐색 타임아웃 — 복귀 대기로 전환")
                self.state, self.t_state = "WAIT_RTH", 0.0

        elif self.state == "WAIT_RTH":
            if self.docked:
                self._event("도킹 완료 — 미션 종료")
                self.state = "DONE"


def main():
    rclpy.init()
    node = Mission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
