#!/usr/bin/env python3
"""구역 순찰 노드 (보너스②) — 지정 waypoint 루프를 A* 경로로 반복 순찰.

파라미터:
  waypoints: [x1, y1, x2, y2, ...]  순찰 지점 (도착 순서대로 순환)
  cmd_topic / odom_topic: 로봇별 토픽 (다중 로봇 네임스페이스 분리)

각 구간은 rth의 A*(월드 기하 공유)로 계획해 장애물을 우회하고,
헤딩 조향으로 추종한다. 마지막 지점 도착 시 처음으로 순환.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

sys.path.insert(0, __file__.rsplit("/", 3)[0]
                + "/module3_gas_safety/return_to_home")
from rth import astar  # noqa: E402

V_WALK = 0.22


class ZonePatrol(Node):
    def __init__(self):
        super().__init__("zone_patrol")
        self.declare_parameter("waypoints", [50.0, 0.0, 16.0, 0.0])
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/model/go2/odometry")
        wp = self.get_parameter("waypoints").value
        self.goals = [(wp[i], wp[i + 1]) for i in range(0, len(wp), 2)]
        self.goal_i = 0
        self.pose = None
        self.path = None
        self.wp_i = 0
        self.laps = 0

        self.pub_cmd = self.create_publisher(
            Twist, self.get_parameter("cmd_topic").value, 10)
        self.pub_status = self.create_publisher(String, "~/status", 5)
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._on_odom, 10)
        self.create_timer(0.1, self._step)
        self.get_logger().info("zone_patrol 시작 (%d개 지점)" % len(self.goals))

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def _step(self):
        if self.pose is None:
            return
        x, y, yaw = self.pose
        goal = self.goals[self.goal_i]

        if math.hypot(goal[0] - x, goal[1] - y) < 0.7:
            self.get_logger().info("지점 %d 도착 (%.1f, %.1f)"
                                   % (self.goal_i, goal[0], goal[1]))
            self.goal_i = (self.goal_i + 1) % len(self.goals)
            if self.goal_i == 0:
                self.laps += 1
            self.path = None
            return

        if self.path is None:
            self.path = astar((x, y), goal)
            self.wp_i = 0
            if self.path is None:
                self.get_logger().error("경로 실패 → 다음 지점으로")
                self.goal_i = (self.goal_i + 1) % len(self.goals)
                return

        while self.wp_i < len(self.path) - 1 and \
                math.hypot(self.path[self.wp_i][0] - x,
                           self.path[self.wp_i][1] - y) < 0.5:
            self.wp_i += 1
        tx, ty = self.path[self.wp_i]
        target = math.atan2(ty - y, tx - x)
        err = math.atan2(math.sin(target - yaw), math.cos(target - yaw))
        cmd = Twist()
        cmd.linear.x = V_WALK * max(0.0, math.cos(err))
        cmd.angular.z = max(-0.4, min(0.4, 1.0 * err))
        self.pub_cmd.publish(cmd)
        self.pub_status.publish(String(
            data="goal %d lap %d" % (self.goal_i, self.laps)))


def main():
    rclpy.init()
    node = ZonePatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
