#!/usr/bin/env python3
"""넘어짐 감지 + 자동 기립 (Fall Recovery) 노드.

감지: /imu 쿼터니언 → 롤/피치 각도. |roll| 또는 |pitch| > 임계값(1.0rad)이
0.3초 이상 지속되면 '넘어짐'으로 판정한다 (순간 충격 오검출 방지).

복구 시퀀스 (상태 머신):
  NOMINAL → (넘어짐 감지) → TUCK   : 다리를 몸에 붙여 웅크림 (관성 최소화)
          → RIGHT                  : 비대칭 킥 — 한쪽 다리쌍을 힙 롤 + 신전으로
                                     바닥을 차서 몸통을 굴린다. 2초 내 미기립 시
                                     반대쪽으로 재시도 (최대 8회)
          → EXTEND                 : 기립 자세로 천천히 램프 (1.5초)
          → NOMINAL

복구 중에는 /gait_enable(false) 를 발행해 게이트 생성기를 멈추고,
관절 명령 토픽을 이 노드가 점유한다.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64MultiArray

FALL_ANGLE = 1.0       # rad — 이 이상 기울면 넘어짐 후보
FALL_HOLD = 0.3        # s — 지속 시간(순간 충격 무시)
UPRIGHT_ANGLE = 0.35   # rad — 이 미만이면 몸통이 다시 선 것

TUCK = (0.0, 1.35, -2.55)
STAND = (0.0, 0.8, -1.6)
LEGS = ("FL", "FR", "RL", "RR")


def rp_from_quat(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    return roll, pitch


class FallRecovery(Node):
    def __init__(self):
        super().__init__("fall_recovery")
        self.state = "NOMINAL"
        self.roll = 0.0
        self.pitch = 0.0
        self.tilt_since = None
        self.t_state = 0.0
        self.dt = 0.02

        self.pub_cmd = self.create_publisher(
            Float64MultiArray, "/joint_group_position_controller/commands", 10)
        self.pub_enable = self.create_publisher(Bool, "/gait_enable", 10)
        self.create_subscription(Imu, "/imu", self._on_imu, 50)
        self.create_timer(self.dt, self._step)
        self.get_logger().info("fall_recovery 시작")

    def _on_imu(self, msg):
        q = msg.orientation
        self.roll, self.pitch = rp_from_quat(q.x, q.y, q.z, q.w)

    def _cmd(self, pose_per_leg):
        data = []
        for leg in LEGS:
            data.extend(pose_per_leg)
        self.pub_cmd.publish(Float64MultiArray(data=data))

    def _cmd_kick(self, side, extend):
        """비대칭 킥으로 몸통 굴리기. side(+1=왼쪽 FL/RL 킥, -1=오른쪽), extend 0→1.

        - 뒤집힘(supine, |roll|>2.2): thigh를 2.9rad까지 돌려 다리를 등 너머로 넘긴 뒤
          (발이 땅에 닿음) 킥 쪽 calf를 펴서 바닥을 밀어 굴린다.
        - 옆으로 누움: 바닥에 닿은 쪽 다리를 신전해 밀고, 반대쪽은 웅크려 관성 최소화.
        """
        kick_legs = ("FL", "RL") if side > 0 else ("FR", "RR")
        supine = abs(self.roll) > 2.2
        data = []
        for leg in LEGS:
            if supine:
                # 다리를 등 위로 (준비 단계에서 thigh만 먼저 넘김)
                thigh = TUCK[1] + (2.9 - TUCK[1]) * min(1.0, extend * 2 + 0.3)
                if leg in kick_legs:
                    hip = side * 1.04 * extend
                    calf = TUCK[2] + (-0.95 - TUCK[2]) * extend   # 킥: 무릎 신전
                else:
                    hip = -side * 0.5 * extend
                    calf = TUCK[2]
                data.extend((hip, thigh, calf))
            else:
                if leg in kick_legs:
                    hip = side * 1.04 * extend
                    thigh = TUCK[1] + (0.4 - TUCK[1]) * extend
                    calf = TUCK[2] + (-0.95 - TUCK[2]) * extend
                    data.extend((hip, thigh, calf))
                else:
                    data.extend((-side * 0.2, TUCK[1], TUCK[2]))
        self.pub_cmd.publish(Float64MultiArray(data=data))

    def _step(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        tilted = abs(self.roll) > FALL_ANGLE or abs(self.pitch) > FALL_ANGLE

        if self.state == "NOMINAL":
            if tilted:
                if self.tilt_since is None:
                    self.tilt_since = now
                elif now - self.tilt_since > FALL_HOLD:
                    self.get_logger().warn(
                        "넘어짐 감지 (roll=%.2f pitch=%.2f) → 복구 시작"
                        % (self.roll, self.pitch))
                    self.state = "TUCK"
                    self.t_state = 0.0
                    self.kick_side = 1.0 if self.roll > 0 else -1.0
                    self.attempts = 0
                    self.pub_enable.publish(Bool(data=False))
            else:
                self.tilt_since = None
            return

        self.t_state += self.dt

        if self.state == "TUCK":
            self._cmd(TUCK)
            if self.t_state > 1.0:
                self.state, self.t_state = "RIGHT", 0.0

        elif self.state == "RIGHT":
            # 펄스 킥: 0.3s 준비 후 (0.25s 급신전 → 0.25s 수축) 반복 — 흔들림 축적
            if self.t_state < 0.3:
                self._cmd_kick(self.kick_side, 0.0)
            else:
                t = (self.t_state - 0.3) % 0.5
                extend = min(1.0, t / 0.25) if t < 0.25 else max(0.0, 1.0 - (t - 0.25) / 0.25)
                self._cmd_kick(self.kick_side, extend)
            # 기립 판정은 상시 확인 (킥 도중 몸이 돌아설 수 있음)
            if abs(self.roll) < UPRIGHT_ANGLE and abs(self.pitch) < UPRIGHT_ANGLE:
                self.state, self.t_state = "EXTEND", 0.0
            elif self.t_state > 2.8:  # 펄스 5회 후 평가
                self.attempts += 1
                if self.attempts >= 8:
                    self.get_logger().error("복구 실패 (8회 시도) — 수동 개입 필요")
                    self.state = "NOMINAL"
                    self.pub_enable.publish(Bool(data=True))
                    return
                # 반대쪽으로 재시도
                self.kick_side = -self.kick_side
                self.state, self.t_state = "TUCK", 0.5  # 짧은 재웅크림 후 킥

        elif self.state == "EXTEND":
            # 킥 자세에서 바로 STAND로 점프하면 재전도 위험 → TUCK 경유 후 램프
            if self.t_state < 0.6:
                self._cmd(TUCK)
                return
            a = min(1.0, (self.t_state - 0.6) / 1.5)
            pose = tuple(TUCK[i] + a * (STAND[i] - TUCK[i]) for i in range(3))
            self._cmd(pose)
            if self.t_state > 2.4:
                self.get_logger().info("기립 복구 완료")
                self.state = "NOMINAL"
                self.tilt_since = None
                self.pub_enable.publish(Bool(data=True))


def main():
    rclpy.init()
    node = FallRecovery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
