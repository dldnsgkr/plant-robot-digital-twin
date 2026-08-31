#!/usr/bin/env python3
"""Go2 게이트 생성기 (stage1) — 트로트/크롤 보행 + 해석적 다리 IK.

/cmd_vel (Twist) 를 받아 12관절 목표각을
/joint_group_position_controller/commands (Float64MultiArray) 로 발행한다.

보행 원리:
  - 위상 오실레이터가 다리별 위상(0~1)을 돌리고, 위상에 따라 각 발이
    지지(stance: 몸 아래에서 뒤로 밀기) / 유각(swing: 사이클로이드로 앞으로 복귀)을 반복
  - 발 목표 위치(힙 기준 3D)를 해석적 IK로 관절각 (hip roll, thigh pitch, calf pitch) 변환
  - 게이트 전환: trot(대각 2쌍, duty 0.5) ↔ crawl(한 다리씩, duty 0.75, 험지용 저속 안정 보행)

실행:
  ros2 run … 또는 python3 gait_controller.py
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}' -r 10
  게이트 전환: ros2 param set /gait_controller gait crawl
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float32MultiArray, Float64MultiArray

# ---- Go2 기구 파라미터 (urdf 실측) ----
L_HIP = 0.0955     # 힙 롤축 → 다리 평면 y 오프셋
L_THIGH = 0.213
L_CALF = 0.213
HIP_X = 0.1934     # base → 힙 x 오프셋
HIP_Y = 0.0465     # base → 힙 y 오프셋

BODY_HEIGHT = 0.30  # 지지 시 힙 아래 발 깊이
STAND = (0.0, 0.8, -1.6)

# 다리 순서는 컨트롤러 yaml의 관절 순서와 동일해야 한다
LEGS = ("FL", "FR", "RL", "RR")
SIDE = {"FL": +1, "FR": -1, "RL": +1, "RR": -1}   # y 부호 (왼쪽 +)

# 게이트 정의: {다리: 위상 오프셋}, duty = 지지 구간 비율
GAITS = {
    "trot":  {"offset": {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5},
              "duty": 0.5, "period": 0.5, "swing_h": 0.06},
    # 험지 모드: 지형 lift 감지 시 자동 전환 — 느린 주기·긴 지지(보폭 증가,
    # 항상 3발 근접 지지)와 빠른 착지 프로파일로 접촉 타이밍 오차를 줄인다
    "obstacle": {"offset": {"FL": 0.0, "RR": 0.0, "FR": 0.5, "RL": 0.5},
                 "duty": 0.62, "period": 0.9, "swing_h": 0.07},
    "crawl": {"offset": {"FL": 0.0, "RR": 0.25, "FR": 0.5, "RL": 0.75},
              "duty": 0.75, "period": 1.2, "swing_h": 0.08},
}


def leg_ik(x, y, z, side):
    """힙 원점 기준 발 목표 (x앞+, y왼+, z위+) → (hip, thigh, calf) 관절각."""
    l_off = side * L_HIP
    # 힙 롤: 발을 다리 시상면으로 보내는 회전
    r_sq = y * y + z * z - l_off * l_off
    r = math.sqrt(max(r_sq, 1e-6))          # 시상면 내 힙-발 깊이
    q1 = math.atan2(z, y) + math.atan2(r, l_off)
    # 시상면 2링크 IK (q2: 수직 아래 기준, +는 발이 뒤로)
    d = math.sqrt(x * x + r * r)
    d = max(min(d, L_THIGH + L_CALF - 1e-4), 0.11)  # 무릎 완전접힘 특이점 회피
    cos_knee = (L_THIGH**2 + L_CALF**2 - d * d) / (2 * L_THIGH * L_CALF)
    q3 = math.acos(max(-1.0, min(1.0, cos_knee))) - math.pi
    psi = math.acos(max(-1.0, min(1.0,
        (L_THIGH**2 + d * d - L_CALF**2) / (2 * L_THIGH * d))))
    q2 = math.atan2(-x, r) + psi
    return q1, q2, q3


class GaitController(Node):
    def __init__(self):
        super().__init__("gait_controller")
        self.declare_parameter("gait", "trot")
        self.declare_parameter("rate_hz", 100.0)

        self.cmd = Twist()
        self.phase = 0.0
        self.active = False   # cmd_vel이 0이면 기립 유지
        self.enabled = True   # fall_recovery가 복구 중이면 false
        self.body_h = BODY_HEIGHT  # 험지에서 동적으로 상승 (최대 0.36)

        self.pub = self.create_publisher(
            Float64MultiArray, "/joint_group_position_controller/commands", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(Bool, "/gait_enable", self._on_enable, 10)
        # 지형 적응: elevation map에서 전방 최대 장애물 높이를 읽어 스윙 높이 가산
        self.declare_parameter("terrain_adapt", True)
        self.declare_parameter("force_lift", -1.0)  # 디버그: 리프트 강제 지정
        self.terrain_lift = 0.0
        self.create_subscription(
            Float32MultiArray, "/elevation_map/raw", self._on_elev, 1)
        # [실험 기능] 접촉 감지 스윙 동결: 스윙 중 무릎 토크 스파이크 → 수평 이동 동결.
        # 정상 스윙의 동적 토크와 접촉 토크의 분리가 아직 안 되어 기본 비활성.
        # 활성화: -p contact_freeze:=true (임계값은 contact_effort_th)
        self.declare_parameter("contact_freeze", False)
        self.declare_parameter("contact_effort_th", 18.0)
        self.calf_effort = {leg: 0.0 for leg in LEGS}
        self.swing_frozen = {leg: None for leg in LEGS}  # 동결 시점의 sx
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)

        # stage2 LIP-MPC: CoP 오프셋을 발 목표에 가산 (mpc_node 미실행 시 0)
        self.cop = [0.0, 0.0]
        self.cop_stamp = 0.0
        self.create_subscription(Vector3, "/mpc/cop_offset", self._on_cop, 10)

        # IMU 피드백: 요 유지(횡 드리프트 방지) + 롤/피치 자세 안정화
        self.declare_parameter("yaw_hold", True)
        self.declare_parameter("posture_gain", 0.5)
        self.roll = self.pitch = self.yaw = 0.0
        self.gyro_z = 0.0
        self.target_yaw = None
        self.wz_corr = 0.0
        self.create_subscription(Imu, "/imu", self._on_imu, 50)
        self.create_timer(2.0, self._debug)

    def _debug(self):
        self.get_logger().info(
            "dbg lift=%.3f body_h=%.3f roll=%.2f pitch=%.2f wz_corr=%.2f active=%s"
            % (self.terrain_lift, self.body_h, self.roll, self.pitch,
               self.wz_corr, self.active))

        rate = self.get_parameter("rate_hz").value
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self._step)
        self.get_logger().info("gait_controller 시작 (gait=%s)"
                               % self.get_parameter("gait").value)

    def _on_cmd(self, msg):
        self.cmd = msg

    def _on_cop(self, msg):
        self.cop = [msg.x, msg.y]
        self.cop_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _on_joints(self, msg):
        for i, name in enumerate(msg.name):
            if name.endswith("_calf_joint") and i < len(msg.effort):
                self.calf_effort[name[:2]] = abs(msg.effort[i])

    def _on_imu(self, msg):
        q = msg.orientation
        self.roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                               1 - 2 * (q.x * q.x + q.y * q.y))
        s = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))
        self.pitch = math.asin(s)
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))
        self.gyro_z = msg.angular_velocity.z

    def _on_enable(self, msg):
        self.enabled = msg.data
        if not msg.data:
            self.active = False
            self.phase = 0.0

    def _on_elev(self, msg):
        """전방 스트립(x 0.2~1.2m, |y|<0.3m)의 최대 높이 → 스윙 리프트 가산량.

        LiDAR 최저 빔(-30°)의 근접 사각(약 0.7m) 때문에 장애물이 가까워지면
        맵에서 사라진다 → 리프트를 즉시 낮추지 않고 천천히 감쇠(이력 유지)시켜
        장애물 위를 지나는 동안 발 들기를 유지한다.
        """
        if not self.get_parameter("terrain_adapt").value:
            self.terrain_lift = 0.0
            return
        n, res, half = 80, 0.05, 2.0
        h, hx_dbg, hy_dbg = 0.0, 0.0, 0.0
        try:
            for i in range(int((0.45 + half) / res), int((1.2 + half) / res)):
                for j in range(int((-0.3 + half) / res), int((0.3 + half) / res)):
                    v = msg.data[i * n + j]
                    if v == v and v > h:
                        h, hx_dbg, hy_dbg = v, i * res - half, j * res - half
        except IndexError:
            pass
        if h > 0.03:
            self.get_logger().info(
                "지형: h=%.2f at (%.2f, %.2f)" % (h, hx_dbg, hy_dbg),
                throttle_duration_sec=2.0)
        # 몸이 기울어진 동안은 맵이 신뢰 불가(테레포트/충격 과도상태) → 갱신 보류
        if abs(self.roll) > 0.25 or abs(self.pitch) > 0.25:
            return
        # 오검출 방지: 2연속 관측된 높이만 신뢰
        h_eff = min(h, getattr(self, "_h_prev", 0.0))
        self._h_prev = h

        # 장애물 상단 + 3cm 여유만큼 발을 든다 (지면 노이즈 3cm 무시).
        # 상한 0.10: 그 이상은 관절 진폭이 커져 접지 타이밍이 무너진다(실험으로 확인).
        # 더 높은 장애물은 '넘는' 대상이 아니라 Nav2 코스트맵으로 '우회'하는 대상.
        new_lift = min(max(h_eff + 0.03, 0.0), 0.10) if h_eff > 0.03 else 0.0
        # 이력 유지(장애물이 근접 사각지대에 들어가도 수 초간 발 들기 유지, 반감기 ~5s)
        self.terrain_lift = max(new_lift, self.terrain_lift * 0.93)
        if self.terrain_lift < 0.01:
            self.terrain_lift = 0.0

    def _foot_target(self, leg, gait, phase):
        """다리별 위상 → 힙 기준 발 목표 좌표."""
        duty = gait["duty"]
        vx = self.cmd.linear.x
        vy = self.cmd.linear.y
        wz = self.cmd.angular.z
        # 요 유지: 회전 명령이 없을 때 초기 방위를 PD제어로 유지 (횡 드리프트 방지).
        # 단, 리프트 보행 중에는 회전 스윕이 지면을 긁어 오히려 요를 흩뜨리므로 중단.
        if (abs(wz) < 1e-3 and self.target_yaw is not None
                and self.terrain_lift < 0.02
                and self.get_parameter("yaw_hold").value):
            err = math.atan2(math.sin(self.target_yaw - self.yaw),
                             math.cos(self.target_yaw - self.yaw))
            # PD 제어: 자이로 감쇠 없이는 지연 때문에 요 진동이 발산한다
            wz = max(-0.25, min(0.25, 0.7 * err - 0.25 * self.gyro_z))
            self.wz_corr = wz
        # 회전 성분: 힙 위치에 따른 접선 속도 기여
        hx = HIP_X if leg[0] == "F" else -HIP_X
        hy = SIDE[leg] * (HIP_Y + L_HIP)
        vx_leg = vx - wz * hy
        vy_leg = vy + wz * hx

        t_stance = gait["period"] * duty
        step_x = vx_leg * t_stance / 2.0     # 발 스트로크 절반
        step_y = vy_leg * t_stance / 2.0

        p = (phase + gait["offset"][leg]) % 1.0
        if p < duty:                          # 지지: 앞→뒤로 밀기
            self.swing_frozen[leg] = None
            s = p / duty                      # 0→1
            x = step_x * (1 - 2 * s)
            y = step_y * (1 - 2 * s)
            z = -self.body_h
        else:                                 # 유각: 뒤→앞 복귀 + 사이클로이드 들어올림
            s = (p - duty) / (1 - duty)
            # 수평 이동은 s=0.15(이륙 후)~0.8(착지 전)에만 — 발이 지면에 있는
            # 이륙/착지 구간에 전진하면 지면을 긁어 몸을 뒤로 밀어낸다
            sx = min(max((s - 0.15) / 0.65, 0.0), 1.0)
            # 조기 접촉(무릎 토크 스파이크) 감지 시 수평 이동 동결 → 긁힘 차단
            if (self.get_parameter("contact_freeze").value
                    and self.swing_frozen[leg] is None and 0.25 < s < 0.95
                    and self.calf_effort[leg]
                    > self.get_parameter("contact_effort_th").value):
                self.swing_frozen[leg] = sx
            if self.swing_frozen[leg] is not None:
                sx = self.swing_frozen[leg]
            x = step_x * (2 * sx - 1)
            y = step_y * (2 * sx - 1)
            lift = gait["swing_h"] + self.terrain_lift  # 지형 적응 가산
            if self.swing_frozen[leg] is not None:
                z = -self.body_h                        # 접촉했으면 바로 지지 전환
            else:
                z = -self.body_h + lift * math.sin(math.pi * s)
        # 자세 안정화: 롤/피치만큼 다리별 지지 깊이를 보정해 몸통 수평 유지
        # (roll>0 = 왼쪽 위 → 오른쪽(hy<0) 다리 신전 → dz = +k·hy·roll)
        kp = self.get_parameter("posture_gain").value
        dz = kp * (hy * self.roll - hx * self.pitch)
        z += max(-0.04, min(0.04, dz))
        # stage2 MPC CoP 오프셋: 지지다각형을 CoM 대비 이동시켜 예측적 균형 회복
        # (0.5초 이상 미수신이면 무시 — MPC 노드 없이도 stage1 단독 동작)
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.cop_stamp < 0.5:
            x += self.cop[0]
            y += self.cop[1]
        return x, SIDE[leg] * L_HIP + y, z

    def _step(self):
        if not self.enabled:   # 복구 중에는 fall_recovery가 관절을 점유
            return
        fl = self.get_parameter("force_lift").value
        if fl >= 0.0:
            self.terrain_lift = fl
        gait = GAITS[self.get_parameter("gait").value]
        # 험지 lift 활성 시 험지 모드로 자동 전환 (사용자가 crawl 지정 시 유지)
        if self.terrain_lift > 0.05 and self.get_parameter("gait").value == "trot":
            gait = GAITS["obstacle"]

        # 험지에서 몸통을 서서히 높인다 (장애물에 배가 걸리지 않도록, 5cm/s 램프)
        target_h = BODY_HEIGHT + min(self.terrain_lift * 0.5, 0.06)
        step = 0.05 * self.dt
        self.body_h += max(-step, min(step, target_h - self.body_h))
        moving = (abs(self.cmd.linear.x) + abs(self.cmd.linear.y)
                  + abs(self.cmd.angular.z)) > 1e-3

        if moving:
            if not self.active:
                self.target_yaw = self.yaw  # 보행 시작 방위를 유지 목표로
            self.active = True
            self.phase = (self.phase + self.dt / gait["period"]) % 1.0
        elif self.active:
            # 정지 명령: 위상을 0으로 되돌리고 기립 자세로 복귀
            self.active = False
            self.phase = 0.0
            self.target_yaw = None

        # 기립 정지 중에도 MPC CoP 오프셋을 반영해 외란에 예측적으로 대응
        now = self.get_clock().now().nanoseconds * 1e-9
        cop_fresh = now - self.cop_stamp < 0.5

        angles = []
        for leg in LEGS:
            if self.active:
                x, y, z = self._foot_target(leg, gait, self.phase)
                q = leg_ik(x, y, z, SIDE[leg])
            elif cop_fresh and (abs(self.cop[0]) + abs(self.cop[1])) > 0.003:
                q = leg_ik(self.cop[0], SIDE[leg] * L_HIP + self.cop[1],
                           -self.body_h, SIDE[leg])
            else:
                q = STAND
            angles.extend(q)

        self.pub.publish(Float64MultiArray(data=angles))


def main():
    rclpy.init()
    node = GaitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
