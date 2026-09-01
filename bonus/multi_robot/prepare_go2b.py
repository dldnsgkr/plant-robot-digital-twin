#!/usr/bin/env python3
"""2호기(go2b) 스폰용 URDF·컨트롤러 설정 생성 (보너스②: 다중 로봇).

1호기(go2_sim.urdf)와의 차이:
  - 센서: IMU만 탑재 (카메라·LiDAR 제외 — 협업 순찰 데모의 렌더링 부하 절감)
  - IMU 토픽: go2b/imu (1호기와 충돌 방지)
  - ros2_control: /go2b 네임스페이스 (+ 전용 컨트롤러 yaml)
  - 오도메트리: 모델명 기반 /model/go2b/odometry (플러그인이 자동 분리)

실행: python3 prepare_go2b.py  →  simulation/models/go2b_sim.urdf,
      module1_locomotion/config/go2b_controllers.yaml
"""
import re

ROOT = __file__.rsplit("/", 3)[0]
SRC = ROOT + "/simulation/models/go2_description/urdf/go2_description.urdf"
OUT_URDF = ROOT + "/simulation/models/go2b_sim.urdf"
OUT_YAML = ROOT + "/module1_locomotion/config/go2b_controllers.yaml"

text = open(SRC).read().replace(
    "package://go2_description", "model://go2_description")

STAND = {"hip": 0.0, "thigh": 0.8, "calf": -1.6}
joints = []
for leg in ("FL", "FR", "RL", "RR"):
    for part in ("hip", "thigh", "calf"):
        joints.append(f'''    <joint name="{leg}_{part}_joint">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">{STAND[part]}</param></state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>''')

block = '''
  <gazebo reference="base">
    <sensor name="imu" type="imu">
      <update_rate>200</update_rate>
      <topic>go2b/imu</topic>
      <always_on>1</always_on>
    </sensor>
  </gazebo>
  <ros2_control name="GazeboSimSystem" type="system">
    <hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>
''' + "\n".join(joints) + '''
  </ros2_control>
  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <ros><namespace>go2b</namespace></ros>
      <parameters>/ws/src/plant_dt/module1_locomotion/config/go2b_controllers.yaml</parameters>
    </plugin>
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_publish_frequency>50</odom_publish_frequency>
      <dimensions>3</dimensions>
    </plugin>
  </gazebo>
'''

# 로봇 이름 충돌 방지
text = re.sub(r'<robot name="[^"]+"', '<robot name="go2b_description"', text)
open(OUT_URDF, "w").write(text.replace("</robot>", block + "</robot>"))

yaml = """# go2b 네임스페이스 컨트롤러 구성 (prepare_go2b.py 생성)
go2b:
  controller_manager:
    ros__parameters:
      update_rate: 200
      joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster
      joint_group_position_controller:
        type: position_controllers/JointGroupPositionController
  joint_group_position_controller:
    ros__parameters:
      joints:
        - FL_hip_joint
        - FL_thigh_joint
        - FL_calf_joint
        - FR_hip_joint
        - FR_thigh_joint
        - FR_calf_joint
        - RL_hip_joint
        - RL_thigh_joint
        - RL_calf_joint
        - RR_hip_joint
        - RR_thigh_joint
        - RR_calf_joint
"""
open(OUT_YAML, "w").write(yaml)
print("생성:", OUT_URDF)
print("생성:", OUT_YAML)
