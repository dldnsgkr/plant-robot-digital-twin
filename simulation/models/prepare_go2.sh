#!/usr/bin/env bash
# go2_description 원본 URDF → Gazebo Harmonic 스폰용 URDF 생성 (두 벌)
#   go2_sim.urdf        : position 명령 인터페이스만 (기본 — Phase1~5, 미션)
#   go2_sim_effort.urdf : position+effort 이중 인터페이스 (stage3 토크 제어)
# 분리 이유: effort 인터페이스가 등록만 되어 있어도(미클레임 기본 0 토크)
# position 보행의 지지력이 간헐적으로 약해져 전복이 발생함을 실측으로 확인.
# 1) package:// → model:// 변환  2) 센서 부착  3) ros2_control 주입
set -e
cd "$(dirname "$0")"

SRC=go2_description/urdf/go2_description.urdf

sed 's|package://go2_description|model://go2_description|g' "$SRC" > go2_sim.urdf
cp go2_sim.urdf go2_sim_effort.urdf

python3 - <<'EOF'
sensors = '''
  <!-- ===== 시뮬레이션 센서 (Phase 1에서 부착) ===== -->
  <!-- 3D LiDAR: Module 1 Elevation Map 입력 -->
  <gazebo reference="base">
    <sensor name="lidar" type="gpu_lidar">
      <pose>0.15 0 0.1 0 0 0</pose>
      <update_rate>10</update_rate>
      <topic>lidar</topic>
      <gz_frame_id>base</gz_frame_id>
      <lidar>
        <scan>
          <horizontal><samples>360</samples><min_angle>-3.1416</min_angle><max_angle>3.1416</max_angle></horizontal>
          <vertical><samples>16</samples><min_angle>-0.5236</min_angle><max_angle>0.2618</max_angle></vertical>
        </scan>
        <range><min>0.1</min><max>20.0</max><resolution>0.01</resolution></range>
        <noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise>
      </lidar>
      <always_on>1</always_on>
      <visualize>false</visualize>
    </sensor>
    <!-- IMU: 보행 제어 및 넘어짐 판정 -->
    <sensor name="imu" type="imu">
      <update_rate>200</update_rate>
      <topic>imu</topic>
      <gz_frame_id>base</gz_frame_id>
      <always_on>1</always_on>
    </sensor>
  </gazebo>
  <!-- 전방 RGB 카메라: Module 2 게이지 판독 (Head_upper 링크에 장착) -->
  <gazebo reference="Head_upper">
    <sensor name="front_camera" type="camera">
      <pose>0.05 0 0.02 0 0 0</pose>
      <update_rate>15</update_rate>
      <topic>front_camera</topic>
      <gz_frame_id>Head_upper</gz_frame_id>
      <camera>
        <horizontal_fov>1.396</horizontal_fov>
        <image><width>1280</width><height>720</height><format>R8G8B8</format></image>
        <clip><near>0.05</near><far>30</far></clip>
      </camera>
      <always_on>1</always_on>
    </sensor>
    <!-- Depth 카메라: 지형 인지 보조 + 열화상 정합용 (3cm 베이스라인) -->
    <sensor name="depth_camera" type="depth_camera">
      <pose>0.05 0.03 0.02 0 0 0</pose>
      <update_rate>10</update_rate>
      <topic>depth_camera</topic>
      <gz_frame_id>Head_upper</gz_frame_id>
      <camera>
        <horizontal_fov>1.396</horizontal_fov>
        <image><width>640</width><height>480</height><format>R_FLOAT32</format></image>
        <clip><near>0.05</near><far>15</far></clip>
      </camera>
      <always_on>1</always_on>
    </sensor>
  </gazebo>
'''

STAND = {'hip': 0.0, 'thigh': 0.8, 'calf': -1.6}

def ros2_control_block(with_effort):
    joints = []
    for leg in ('FL', 'FR', 'RL', 'RR'):
        for part in ('hip', 'thigh', 'calf'):
            eff = '      <command_interface name="effort"/>\n' if with_effort else ''
            joints.append(
                f'    <joint name="{leg}_{part}_joint">\n'
                f'      <command_interface name="position"/>\n'
                f'{eff}'
                f'      <state_interface name="position"><param name="initial_value">{STAND[part]}</param></state_interface>\n'
                f'      <state_interface name="velocity"/>\n'
                f'      <state_interface name="effort"/>\n'
                f'    </joint>')
    return '''
  <!-- ===== ros2_control ===== -->
  <ros2_control name="GazeboSimSystem" type="system">
    <hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>
''' + '\n'.join(joints) + '''
  </ros2_control>
  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>/ws/src/plant_dt/module1_locomotion/config/go2_controllers.yaml</parameters>
    </plugin>
    <!-- 시뮬 odometry (MPC 상태추정·관제용, gz 토픽 /model/go2/odometry) -->
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_publish_frequency>50</odom_publish_frequency>
      <dimensions>3</dimensions>
    </plugin>
  </gazebo>
'''

for path, with_effort in (("go2_sim.urdf", False), ("go2_sim_effort.urdf", True)):
    text = open(path).read()
    assert '</robot>' in text
    open(path, 'w').write(text.replace(
        '</robot>', sensors + ros2_control_block(with_effort) + '</robot>'))
    print(f'{path}: 생성 완료 (effort={with_effort})')
EOF
