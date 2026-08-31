#!/usr/bin/env bash
# go2_description 원본 URDF → Gazebo Harmonic 스폰용 go2_sim.urdf 생성
# 1) package:// URI를 model:// 로 변환 (GZ_SIM_RESOURCE_PATH 로 해석)
# 2) 시뮬레이션 센서 부착: LiDAR, RGB 카메라, Depth 카메라, IMU
# 3) ros2_control(12관절 position 인터페이스) + gz_ros2_control 플러그인 주입
set -e
cd "$(dirname "$0")"

SRC=go2_description/urdf/go2_description.urdf
OUT=go2_sim.urdf

sed 's|package://go2_description|model://go2_description|g' "$SRC" > "$OUT"

# </robot> 직전에 센서 정의 삽입 (gz-sim은 <gazebo> 태그의 <sensor>를 SDF로 통과시킴)
python3 - "$OUT" <<'EOF'
import sys

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
    <!-- Depth 카메라: 지형 인지 보조 + 열화상 정합 실험용 (별도 pose로 오프셋) -->
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

# 12관절 ros2_control 정의 — 초기값은 기립 자세 (hip 0, thigh 0.8, calf -1.6)
STAND = {'hip': 0.0, 'thigh': 0.8, 'calf': -1.6}
joints = []
for leg in ('FL', 'FR', 'RL', 'RR'):
    for part in ('hip', 'thigh', 'calf'):
        joints.append(f'''    <joint name="{leg}_{part}_joint">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">{STAND[part]}</param></state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>''')

ros2_control = '''
  <!-- ===== ros2_control (Phase 2에서 주입) ===== -->
  <ros2_control name="GazeboSimSystem" type="system">
    <hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>
''' + '\\n'.join(joints) + '''
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

path = sys.argv[1]
text = open(path).read()
assert '</robot>' in text
open(path, 'w').write(text.replace('</robot>', sensors + ros2_control + '</robot>'))
print(f'{path}: 센서 + ros2_control 삽입 완료')
EOF

echo "생성 완료: $OUT"
