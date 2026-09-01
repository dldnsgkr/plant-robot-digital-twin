"""보너스②: 다중 로봇 협업 순찰 — go2(복도 구역) + go2b(공장 구역).

사전 준비 (호스트에서 1회): python3 bonus/multi_robot/prepare_go2b.py

실행 (컨테이너 안):
    ros2 launch /ws/src/plant_dt/bonus/multi_robot/multi_robot.launch.py
대시보드(dashboard/index.html)에 두 로봇이 색상 구분되어 표시된다.
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

ROOT = "/ws/src/plant_dt"
ENV = {
    "GZ_SIM_RESOURCE_PATH": os.path.join(ROOT, "simulation/models"),
    "GZ_SIM_SYSTEM_PLUGIN_PATH": "/opt/ros/jazzy/lib",
}


def py(path, *args):
    return ExecuteProcess(
        cmd=["python3", os.path.join(ROOT, path)] + list(args),
        output="screen")


def generate_launch_description():
    return LaunchDescription([
        # 기본 시뮬 + 1호기(go2, 전체 센서) + 브리지 + rosbridge
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(ROOT, "simulation/launch/plant_sim.launch.py"))),
        ExecuteProcess(
            cmd=["ros2", "launch", "rosbridge_server",
                 "rosbridge_websocket_launch.xml", "port:=9090"],
            output="screen"),

        # 2호기 스폰 (공장 시작점) + 전용 브리지
        # 주의: 1호기 컨트롤러 활성화(스포너 완료) 이후에 스폰해야 한다 —
        # 두 gz_ros2_control 인스턴스가 겹쳐 로드되면 CM 서비스가 멈춘다(실측)
        TimerAction(period=25.0, actions=[
            # go2b CM은 /go2b/robot_description 을 기다린다 — 없으면 CM 초기화가
            # gz 플러그인 스레드를 블로킹해 시뮬 전체가 정지한다(실측)
            Node(package="robot_state_publisher",
                 executable="robot_state_publisher",
                 namespace="go2b",
                 parameters=[{
                     "robot_description": open(os.path.join(
                         ROOT, "simulation/models/go2b_sim.urdf")).read(),
                     "use_sim_time": True,
                     "frame_prefix": "go2b/",
                 }],
                 output="screen"),
            Node(package="ros_gz_sim", executable="create",
                 arguments=["-world", "plant",
                            "-file", os.path.join(
                                ROOT, "simulation/models/go2b_sim.urdf"),
                            "-name", "go2b", "-x", "5.0", "-y", "-2.0",
                            "-z", "0.45"],
                 additional_env=ENV, output="screen"),
            Node(package="ros_gz_bridge", executable="parameter_bridge",
                 arguments=[
                     "/go2b/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                     "/model/go2b/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                 ],
                 parameters=[{"use_sim_time": True}], output="screen"),
        ]),

        # 2호기 컨트롤러
        TimerAction(period=40.0, actions=[
            Node(package="controller_manager", executable="spawner",
                 arguments=["joint_state_broadcaster",
                            "joint_group_position_controller",
                            "-c", "/go2b/controller_manager"],
                 output="screen"),
        ]),

        # 1·2호기 보행/복구 + 구역 순찰
        TimerAction(period=48.0, actions=[
            # 1호기 (복도 구역: x 50↔16 왕복)
            py("module1_locomotion/stage1_gait_controller/gait_controller.py"),
            py("module1_locomotion/fall_recovery/fall_recovery.py"),
            py("bonus/multi_robot/zone_patrol.py", "--ros-args",
               "-p", "waypoints:=[50.0, 0.0, 16.0, 0.0]",
               "-p", "cmd_topic:=/cmd_vel",
               "-p", "odom_topic:=/model/go2/odometry"),
            # 2호기 (공장 구역: 기계→탱크→중앙 순환) — 토픽 리매핑
            py("module1_locomotion/stage1_gait_controller/gait_controller.py",
               "--ros-args", "-p", "terrain_adapt:=false",
               "-r", "/cmd_vel:=/go2b/cmd_vel",
               "-r", "/imu:=/go2b/imu",
               "-r", "/model/go2/odometry:=/model/go2b/odometry",
               "-r", "/joint_states:=/go2b/joint_states",
               "-r", "/gait_enable:=/go2b/gait_enable",
               "-r", "/joint_group_position_controller/commands:="
                     "/go2b/joint_group_position_controller/commands"),
            py("module1_locomotion/fall_recovery/fall_recovery.py",
               "--ros-args",
               "-r", "/imu:=/go2b/imu",
               "-r", "/gait_enable:=/go2b/gait_enable",
               "-r", "/joint_group_position_controller/commands:="
                     "/go2b/joint_group_position_controller/commands"),
            py("bonus/multi_robot/zone_patrol.py", "--ros-args",
               "-r", "__node:=zone_patrol_b",
               "-p", "waypoints:=[-5.5, -6.0, -6.8, 6.0, 3.0, 2.0]",
               "-p", "cmd_topic:=/go2b/cmd_vel",
               "-p", "odom_topic:=/model/go2b/odometry"),
        ]),
    ])
