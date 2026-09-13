"""Digital Twin 통합 실행 — 시뮬 + 3개 모듈 + 관제 브리지 + (선택) 미션 데모.

사용 (컨테이너 안에서):
    ros2 launch /ws/src/plant_dt/simulation/launch/plant_dt.launch.py
    ros2 launch ... mission:=true rth_start_pct:=60.0   # 통합 시나리오 데모
    ros2 launch ... degrade:=true                        # 저조도/블러 조건

대시보드: dashboard/index.html 을 브라우저로 열기 (rosbridge ws://localhost:9090)
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ROOT = "/ws/src/plant_dt"


def py(path, *args, **kw):
    return ExecuteProcess(
        cmd=["python3", os.path.join(ROOT, path)] + list(args),
        output="screen", **kw)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mission", default_value="false"),
        DeclareLaunchArgument("degrade", default_value="false"),
        DeclareLaunchArgument("mpc", default_value="false"),
        DeclareLaunchArgument("rth_start_pct", default_value="100.0"),

        # 시뮬레이터 + 로봇 + 센서 브리지 + Foxglove
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(ROOT, "simulation/launch/plant_sim.launch.py"))),

        # 관제용 rosbridge (웹 대시보드 ws://:9090)
        ExecuteProcess(
            cmd=["ros2", "launch", "rosbridge_server",
                 "rosbridge_websocket_launch.xml", "port:=9090"],
            output="screen"),

        # 모듈 노드들 (시뮬 기동 후 순차 시작)
        TimerAction(period=15.0, actions=[
            # Module 1
            py("module1_locomotion/stage1_gait_controller/gait_controller.py"),
            py("module1_locomotion/terrain_mapping/elevation_map.py"),
            py("module1_locomotion/fall_recovery/fall_recovery.py"),
            # Module 2
            py("module2_inspection/gauge_reader/plant_process.py"),
            py("module2_inspection/gauge_reader/gauge_reader.py"),
            py("module2_inspection/thermal_fusion/thermal_camera_sim.py"),
            py("module2_inspection/thermal_fusion/thermal_fusion.py"),
            # Module 3
            py("module3_gas_safety/gas_simulation/gas_field.py"),
            py("module3_gas_safety/source_seeking/source_seeker.py",
               "--ros-args", "-p", "wait_alarm:=true"),
            # 관제 텔레메트리 릴레이 (라이브 피드·이벤트 이력·알람 ack)
            py("dashboard/telemetry_relay.py"),
        ]),
        TimerAction(period=17.0, actions=[
            ExecuteProcess(
                cmd=["python3",
                     os.path.join(ROOT, "module3_gas_safety/return_to_home/rth.py"),
                     "--ros-args", "-p",
                     ["start_pct:=", LaunchConfiguration("rth_start_pct")],
                     "-p", "drain_move:=0.07"],
                output="screen"),
        ]),

        # 선택 노드
        TimerAction(period=17.0, actions=[
            py("module2_inspection/gauge_reader/image_degrader.py",
               condition=IfCondition(LaunchConfiguration("degrade"))),
            py("module1_locomotion/stage2_simple_mpc_py/mpc_node.py",
               cwd=os.path.join(ROOT, "module1_locomotion/stage2_simple_mpc_py"),
               condition=IfCondition(LaunchConfiguration("mpc"))),
        ]),

        # 통합 미션 데모
        TimerAction(period=22.0, actions=[
            py("simulation/mission_controller.py",
               condition=IfCondition(LaunchConfiguration("mission"))),
        ]),
    ])
