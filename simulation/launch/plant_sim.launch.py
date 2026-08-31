"""발전소 Virtual Plant 통합 실행.

gz sim 서버(headless) + Go2 스폰 + gz↔ROS2 센서 브리지 + Foxglove 브리지.

사용 (컨테이너 안에서):
    ros2 launch /ws/src/plant_dt/simulation/launch/plant_sim.launch.py
    ros2 launch ... gui:=true       # noVNC로 Gazebo GUI도 띄울 때
    ros2 launch ... foxglove:=false # Foxglove 브리지 끌 때
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG_ROOT = "/ws/src/plant_dt/simulation"
WORLD = os.path.join(PKG_ROOT, "worlds", "plant_world.sdf")
ROBOT_URDF = os.path.join(PKG_ROOT, "models", "go2_sim.urdf")

# 로봇 초기 위치: 복도 끝 (미션 시나리오 — 복도를 지나 공장으로 진입)
SPAWN = {"x": "55.0", "y": "0.0", "z": "0.45"}

BRIDGE_TOPICS = [
    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    "/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    "/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
    "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
    "/front_camera@sensor_msgs/msg/Image[gz.msgs.Image",
    "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
    "/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image",
    "/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
]


def generate_launch_description():
    env = {
        "GZ_SIM_RESOURCE_PATH": os.path.join(PKG_ROOT, "models"),
        # gz_ros2_control 플러그인(libgz_ros2_control-system.so) 탐색 경로
        "GZ_SIM_SYSTEM_PLUGIN_PATH": "/opt/ros/jazzy/lib",
    }

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("foxglove", default_value="true"),

        # Gazebo 서버 (물리 + 센서)
        ExecuteProcess(
            cmd=["gz", "sim", "-s", "-r", "-v", "1", WORLD],
            additional_env=env,
            output="screen",
        ),
        # Gazebo GUI (선택, noVNC 디스플레이로 출력)
        ExecuteProcess(
            cmd=["gz", "sim", "-g"],
            condition=IfCondition(LaunchConfiguration("gui")),
            output="screen",
        ),

        # 월드 기동 후 로봇 스폰
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    arguments=[
                        "-world", "plant",
                        "-file", ROBOT_URDF,
                        "-name", "go2",
                        "-x", SPAWN["x"], "-y", SPAWN["y"], "-z", SPAWN["z"],
                        "-Y", "3.14159",  # 공장 방향(-x)을 바라보고 시작
                    ],
                    additional_env=env,
                    output="screen",
                ),
            ],
        ),

        # 로봇 TF 발행 (/joint_states → TF 트리)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": open(ROBOT_URDF).read(),
                "use_sim_time": True,
            }],
            output="screen",
        ),

        # 컨트롤러 기동 (로봇 스폰 → gz_ros2_control 로드 후)
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster",
                               "joint_group_position_controller"],
                    output="screen",
                ),
            ],
        ),

        # gz ↔ ROS2 센서 브리지
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=BRIDGE_TOPICS,
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),

        # Foxglove Studio 접속용 (Mac에서 ws://localhost:8765)
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            parameters=[{"port": 8765, "use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("foxglove")),
            output="screen",
        ),
    ])
