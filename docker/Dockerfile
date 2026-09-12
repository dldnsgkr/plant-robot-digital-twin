# 플랜트 4족 보행 로봇 Digital Twin — 개발 컨테이너
# ROS 2 Jazzy + Gazebo Harmonic + Nav2 + Foxglove Bridge
#
# 스택 선정 경위 (Phase 0에서 검증):
# - osrf/ros:humble-desktop 은 amd64 전용 → Apple Silicon에서 에뮬레이션(RTF ~0.5)
# - Humble arm64 에는 Gazebo Classic 도, ros-gz-sim 도 빌드가 없음
# - Jazzy + Harmonic 은 arm64 네이티브로 전체 스택 제공 → 채택
FROM ros:jazzy

ARG DEBIAN_FRONTEND=noninteractive

# 시뮬레이션·제어·내비게이션 스택
# - ros-jazzy-ros-gz: Gazebo Harmonic(gz-sim8) + ROS2 브리지/스폰 유틸
# - gz-ros2-control: Gazebo 안의 로봇 관절을 ros2_control로 구동
# - navigation2/nav2-bringup: Nav2 내비게이션 (과제 지정)
# - foxglove-bridge: Mac 호스트의 Foxglove Studio에서 WebSocket(8765)으로 시각화
# - rosbridge-server: Phase 5 웹 대시보드용 WebSocket(9090)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-rosbridge-server \
    ros-jazzy-tf-transformations \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-osqp-vendor \
    ros-jazzy-eiquadprog \
    libeigen3-dev \
    python3-pip \
    python3-colcon-common-extensions \
    python3-opencv \
    python3-scipy \
    python3-transforms3d \
    git \
    wget \
    unzip \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Gazebo GUI 확인용 — 브라우저(noVNC)로 컨테이너 안 X 화면을 본다 (arm64 네이티브)
# Xvfb(가상 X 서버) + x11vnc(VNC 서버) + noVNC/websockify(브라우저 클라이언트)
# libgl1-mesa-dri: GPU 없는 컨테이너에서 llvmpipe 소프트웨어 OpenGL 렌더링
RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb \
    x11vnc \
    novnc \
    websockify \
    libgl1-mesa-dri \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Unity 디지털 트윈 뷰어 연결용 ROS-TCP-Endpoint (Unity Robotics Hub, ROS 2 브랜치) — 포트 10000
RUN mkdir -p /ws/src && cd /ws/src && \
    git clone --depth 1 -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git && \
    cd /ws && . /opt/ros/jazzy/setup.sh && colcon build --packages-select ros_tcp_endpoint

# PyTorch는 이미지가 800MB+ 커지므로 Module 2 착수 시점(Phase 3)에 추가한다
# (Ubuntu 24.04는 PEP668 적용 — pip 설치 시 --break-system-packages 필요)

# colcon 워크스페이스: 호스트의 리포지토리가 /ws/src/plant_dt 로 마운트됨
WORKDIR /ws

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /root/.bashrc

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

CMD ["bash"]
