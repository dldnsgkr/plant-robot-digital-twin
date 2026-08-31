# 플랜트 4족 보행 로봇 Digital Twin — 개발 컨테이너
# ROS 2 Humble + Gazebo Classic 11 + Nav2 + Foxglove Bridge
# Apple Silicon(arm64)과 x86_64 모두 지원 (멀티아치 베이스 이미지)
FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

# 시뮬레이션·제어·내비게이션 스택
# - gazebo-ros-pkgs: Gazebo Classic 11 + ROS2 브리지 (CHAMP 스택 호환)
# - navigation2/nav2-bringup: Nav2 내비게이션 (과제 지정)
# - ros2-control 계열: 관절 제어기 (보행 컨트롤러 하부)
# - foxglove-bridge: Mac 호스트의 Foxglove Studio에서 WebSocket(8765)으로 시각화
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-foxglove-bridge \
    ros-humble-tf-transformations \
    ros-humble-rmw-cyclonedds-cpp \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    wget \
    unzip \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Python 라이브러리 (비전·수치 계산)
# PyTorch는 이미지가 800MB+ 커지므로 Module 2 착수 시점(Phase 3)에 추가한다
RUN pip3 install --no-cache-dir \
    "numpy<2" \
    scipy \
    opencv-python-headless \
    transforms3d

# colcon 워크스페이스: 호스트의 리포지토리가 /ws/src/plant_dt 로 마운트됨
WORKDIR /ws

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /root/.bashrc

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

CMD ["bash"]
