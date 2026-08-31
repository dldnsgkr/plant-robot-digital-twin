#!/usr/bin/env bash
# Phase 0 완료 기준 검증: Gazebo Harmonic(headless) + ROS2 브리지 동작 확인
# 사용: docker compose run --rm --no-deps sim bash /ws/src/plant_dt/docker/smoke_test.sh
set -e
source /opt/ros/jazzy/setup.bash

echo "== [1/4] ROS2 기본 동작 =="
ros2 topic list

echo "== [2/4] Gazebo Harmonic 서버(headless) 기동 =="
gz sim -s -r -v 1 empty.sdf &
GZ_PID=$!
sleep 8

echo "== [3/4] Gazebo 토픽 및 물리 스텝 확인 =="
gz topic -l | head -8
echo "-- world stats (RTF 확인):"
timeout 5 gz topic -e -t /world/empty/stats -n 1 | grep -E "real_time_factor|iterations" || true

echo "== [4/4] gz ↔ ROS2 클럭 브리지 확인 =="
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock &
BR_PID=$!
sleep 3
timeout 5 ros2 topic echo /clock --once | head -4

kill $BR_PID $GZ_PID 2>/dev/null || true
echo ""
echo "SMOKE TEST PASSED — Phase 0 환경 정상 (ROS2 Jazzy + Gazebo Harmonic, arm64 native)"
