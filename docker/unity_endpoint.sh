#!/usr/bin/env bash
# Unity ↔ ROS 2 연결 서버(ROS-TCP-Endpoint) 감시 실행기.
# ROS 2 판 엔드포인트는 클라이언트(Unity Play 정지 등)가 끊긴 뒤 소켓 상태가 깨져
# ("[Errno 9] Bad file descriptor") 이후 접속을 1초 만에 끊는 결함이 있다.
# 그 로그가 보이면 엔드포인트를 즉시 재시작한다 (Unity 쪽은 자동 재접속).
# 사용 (컨테이너 안): bash /ws/src/plant_dt/docker/unity_endpoint.sh   [포트 기본 10000]
PORT=${1:-10000}; LOG=/tmp/endpoint.log
source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash
while true; do
  : > "$LOG"
  ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=$PORT >> "$LOG" 2>&1 &
  EP=$!
  echo "[$(date +%H:%M:%S)] endpoint started pid=$EP" >> /tmp/endpoint_watchdog.log
  while kill -0 $EP 2>/dev/null; do
    if grep -q "Bad file descriptor" "$LOG"; then
      echo "[$(date +%H:%M:%S)] broken socket detected → restart" >> /tmp/endpoint_watchdog.log
      kill $EP 2>/dev/null; sleep 1; kill -9 $EP 2>/dev/null; break
    fi
    sleep 1
  done
  sleep 1
done
