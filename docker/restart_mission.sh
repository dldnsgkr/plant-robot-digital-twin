#!/usr/bin/env bash
# 컨테이너·Unity 엔드포인트는 그대로 두고 시뮬레이션+미션만 다시 띄운다 (Unity Play 유지 가능, 에디터 멈춤 없음)
#   bash docker/restart_mission.sh [rth_start_pct=50.0]
# 이전 launch 트리를 SIGTERM → 확인 → SIGKILL 순으로 완전히 정리하고(ros_gz 브리지 고아 방지),
# 잔여 프로세스가 0개인지 검증한 뒤 launch 한다.
set -e
cd "$(dirname "$0")"
PCT=${1:-50.0}
docker compose exec -T sim bash -c '
  pkill -TERM -f "ros[2] launch" 2>/dev/null; sleep 5
  for pat in "gz si[m]" "parameter_bridg[e]" "robot_state_publishe[r]" "foxglove_bridg[e]" "rosbridg[e]" "controller_manage[r]" "spawne[r]" "[p]ython3 /ws/src/plant_dt" "[p]ython3 /opt/ros/jazzy/lib"; do
    pkill -KILL -f "$pat" 2>/dev/null || true
  done
  sleep 2
  left=$(pgrep -f "gz si[m]|parameter_bridg[e]|ros[2] launch|[p]ython3 /ws/src/plant_dt" | wc -l)
  echo "잔여 시뮬 프로세스: $left (0 이어야 함)"
  pgrep -f "default_server_endpoin[t]" >/dev/null && echo "Unity 엔드포인트: 유지됨" || echo "Unity 엔드포인트: 없음 → unity_endpoint.sh 필요"
  : > /tmp/mission.log'
docker compose exec -d sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 launch /ws/src/plant_dt/simulation/launch/plant_dt.launch.py mission:=true rth_start_pct:=$PCT > /tmp/mission.log 2>&1"
until docker compose exec -T sim grep -aq "\[mission_controller\]: 미션 시작" /tmp/mission.log 2>/dev/null; do sleep 3; done
echo "mission restarted $(date +%H:%M:%S) (rth_start_pct=$PCT)"
