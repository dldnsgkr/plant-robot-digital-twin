#!/usr/bin/env bash
# 통합 미션을 깨끗한 컨테이너에서 실행 (Mac 에서 docker/ 디렉토리 기준으로 실행)
#   bash docker/run_mission.sh [rth_start_pct=50.0]
# 컨테이너를 재시작해 이전 실행의 잔여 프로세스(ros_gz 브리지 등 고아 프로세스가 쌓이면
# RTF 가 급락하고 명령 토픽이 중복돼 로봇이 넘어진다)를 완전히 정리한 뒤,
# noVNC 화면·Unity 엔드포인트 감시기·미션 launch 를 순서대로 띄운다.
set -e
cd "$(dirname "$0")"
PCT=${1:-50.0}
docker compose restart sim >/dev/null
sleep 3
docker compose exec -T sim bash /ws/src/plant_dt/docker/gui.sh >/dev/null
docker compose exec -d sim bash /ws/src/plant_dt/docker/unity_endpoint.sh
docker compose exec -d sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 launch /ws/src/plant_dt/simulation/launch/plant_dt.launch.py mission:=true rth_start_pct:=$PCT > /tmp/mission.log 2>&1"
until docker compose exec -T sim grep -aq "\[mission_controller\]: 미션 시작" /tmp/mission.log 2>/dev/null; do sleep 3; done
echo "mission started $(date +%H:%M:%S) (rth_start_pct=$PCT) — 로그: docker compose exec sim tail -f /tmp/mission.log"
