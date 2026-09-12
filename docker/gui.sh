#!/usr/bin/env bash
# Gazebo GUI를 브라우저(noVNC)로 보기 위한 컨테이너 내부 디스플레이 기동 스크립트
#
# 사용 (sim 컨테이너 안에서):
#   bash /ws/src/plant_dt/docker/gui.sh          # Xvfb + VNC + noVNC 만 기동
#   bash /ws/src/plant_dt/docker/gui.sh --gz     # 위 + Gazebo GUI 클라이언트(gz sim -g)까지
# 그 다음 Mac 브라우저에서 http://localhost:8080/vnc.html → Connect
#
# 구성: Xvfb(:1, 가상 X 화면) ← x11vnc(5900) ← websockify+noVNC(8080, 브라우저)
# GPU 가 없으므로 Mesa llvmpipe 소프트웨어 렌더링 (LIBGL_ALWAYS_SOFTWARE=1)
set -e
export DISPLAY=:1
W=${DISPLAY_WIDTH:-1600}; H=${DISPLAY_HEIGHT:-900}

if ! pgrep -x Xvfb >/dev/null; then
  Xvfb :1 -screen 0 ${W}x${H}x24 +extension GLX +render -noreset >/tmp/xvfb.log 2>&1 &
  sleep 1
fi
if ! pgrep -x x11vnc >/dev/null; then
  x11vnc -display :1 -forever -shared -nopw -quiet -rfbport 5900 -localhost >/tmp/x11vnc.log 2>&1 &
fi
if ! pgrep -f websockify >/dev/null; then
  websockify --web /usr/share/novnc 0.0.0.0:8080 localhost:5900 >/tmp/novnc.log 2>&1 &
fi
sleep 1
echo "[gui.sh] display :1 (${W}x${H}) ready — 브라우저: http://localhost:8080/vnc.html"

if [[ "$1" == "--gz" ]]; then
  source /opt/ros/jazzy/setup.bash
  # ogre(1.x) 렌더 엔진: 소프트웨어 렌더링에서 기본 ogre2 보다 훨씬 가볍다
  export LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
  echo "[gui.sh] gz sim -g (ogre) 기동 — 실행 중인 gz 서버에 자동 접속"
  # gui.config: 창 크기를 Xvfb 화면(1600x900)에 맞춤 (기본 1000x845)
  exec gz sim -g --render-engine ogre --gui-config /ws/src/plant_dt/docker/gui.config
fi
