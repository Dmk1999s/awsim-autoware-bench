#!/bin/bash
# RViz 전용 두 번째 X 디스플레이 :21 + VNC(5901) + noVNC(6081)
#
# AWSIM 과 RViz 가 :20 한 화면에 겹치면 작업이 어렵다. RViz 만 분리한다.
# AWSIM 은 :20(selkies/WebRTC)에 그대로 둔다 — 3D 영상은 프레임 차분 방식인 VNC 에 불리하다.
set -e
pgrep -f 'Xvfb :21' >/dev/null || {
  setsid nohup Xvfb :21 -screen 0 1920x1080x24 > /var/log/xvfb21.log 2>&1 < /dev/null &
  sleep 2
}
pgrep -f 'x11vnc -display :21' >/dev/null || {
  setsid nohup x11vnc -display :21 -forever -shared -nopw -rfbport 5901 \
    > /var/log/x11vnc21.log 2>&1 < /dev/null &
  sleep 1
}
pgrep -f 'websockify.*6081' >/dev/null || {
  setsid nohup websockify --web=/usr/share/novnc 6081 localhost:5901 \
    > /var/log/novnc21.log 2>&1 < /dev/null &
}
echo ":21 준비 완료 — VNC 5901 / noVNC 6081"
echo "noVNC 는 최초 1회 Settings → Scaling Mode → Local Scaling 이 필요하다."
