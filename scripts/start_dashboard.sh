#!/bin/bash
# 실시간 모니터링 대시보드. 브라우저로 열어 두고 주행을 지켜보는 용도.
#   컨테이너 10100 → vast.ai 가 직접 TCP 매핑한 외부 포트 (env 의 VAST_TCP_PORT_10100).
#   6100·6200 은 portal 의 caddy 가 이미 쓰고 있어 못 쓴다 (인증 붙은 401 이 돌아온다).
set -e
unset VIRTUAL_ENV
export PATH="/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/humble/setup.bash
source /root/autoware/install/setup.bash
source /workspace/install/setup.bash
export CYCLONEDDS_URI=file:///root/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# sim time 필수 — 스택 전체가 sim time 으로 돈다
setsid nohup ros2 run autoware_bench dashboard --ros-args -p use_sim_time:=true \
  > /var/log/dashboard.log 2>&1 < /dev/null &
echo "대시보드 기동 (PID $!) — 컨테이너 10100 / 외부 ${VAST_TCP_PORT_10100:-미확인}"
