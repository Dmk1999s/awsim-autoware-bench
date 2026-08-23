#!/bin/bash
# 속도 상한을 계속 발행한다. 이게 없으면 주행 자체가 불가능하다.
#
# /planning/scenario_planning/max_velocity_default 의 원래 발행자는 RViz 의
# "Set Velocity Limit" 패널이다. 헤드리스로(또는 RViz 를 나중에 띄우고) 스택을 재기동하면
# 이 토픽의 발행자가 0이 되고, 그러면:
#   external_velocity_limit_selector 침묵 → velocity_smoother 침묵 →
#   /planning/trajectory 없음 → 자율주행 준비 안 됨 (러너는 "자율주행 준비 60s 초과"로만 보인다)
#
# 값 4.17 m/s(=15 km/h)는 external_velocity_limit_selector 의 기본 파라미터 max_vel 과 같다.
# 지금까지 측정한 모든 주행의 최고 속도(약 4.03 m/s)가 이 상한에서 나왔으므로,
# 다른 값을 쓰면 과거 회차와 비교할 수 없게 된다.
#
# 값은 인자로 준다 (기본 4.17). 높이면 횡가속도가 속도의 제곱으로 커지므로 곡선 실험의
# 조건이 달라진다 — 회차를 섞지 말 것.
set -e
LIMIT="${1:-4.17}"
unset VIRTUAL_ENV
export PATH="/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/humble/setup.bash
source /root/autoware/install/setup.bash
export CYCLONEDDS_URI=file:///root/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# transient_local + 저빈도 반복: 늦게 뜬 구독자도 받게 하고, 발행자가 살아 있어야 하므로 상주시킨다
setsid nohup ros2 topic pub -r 0.5 --qos-durability transient_local \
  /planning/scenario_planning/max_velocity_default \
  autoware_internal_planning_msgs/msg/VelocityLimit "{max_velocity: $LIMIT, sender: bench}" \
  > /var/log/velocity_limit.log 2>&1 < /dev/null &
echo "속도 상한 $LIMIT m/s 발행 (PID $!)"
