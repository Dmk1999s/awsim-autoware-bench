#!/bin/bash
# 시나리오 1개를 무인으로 완주시킨다. 예: run_scenario.sh scenarios/01_straight.yaml
set -eo pipefail
unset VIRTUAL_ENV
export PATH="/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/humble/setup.bash
source /root/autoware/install/setup.bash
source /workspace/install/setup.bash
export CYCLONEDDS_URI=file:///root/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# use_sim_time 필수 — 스택 전체가 sim time 으로 돈다. 벽시계로 스탬프를 찍으면
# traffic_light_arbiter 가 external_time_tolerance 초과로 외부 신호를 조용히 버린다
# (실측: 빨강을 발행해도 judged 출력이 빈 채로 유지, 차가 그대로 통과).
exec ros2 run autoware_bench scenario_runner --ros-args -p scenario:="$1" -p use_sim_time:=true
