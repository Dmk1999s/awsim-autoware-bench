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
exec ros2 run autoware_bench scenario_runner --ros-args -p scenario:="$1"
