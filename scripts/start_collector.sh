#!/bin/bash
# metrics_collector — 주행 1회를 CSV 로 기록 (경로 SET → ARRIVED 자동 감지)
set -e
unset VIRTUAL_ENV
export PATH="/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/humble/setup.bash
source /root/autoware/install/setup.bash
source /workspace/install/setup.bash
export CYCLONEDDS_URI=file:///root/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
exec ros2 run autoware_bench metrics_collector "$@"
