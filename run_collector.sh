#!/bin/bash
# metrics_collector 실행 (Autoware 오버레이 + DDS 설정 포함)
source /opt/ros/humble/setup.bash
source /root/autoware/install/setup.bash
source /workspace/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///root/cyclonedds.xml
exec ros2 run autoware_bench metrics_collector "$@"
