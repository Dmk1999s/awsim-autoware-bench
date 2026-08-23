#!/bin/bash
# 파라미터 하나를 바꿔 배치를 돌린다.
#   run_ab.sh <시나리오> <횟수> <태그> <파라미터> <값>
#
# MPC 가중치는 런타임 변경이 가능해 Autoware 재시작이 필요 없다.
# 재시작하면 그 자체가 조건 차이를 만들므로 오히려 이쪽이 깨끗하다.
set -eo pipefail
SCENARIO="$1"; N="$2"; TAG="$3"; PARAM="$4"; VALUE="$5"
unset VIRTUAL_ENV
export PATH="/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/humble/setup.bash
source /root/autoware/install/setup.bash
export CYCLONEDDS_URI=file:///root/cyclonedds.xml RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

NODE=/control/trajectory_follower/controller_node_exe
ros2 param set "$NODE" "$PARAM" "$VALUE"
echo "설정: $PARAM = $(ros2 param get "$NODE" "$PARAM" | tail -1)"
exec bash /workspace/scripts/run_batch.sh "$SCENARIO" "$N" "$TAG"
