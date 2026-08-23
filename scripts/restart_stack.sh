#!/bin/bash
# Autoware 스택을 깨끗이 내리고 다시 올린다. behavior_planning 이 상류 버그로 죽었을 때
# (WORKLOG 12 #1) 되살릴 방법이 이것뿐이라 절차를 코드로 남긴다.
#
# 그냥 런치 프로세스만 죽이면 자식 노드가 고아로 남고, 다시 올렸을 때 이름이 겹쳐
# duplicated_node_checker 가 자율주행을 막는다. 경로가 /root/autoware/install 이 아닌
# 노드(robot_state_publisher, topic_tools/relay)까지 지워야 하는 이유다.
set -eo pipefail

echo "[1/4] 스택 종료"
pkill -f "e2e_simulator.launch" 2>/dev/null || true
sleep 8
# 노드 실행파일은 install/<패키지>/lib/ 아래에 있다. 경로를 이만큼 좁히지 않으면
# RViz 까지 잡힌다 — RViz 는 -d 인자로 install/<패키지>/share/ 경로를 들고 있다 (실측).
for pat in "/root/autoware/install/[a-z0-9_]*/lib/" "component_container" "rclcpp_components" "topic_tools/relay" "robot_state_publisher"; do
  pkill -f "$pat" 2>/dev/null || true
done
sleep 3
for pat in "/root/autoware/install/[a-z0-9_]*/lib/" "component_container" "topic_tools/relay" "robot_state_publisher"; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
sleep 2

echo "[2/4] 재기동"
LOG=/workspace/autoware_launch.log
setsid nohup bash /workspace/scripts/start_autoware.sh > "$LOG" 2>&1 < /dev/null &

echo "[3/4] 노드 기동 대기 (약 2분)"
source /opt/ros/humble/setup.bash
export CYCLONEDDS_URI=file:///root/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
for _ in $(seq 1 30); do
  sleep 10
  n=$(timeout 25 ros2 node list 2>/dev/null | grep -c . || echo 0)
  if [ "$n" -ge 180 ]; then break; fi
done
echo "  노드 ${n}개"

echo "[4/4] 사전 점검"
sleep 10
exec bash /workspace/scripts/preflight.sh
