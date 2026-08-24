#!/bin/bash
# 주행/배치 전에 스택이 실제로 달릴 수 있는 상태인지 확인하고, 고칠 수 있는 것은 고친다.
#
# 왜 필요한가: 아래 4가지는 전부 러너에게 `자율주행 준비 60s 초과` 한 줄로만 보인다.
# 무인 배치가 이 상태로 시작하면 회차를 통째로 날린다 (WORKLOG 12).
set -o pipefail   # -u 는 쓰지 않는다 — ROS setup.bash 가 미설정 변수를 참조한다
unset VIRTUAL_ENV
export PATH="/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/humble/setup.bash
source /root/autoware/install/setup.bash
source /workspace/install/setup.bash
export CYCLONEDDS_URI=file:///root/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

fail=0
nodes=$(timeout 30 ros2 node list 2>/dev/null)

# 1. 노드 수 — 정상은 185개 안팎
n=$(echo "$nodes" | grep -c .)
echo "노드 $n개"
[ "$n" -ge 180 ] || { echo "  ✗ 노드가 모자란다 — 아직 기동 중이거나 죽은 노드가 있다"; fail=1; }

# 2. 중복 노드 — 구 스택 잔존 프로세스. duplicated_node_checker 가 ERROR 를 내고 자율주행이 막힌다
dup=$(echo "$nodes" | sort | uniq -d)
if [ -n "$dup" ]; then
  echo "  ✗ 중복 노드: $dup"
  echo "    구 스택 프로세스가 남아 있다. ps -eo pid,lstart,cmd 로 오래된 쪽을 골라 kill 할 것"
  fail=1
fi

# 2b. behavior_planning — 새 경로를 받을 때 상류 버그로 컨테이너가 죽는다(WORKLOG 12 #1).
#     죽어도 노드 수는 4개만 줄어 문턱을 넘으므로, 이름으로 직접 확인해야 잡힌다.
for n in behavior_planning/behavior_path_planner behavior_planning/behavior_velocity_planner; do
  if ! echo "$nodes" | grep -q "$n$"; then
    echo "  ✗ $n 없음 — behavior_planning 컨테이너가 죽었다. 스택 재기동이 필요하다"
    fail=1
  fi
done

# 2c. motion_planning — behavior_planning 과 같은 방식으로 죽는다 (WORKLOG 41, disc4 07).
#     죽으면 궤적이 안 나와 「자율주행 준비 60s 초과」로만 보이고, 노드 수 문턱은 넘는다.
for n in motion_planning/path_optimizer motion_planning/motion_velocity_planner; do
  if ! echo "$nodes" | grep -q "$n$"; then
    echo "  ✗ $n 없음 — motion_planning 컨테이너가 죽었다. 스택 재기동이 필요하다"
    fail=1
  fi
done

# 3. mrm_comfortable_stop_operator — 컴포저블 노드 로드가 재기동마다 간헐적으로 실패한다.
#    빠지면 mrm_handler 침묵 → vehicle_cmd_gate 가 control_cmd 를 아예 안 낸다.
if ! echo "$nodes" | grep -q "^/system/mrm_comfortable_stop_operator$"; then
  echo "  ! mrm_comfortable_stop_operator 없음 — 적재한다"
  timeout 60 ros2 component load /system/mrm_comfortable_stop_operator/mrm_comfortable_stop_operator_container \
    autoware_mrm_comfortable_stop_operator autoware::mrm_comfortable_stop_operator::MrmComfortableStopOperator \
    -n mrm_comfortable_stop_operator --node-namespace /system \
    -p update_rate:=10 -p min_acceleration:=-1.0 -p max_jerk:=0.3 -p min_jerk:=-0.3 -p use_sim_time:=true \
    -r '~/input/mrm/comfortable_stop/operate:=/system/mrm/comfortable_stop/operate' \
    -r '~/output/mrm/comfortable_stop/status:=/system/mrm/comfortable_stop/status' \
    -r '~/output/velocity_limit:=/planning/scenario_planning/max_velocity_candidates' \
    -r '~/output/velocity_limit/clear:=/planning/scenario_planning/clear_velocity_limit' \
    -r '~/input/driving_mode_request:=/system/driving_mode/request' \
    -r '~/input/driving_mode_info:=/system/driving_mode/info' \
    -r '~/output/mrm_state:=/system/driving_mode/mrm_state' > /dev/null 2>&1 \
    && echo "    적재 완료" || { echo "    ✗ 적재 실패"; fail=1; }
fi

# 4. 속도 상한 — 발행자가 없으면 velocity_smoother 가 멈춰 계획이 통째로 안 나온다
if ! pgrep -f "topic pub .*max_velocity_default" > /dev/null; then
  echo "  ! 속도 상한 발행자 없음 — 띄운다"
  bash /workspace/scripts/start_velocity_limit.sh
  sleep 4
fi

# 4b. 수집기 — 이게 죽으면 주행은 되는데 CSV 가 안 남는다. 배치를 다 돌리고 나서야 안다
if ! pgrep -f "autoware_bench/metrics_collector" > /dev/null; then
  echo "  ! 수집기 없음 — 띄운다"
  setsid nohup bash /workspace/scripts/start_collector.sh >> /workspace/collector.log 2>&1 < /dev/null &
  sleep 6
fi

# 5. 위 조치가 먹었는지 최종 확인 — control_cmd 가 나오면 자율주행 전환이 가능하다
# 경로가 없으면 gate 는 아주 낮은 빈도로만 낸다 — 빈도가 아니라 "한 개라도 오는가"로 본다
if timeout 20 ros2 topic echo /control/command/control_cmd --once > /dev/null 2>&1; then
  echo "  ✓ control_cmd 발행 중 — 주행 가능"
else
  echo "  ✗ control_cmd 가 안 나온다. 계획 파이프라인을 상류부터 topic hz 로 좁힐 것 (WORKLOG 12)"
  fail=1
fi

exit $fail
