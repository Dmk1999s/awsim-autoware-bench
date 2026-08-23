#!/bin/bash
# 군집 채널을 켠 「그림자」 추적기를 원본과 나란히 띄운다 (WORKLOG 37).
#
# 왜 이런 방식인가: 추적기 입력 채널은 런치 시점에 정해진다 (파라미터로 켜도 구독이
# 안 만들어진다). 설치 트리를 고치면 전체 재기동과 되돌리기 비용이 붙는다. 그래서
# **돌고 있는 추적기의 커맨드라인을 그대로 복사**해 채널 하나만 바꿔 별도 이름으로 띄운다.
# 원본 파이프라인은 건드리지 않는다 — 출력은 /shadow/tracking/objects 로만 나간다.
#
# 파라미터 파일이 노드 경로로 키가 걸려 있어 그대로는 새 이름의 노드에 안 먹는다.
# 복사본에서 그 키를 /** 로 바꿔 쓴다.
set -eo pipefail
unset VIRTUAL_ENV
export PATH="/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/humble/setup.bash
source /root/autoware/install/setup.bash
export CYCLONEDDS_URI=file:///root/cyclonedds.xml RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORK=${1:-/tmp/shadow_tracker}
NODE=/root/autoware/install/autoware_multi_object_tracker/lib/autoware_multi_object_tracker/multi_object_tracker_node
PID=$(ps -eo pid,comm | awk '$2 ~ /^multi_object_tr/ {print $1; exit}')
[ -n "$PID" ] || { echo "원본 추적기를 못 찾았다 — 스택이 떠 있는지 확인할 것"; exit 1; }

mkdir -p "$WORK/params"; rm -f "$WORK/params"/*
tr '\0' '\n' < /proc/$PID/cmdline > "$WORK/argv.txt"
i=0
for f in $(grep -E '^(/tmp/launch_params|/root/autoware.*\.yaml)$' "$WORK/argv.txt"); do
  i=$((i+1))
  sed 's|^/perception/object_recognition/tracking/multi_object_tracker:|/**:|' "$f" \
    > "$WORK/params/$(printf '%02d' $i).yaml"
done
echo "파라미터 $i 개 복사 → $WORK/params"

PF=""
for f in "$WORK"/params/*.yaml; do PF="$PF --params-file $f"; done

# detection02 는 원본에서 centerpoint_short_range(=none) 자리다. 그 자리에 군집을 넣는다.
exec "$NODE" \
  --ros-args -r __node:=mot_shadow -r __ns:=/shadow \
  -p use_sim_time:=True \
  -p wheel_radius:=0.383 -p wheel_width:=0.235 -p wheel_base:=2.79 -p wheel_tread:=1.64 \
  -p front_overhang:=1.0 -p rear_overhang:=1.1 -p left_overhang:=0.128 -p right_overhang:=0.128 \
  -p vehicle_height:=2.5 -p max_steer_angle:=0.7 \
  $PF \
  -p input/detection02/channel:=lidar_clustering \
  -r '~/input/detection01/objects:=/perception/object_recognition/detection/centerpoint/objects' \
  -r '~/input/detection02/objects:=/perception/object_recognition/detection/clustering/objects' \
  -r '~/input/odometry:=/localization/kinematic_state' \
  -r '~/output/objects:=/shadow/tracking/objects' \
  -r '~/output/merged_objects:=/shadow/detection/objects'
