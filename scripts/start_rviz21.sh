#!/bin/bash
# RViz 를 :21 에 띄운다 (Autoware 런치의 RViz 는 rviz:=false 로 꺼둔 상태)
set -e

# 이 컨테이너의 /venv/main 이 python3 를 가로챈다. 그 venv 는
# include-system-site-packages = false 라 numpy 가 없고, Autoware 의 파이썬 노드
# (map_hash_generator 등)가 rclpy 임포트 단계에서 ModuleNotFoundError 로 죽는다.
# PROGRESS.md 진단 #3 과 같은 원인. venv 를 PATH 에서 빼서 시스템 파이썬을 쓴다.
unset VIRTUAL_ENV
export PATH="/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/humble/setup.bash
source /root/autoware/install/setup.bash
export DISPLAY=:21
export CYCLONEDDS_URI=file:///root/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
SHARE=/root/autoware/install/autoware_launch/share/autoware_launch
setsid nohup ros2 run rviz2 rviz2 \
  -d "$SHARE/rviz/autoware.rviz" -s "$SHARE/rviz/image/autoware.png" \
  --ros-args -r __node:=rviz2_display21 -p use_sim_time:=True \
  -p wheel_radius:=0.383 -p wheel_width:=0.235 -p wheel_base:=2.79 -p wheel_tread:=1.64 \
  -p front_overhang:=1.0 -p rear_overhang:=1.1 -p left_overhang:=0.128 -p right_overhang:=0.128 \
  -p vehicle_height:=2.5 -p max_steer_angle:=0.7 \
  > /var/log/rviz21.log 2>&1 < /dev/null &
echo "RViz 기동 (PID $!) — :21"
