#!/bin/bash
# Autoware e2e_simulator 기동 (RViz 없이 — :21 에 따로 띄운다)
#
# rviz_respawn 기본값이 true 라 그냥 죽이면 :20 에 계속 되살아난다.
# 끄려면 런치 인자로 꺼야 한다.
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
exec ros2 launch autoware_launch e2e_simulator.launch.xml \
  vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit \
  map_path:=/root/awsim/nishishinjuku_autoware_map launch_vehicle_interface:=true \
  rviz:=false rviz_respawn:=false
