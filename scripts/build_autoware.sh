#!/bin/bash
# Autoware 1.9.0 소스 가져오기 → 의존성 → 빌드
#
# 주의: 이 컨테이너의 /venv/main 이 python3 를 가로챈다. vcs·rosdep·colcon 은
# apt/시스템 파이썬에 설치되므로 venv 를 PATH 에서 빼지 않으면 "명령을 찾을 수 없음"이 난다.
# (setup-dev-env.sh 도 같은 이유로 pipx 를 못 찾고 죽었다.)
# set -u 는 쓰지 않는다 — ROS 의 setup.bash 가 미정의 변수(AMENT_TRACE_SETUP_FILES 등)를
# 참조하는 것이 정상이라, -u 를 켜면 source 하는 순간 죽는다.
set -eo pipefail

unset VIRTUAL_ENV
export PATH="/root/.local/bin:/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

source /opt/ros/humble/setup.bash
cd /root/autoware

echo "=== [1/4] vcs import ==="
mkdir -p src
vcs import src < repositories/autoware.repos

echo "=== [2/4] rosdep update ==="
rosdep update

echo "=== [3/4] rosdep install ==="
rosdep install -y --from-paths src --ignore-src --rosdistro humble

echo "=== [4/4] colcon build ==="
# 12코어 / 62GB. 패키지 4개 동시 × 패키지당 make -j4 ≈ 16 잡.
# colcon 기본값(패키지 12개 × make -j12)은 메모리를 터뜨린다.
export MAKEFLAGS="-j4"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 4

echo "=== 빌드 완료 ==="
