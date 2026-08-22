#!/bin/bash
# AWSIM Labs 기동 (디스플레이 :20 — 브라우저 스트리밍)
#
# XMODIFIERS=@im=none 이 없으면 세그폴트한다. 컨테이너 기본값이 @im=fcitx 인데
# Unity 가 fcitx XIM 에 붙다 NULL 역참조로 죽는다 (Player.log 조차 안 남는다).
# -force-vulkan 이 없으면 :20 의 GLX 가 llvmpipe(소프트웨어 렌더링)로 떨어진다.
set -e

# 이 컨테이너의 /venv/main 이 python3 를 가로챈다. 그 venv 는
# include-system-site-packages = false 라 numpy 가 없고, Autoware 의 파이썬 노드
# (map_hash_generator 등)가 rclpy 임포트 단계에서 ModuleNotFoundError 로 죽는다.
# PROGRESS.md 진단 #3 과 같은 원인. venv 를 PATH 에서 빼서 시스템 파이썬을 쓴다.
unset VIRTUAL_ENV
export PATH="/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
export DISPLAY=:20
export XMODIFIERS=@im=none
export XDG_RUNTIME_DIR=/run/user/1001
export CYCLONEDDS_URI=file:///root/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
setsid nohup /root/awsim/awsim_labs_v1.6.1/awsim_labs.x86_64 -force-vulkan \
  > /var/log/awsim.log 2>&1 < /dev/null &
echo "AWSIM 기동 (PID $!). 브라우저 스트리밍 화면에서 Load 를 눌러야 시뮬레이션이 시작된다."
echo "누르기 전에는 ROS 토픽이 6개만 보인다 — 정상이다."
