# RESTART — 인스턴스 재시작 절차

vast.ai 인스턴스를 중지했다 다시 켰을 때 스택을 복구하는 순서.
컨테이너 파일시스템은 유지되므로 설정은 그대로 남아 있고, **프로세스만 다시 띄우면 된다.**

---

## 0. 먼저 확인 — 대역폭 설정이 살아 있는가

**이걸 빼먹으면 시간당 $2.30이 새어나간다.** (자세한 경위는 PROGRESS.md 「대역폭 과금 사고」)

```bash
grep AllowMulticast /root/cyclonedds.xml
```

**반드시 `spdp`** 여야 한다. `true`면 아래로 복구:

```bash
cp /root/cyclonedds.xml.mcast-bak /root/cyclonedds.xml   # 원본(true) 백업
sed -i 's|<AllowMulticast>true</AllowMulticast>|<AllowMulticast>spdp</AllowMulticast>|' /root/cyclonedds.xml
```

---

## 1. AWSIM  (디스플레이 `:20` — 브라우저 스트리밍)

```bash
cd /workspace
export DISPLAY=:20 XMODIFIERS=@im=none XDG_RUNTIME_DIR=/run/user/1001
export CYCLONEDDS_URI=file:///root/cyclonedds.xml RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
setsid nohup /root/awsim/awsim_labs_v1.6.1/awsim_labs.x86_64 -force-vulkan \
  > /var/log/awsim.log 2>&1 < /dev/null &
```

- `XMODIFIERS=@im=none` 없으면 **세그폴트** (Unity가 fcitx XIM에 붙다 죽음)
- `-force-vulkan` 없으면 **llvmpipe 소프트웨어 렌더링**으로 떨어짐

**그다음 브라우저 스트리밍 화면에서 직접 `Load` 를 눌러야 한다.**
런처 값은 기본값 그대로 두면 된다 (Shinjuku / Use traffic 체크 / Lexus RX450h / 81380.72, 49918.78, 41.57 / 0,0,35).
누르기 전까지는 ROS 토픽이 6개만 보인다 — 정상이다.

---

## 2. Autoware  (RViz 없이)

```bash
source /opt/ros/humble/setup.bash
source /root/autoware/install/setup.bash
export DISPLAY=:21 CYCLONEDDS_URI=file:///root/cyclonedds.xml RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch autoware_launch e2e_simulator.launch.xml \
  vehicle_model:=awsim_labs_vehicle sensor_model:=awsim_labs_sensor_kit \
  map_path:=/root/awsim/nishishinjuku_autoware_map launch_vehicle_interface:=true \
  rviz:=false rviz_respawn:=false
```

`rviz_respawn` 기본값이 `true`라 (`autoware_launch/launch/autoware.launch.xml:46`)
그냥 죽이면 `:20`에 계속 되살아난다. **끄려면 런치 인자로 꺼야 한다.**

기동에 약 60초. 노드 약 189개.

---

## 3. RViz  (디스플레이 `:21` — VNC 5901)

```bash
source /opt/ros/humble/setup.bash
source /root/autoware/install/setup.bash
export DISPLAY=:21 CYCLONEDDS_URI=file:///root/cyclonedds.xml RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
setsid nohup ros2 run rviz2 rviz2 \
  -d /root/autoware/install/autoware_launch/share/autoware_launch/rviz/autoware.rviz \
  -s /root/autoware/install/autoware_launch/share/autoware_launch/rviz/image/autoware.png \
  --ros-args -r __node:=rviz2_display21 -p use_sim_time:=True \
  -p wheel_radius:=0.383 -p wheel_width:=0.235 -p wheel_base:=2.79 -p wheel_tread:=1.64 \
  -p front_overhang:=1.0 -p rear_overhang:=1.1 -p left_overhang:=0.128 -p right_overhang:=0.128 \
  -p vehicle_height:=2.5 -p max_steer_angle:=0.7 \
  > /var/log/rviz21.log 2>&1 < /dev/null &
```

---

## 4. 검증

```bash
source /opt/ros/humble/setup.bash && source /root/autoware/install/setup.bash
export CYCLONEDDS_URI=file:///root/cyclonedds.xml RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 node list | wc -l                                    # 약 189
ros2 topic list | wc -l                                   # 약 775
ros2 topic echo /api/localization/initialization_state --once | grep state   # state: 3
ros2 topic hz /localization/pose_estimator/pose_with_covariance             # 약 8 Hz
```

**대역폭 확인 (30초)** — 이게 정상이어야 요금이 안 샌다:

```bash
mc(){ grep -A1 '^IpExt:' /proc/net/netstat | awk 'NR==1{for(i=1;i<=NF;i++)h[i]=$i}NR==2{for(i=1;i<=NF;i++)if(h[i]=="OutMcastOctets")print $i}'; }
A=$(cat /sys/class/net/eth0/statistics/tx_bytes); MA=$(mc); sleep 30
B=$(cat /sys/class/net/eth0/statistics/tx_bytes); MB=$(mc)
awk -v a=$A -v b=$B 'BEGIN{printf "eth0 TX  : %.2f Mbps → $%.3f/hr\n",(b-a)*8/30/1e6,(b-a)*120/1e9*0.04096}'
awk -v a=$MA -v b=$MB 'BEGIN{printf "멀티캐스트: %.3f Mbps\n",(b-a)*8/30/1e6}'
```

| 판정 | 기준 |
|---|---|
| ✅ 정상 | eth0 10 Mbps 미만, 멀티캐스트 0.1 Mbps 미만 |
| ❌ 설정 안 먹음 | 멀티캐스트 수십 Mbps → 0단계로 |

---

## 화면 보는 법

| 디스플레이 | 대상 | 접속 |
|---|---|---|
| `:20` (1920×1080) | AWSIM | 브라우저 스트리밍 (selkies, tryclouflare URL) |
| `:21` (1920×1080) | RViz | VNC 포트 5901 / noVNC |

**브라우저 창은 전체화면보다 1920×1080 근처가 선명하다.**
selkies가 `--enable_resize=false`라 스트림 해상도가 1920×1080 고정이고,
인코더도 `x264enc`(소프트웨어)라 전체화면으로 키우면 업스케일되어 뭉개진다.

---

## 요금 메모

| 항목 | 시간당 |
|---|---|
| GPU + 디스크 | $0.369 |
| 대역폭 (수정 후) | 약 $0.116 |
| **합계** | **약 $0.49/hr ≈ $12/day** |

- 화면 안 볼 때 **브라우저 탭을 닫으면** 스트리밍 트래픽이 멈춰 $0.37/hr까지 내려간다.
- vast.ai 대시보드의 "$/day"는 **인터넷 요금 미포함**이다 (화면 하단 주석).
- **인스턴스를 중지해도 디스크 요금은 계속 나간다.** 완전히 멈추려면 인스턴스를 삭제해야 하고,
  그러면 이 환경 전체가 사라진다 — 중지 상태 유지가 맞다.
