# PROGRESS

AWSIM Labs 1.6.1 + Autoware 1.9.0 (ROS 2 Humble) 가상 도심 자율주행 개인 프로젝트.
실행 환경: vast.ai GPU 컨테이너 (RTX 5080 16GB / Ryzen 7 7700 16코어 / 64GB), MacBook에서 SSH·VS Code로 접속.

---

## 로드맵 진행

| STEP | 내용 | 상태 |
|---|---|---|
| 1 | AWSIM 실행 | ✅ 2026-08-20 |
| 2 | ROS 2 연결 | ✅ 2026-08-20 |
| 3 | 센서 데이터 이해 (RViz) | ✅ 2026-08-20 |
| 4 | Autoware 연결 | ✅ 2026-08-20 |
| 5 | 출발 → 목적지 자율주행 | ✅ 2026-08-20 |
| 6 | 도심 기능 (신호등·앞차·장애물·교차로) | 🔶 진행 중 |

---

## 2026-08-20 · 환경 진단

스택이 뜨지 않던 원인 4가지. 전부 로그만 봐서는 엉뚱한 곳을 가리키도록 오도하는 것들이라
재발견이 어렵다. `migration_backup/RESTORE.md`에 재현 절차와 함께 기록.

| # | 증상 | 실제 원인 | 조치 |
|---|---|---|---|
| 1 | AWSIM 세그폴트 (Player.log조차 안 남음) | 컨테이너 기본값 `XMODIFIERS=@im=fcitx` → Unity가 fcitx XIM에 붙다 NULL 역참조 | `XMODIFIERS=@im=none` |
| 2 | GPU 미사용 | `:20`의 GLX가 llvmpipe(소프트웨어 렌더링) | `-force-vulkan` → RTX 5080 사용, 47fps |
| 3 | ROS 노드 numpy 못 찾음 | `.bashrc`가 활성화하는 `/venv/main`이 `include-system-site-packages=false` | `pyvenv.cfg` → `true` |
| 4 | 노드는 뜨는데 맵이 영원히 안 뜸 | `lo`는 멀티캐스트 불가 → CycloneDDS 유니캐스트 폴백 → 노드 180개 규모에서 `map_container` 하나만 조용히 디스커버리 누락 (에러 없음) | `/root/cyclonedds.xml`을 `eth0` + `AllowMulticast true` |

부수: `autoware_data/ml_models/lidar_centerpoint/`가 변형별 폴더(base/tiny/…) 신규 레이아웃인데
설치된 런처는 `pts_voxel_encoder_<model>.onnx` 평면 레이아웃을 기대 → 접미사 심볼릭 링크 + ml_package yaml 복사로 해결.

**4번이 결정타.** 참가자 수 제한 확장만으로는 노드는 뜨지만 맵 로더가 빈 채로 남아 위치추정이 영원히 안 된다.

---

## 2026-08-20 · 첫 자율주행 검증

```
AWSIM        47 fps, RTX 5080, LiDAR/카메라/GNSS/IMU 발행
Autoware     188 노드, 사망 프로세스 0
Localization Initialized
Perception   객체 인식 9.2 Hz, 신호등 인식 14.7 Hz
Planning     trajectory 10.0 Hz → Control 19.3 Hz
```

주행 기록:

| 회차 | 구간 | 결과 |
|---|---|---|
| 1 | (81380.7, 49918.8) → (81470.3, 49980.1) | 약 108 m, Arrived |
| 2 | (81470.1, 49980.0) → (81567.6, 50015.8) | 약 101 m, 신호 교차로 통과, 오차 **6 cm** |
| 3 | (81567.5, 50015.8) → (81657.1, 50032.7) | 약 91 m, 38.5 s |

---

## 2026-08-20 · 인프라 개선

**AWSIM / RViz 화면 분리.** 두 창이 1920×1080 한 화면에 겹쳐 작업이 어려웠음.
두 번째 X 디스플레이 `:21`을 만들어 RViz를 분리, noVNC로 별도 브라우저 창에 띄움.
AWSIM은 Selkies(WebRTC/x264)에 그대로 둠 — 3D 영상은 프레임 차분 방식인 VNC에 불리하기 때문.
Autoware 런치의 RViz는 `respawn="true"`라 죽여도 되살아나므로, 죽이는 대신 `:21`에 하나 더 띄우는 방식을 택함
(재시작 0회, 188노드·위치추정 그대로 유지).

**인스턴스 이전 시도 (중단).** 요금·지연 때문에 KR 인스턴스로 옮기려 했으나 실패.
머신 97383(SK브로드밴드, 대전)은 컨테이너 내부 sshd가 정상 리슨함에도 **호스트 인바운드 포트 포워딩이 전부 막힘**.
Cloudflare 터널(아웃바운드)만 동작 → 가정용 회선 호스트의 전형적 증상으로 판단, JP 인스턴스 유지 결정.
설정·노하우 백업은 `migration_backup.tar.gz`(524KB)로 확보.

---

## 2026-08-20 · P1 착수 — 주행 평가 하네스

`/workspace/src/autoware_bench/` — 직접 작성한 ROS 2 패키지.

**문제의식.** Autoware의 `planning_evaluator` / `control_evaluator`가 이미 지표를 실시간 발행하지만,
아무도 그것을 **주행 1회 단위로 모으거나 회차 간 비교하지 않는다.** 그 빈자리를 채우는 것이 목표.

### metrics_collector (127줄)

- 구독: `planning_evaluator/metrics`, `control_evaluator/metrics`, `kinematic_state`, `api/routing/state`
- `routing/state`가 `SET`이 되면 자동으로 기록 시작, `ARRIVED`/`UNSET`이면 종료
- 출력: `runs/run_YYYYmmdd_HHMMSS.csv`

**설계 판단**

- **long format** (`t, source, name, value`) 채택. `MetricArray`는 지표 이름이 메시지마다 달라질 수 있어
  고정 컬럼 CSV는 깨지기 쉽다. 분석 시 pivot.
- `/api/routing/state`만 **TRANSIENT_LOCAL** — 구독자 QoS를 맞추지 않으면 시작 시 현재 상태를 못 받아
  기록이 조용히 시작되지 않는다. 코드 작성 전 QoS를 확인해 회피.

### 검증 결과 (2026-08-20)

```
colcon build          성공 (1.92s)
노드 구독              4개 토픽 전부 확인
실주행 1회 기록        36,918행 / 94개 지표 / 38.54초
```

수집된 주요 지표: `lateral_deviation`, `lateral_deviation_centerline`, `yaw_deviation`, `jerk`,
`lateral_acceleration_abs`, `closest_object_distance`, `goal_longitudinal/lateral/yaw_deviation`,
`ego_lane_info/lane_id`, `ego/x·y·yaw·vel`

---

## 2026-08-20 · 대역폭 과금 사고 — DDS 멀티캐스트 유출

인스턴스 생성 10시간도 안 돼 $10 초과 청구. vast.ai 화면의 "$8.84/day"는 상한이 아니었다 —
그 아래 **"Total base cost excludes internet charges."** 가 붙어 있고, Internet은 별도 종량 과금($40.96/TB)이다.

### 측정

| | 값 |
|---|---|
| eth0 누적 TX | 158 GB |
| └ 그중 멀티캐스트 (`OutMcastOctets`) | **152.7 GB (96.7%)** |
| 실시간 eth0 TX | 125 Mbps (시간당 56 GB ≈ **$2.30/hr**) |
| AWSIM·Autoware 전부 종료 시 | **0.96 Mbps, 멀티캐스트 0.00** ← 범인 확정 |

기본요금 $0.369/hr보다 **대역폭이 6배 비쌌다.** 실소진율은 $2.67/hr = $64/day.

### 원인

STEP 4에서 맵 로더 디스커버리 누락을 고치려고 `/root/cyclonedds.xml`을
`eth0` + `AllowMulticast=true`로 바꿨던 것의 부작용. `true`는 디스커버리뿐 아니라
**실제 데이터까지** 멀티캐스트로 내보낸다.

DDS 참가자는 전부 이 컨테이너 한 네임스페이스 안에 있다
(`239.255.0.1`에 142개 프로세스 가입, 외부 참가자 0). 즉 eth0으로 나간 152 GB는
**받는 사람이 아무도 없는 순수 낭비**였고, vast.ai는 그걸 인터넷 egress로 과금했다.

### 조치 — 한 줄

```xml
<AllowMulticast>spdp</AllowMulticast>   <!-- true → spdp -->
```

참가자 디스커버리(SPDP)에만 멀티캐스트를 쓰고, 엔드포인트 디스커버리와 데이터는 유니캐스트.
목적지가 자기 자신의 IP(172.17.0.2)라 커널이 `lo`로 라우팅 → **과금 대상에서 사라진다.**
`lo`는 누적이 아무리 커도 과금되지 않는다.

백업: `/root/cyclonedds.xml.mcast-bak` (되돌리려면 복사 후 재시작)

### 결과 (AWSIM + Autoware 전체 가동 실측)

| | 수정 전 | 수정 후 |
|---|---:|---:|
| eth0 TX (과금) | 125 Mbps | **6.27 Mbps** |
| └ 멀티캐스트 | 118 Mbps | **0.028 Mbps** (4,700배 ↓) |
| `lo` TX (무과금) | — | **1,322 Mbps** ← 데이터 전량 이동 |
| 대역폭 요금 | $2.30/hr | **$0.116/hr** |
| 총 소진율 | $2.67/hr ($64/day) | **$0.49/hr ($12/day)** |

시뮬레이션은 아무것도 잃지 않았다 — 노드 189, 토픽 775, Localization Initialized, NDT 8.4 Hz.
데이터 1.3 Gbps가 그대로 흐르되 과금 인터페이스에서만 사라졌다.

**주의: Autoware와 AWSIM 둘 다 재시작해야 적용된다.** 프로세스가 시작할 때 설정을 읽는다.

### 부수 발견

- **AWSIM Labs는 GUI 클릭 없이 시작되지 않는다.** 기동하면 런처 씬에서 대기하고,
  맵 선택 후 `Load`를 눌러야 시뮬레이션이 뜬다. 이 상태에서는 ROS 토픽이 6개만 보인다.
  CLI 자동시작을 시도했으나 `--json_path` / `-json_path` 둘 다 실패
  (`Player.log`에 `No configuration file provided.`). 문자열 자체는 `data.unity3d` 안에 있으나 미해결.
- **RViz 인스턴스 정리.** 기존에는 런치의 RViz(`:20`)와 수동 RViz(`:21`)가 **둘 다** 떠 있었다.
  RViz 하나가 CPU 300%를 쓰므로 `rviz:=false rviz_respawn:=false`로 런치 쪽을 끄고
  `:21` 하나만 유지하도록 바꿈. 기능 손실 없음.

### 삽질 기록 — 화질 저하 오진

AWSIM `Load` 직후 브라우저 화면 화질이 무너져 **RViz가 `:20`으로 넘어온 탓이라고 진단했으나 틀렸다.**
PROGRESS의 「인프라 개선」에 적혀 있듯 원래도 `:20`에는 AWSIM과 런치 RViz가 함께 있었고,
그때 화질은 멀쩡했다.

실제로는 `:20`이 1920×1080 고정이고 selkies가 `--enable_resize=false`라
**브라우저를 전체화면으로 키우면 1920×1080 스트림이 레티나 해상도로 업스케일**되어 뭉개진다.
화질이 좋았던 스크린샷은 창 모드, 나빴던 쪽은 전체화면이었다.

교훈: 인코더는 `x264enc`(**소프트웨어**, NVENC 아님 — GPU 인코더 사용률 0%)이고
스트림 해상도가 고정이므로, **브라우저 창을 1920×1080 근처로 두는 것이 가장 선명하다.**

### 이전(migration) 재검토

KR 이전의 근거였던 Internet 요금 월 $515는 이번 수정으로 **월 약 $20**이 됐다.
남은 이전 근거는 디스크 요금($150 → $10)뿐이며 급하지 않다. JP 유지 판단 유효.
