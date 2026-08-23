# awsim-autoware-bench

AWSIM Labs 1.6.1 + Autoware 1.9.0 (ROS 2 Humble) 가상 도심 자율주행 —
**주행을 측정 가능하게 만드는 평가 하네스**를 직접 만들고, 그걸로 실제 발견을 남긴 개인 프로젝트.

> 한 줄 요약: Autoware가 흘려보내는 지표를 주행 1회 단위로 붙잡아 반복·비교 가능하게 만들었고,
> 그 도구로 **MPC 횡오차 가중치의 교환비(추종 −41% ↔ 조향 +170%)** 와
> **신호 arbiter의 빨강 우선 병합 규칙**을 측정으로 확인했다.

실행 환경: vast.ai GPU 컨테이너(RTX 3090), 도쿄 니시신주쿠 공식 샘플 맵.
좌측통행은 스택 설정이 아니라 **지도의 `turn_direction` 속성**이다 — Autoware에는
`left_hand` 같은 파라미터가 존재하지 않으며, 우측통행 지도를 넣으면 코드 수정 없이 우측통행으로 동작한다.

---

## 문제의식

Autoware의 `planning_evaluator` / `control_evaluator`는 지표를 실시간으로 발행하지만,
**아무도 그것을 주행 1회 단위로 모으거나 회차 간 비교하지 않는다.**
"돌아간다"와 "측정했다" 사이의 그 빈자리를 채우는 것이 이 저장소다.

## 무엇을 만들었나 — `src/autoware_bench/`

| 구성 | 역할 |
|---|---|
| `metrics_collector` | 주행 1회(경로 SET→ARRIVED 자동 감지)를 long-format CSV로 기록. 평가 지표 94종 + **정지 사유**(`velocity_factors`의 모듈명) + **개별 인지 객체**(거리·속도·분류) |
| `scenario_runner` | YAML 하나로 무인 완주: AWSIM 리셋 → 위치추정 재초기화(**정답 대비 오차 검증 후 출발**) → 경로 → engage → 도착. 장애물 스폰, 신호 강제(빨강→정지 확인→초록) 지원 |
| `run_batch.sh` / `run_ab.sh` | 반복 실행(러너 종료코드로 성공 판정)과 파라미터 A/B |
| `compare_runs.py` | 반복 편차 측정 + A/B 비교. **차이가 반복 편차보다 작으면 "판단 보류"** — 개선/악화를 주장하지 않는다 |
| `check_criteria.py` + `criteria.yaml` | 합격 판정. 임계값은 기본 파라미터 10회 관측치에서 도출 — 근거 없는 숫자를 먼저 박지 않는다 |
| `run_report.py` | 주행 1회를 6분할 그림 + 요약으로. 정지마다 **세운 모듈**과 거리를 표기 |

## 무엇을 측정했나

### MPC 횡오차 가중치의 교환비 (5회씩 + 대조군, `docs/WORKLOG.md` §4)

| `mpc_weight_lat_error` | 횡편차 p95 | 조향속도 p95 |
|---|---:|---:|
| 1.0 (기본) | 6.37 cm | 0.0239 rad/s |
| 5.0 | 4.68 | 0.0357 |
| 20.0 | **3.78 (−41%)** | **0.0646 (+170%)** |

추종 오차를 41% 줄이는 대가는 조향 활동 2.7배. 승차감(저크)에는 측정 가능한 영향 없음.
대조군(기본값 재측정)의 모든 지표가 반복 편차 안 — 변화는 파라미터 때문이다.

### 신호 arbiter는 충돌 시 빨강을 택한다 (§7·§9)

| 외부 입력 | 카메라 인지 | 결과 |
|---|---|---|
| **빨강** | 초록 | 정지 — 빨강 승 |
| 초록 | **빨강** | 정지 — 빨강 승 |
| 빨강 | 시야 밖(미제공) | 외부가 그대로 통과 |

외부 신호 주입으로 "강제 정지"는 만들 수 있어도 "강제 통과"는 못 만든다 —
V2X 스푸핑을 막는 방어적 설계가 측정으로 확인됐다.

### 도심 기능 실험 (STEP 6)

| 실험 | 결과 |
|---|---|
| 장애물 정지 | 앞면 **4.5 m** 정지 → 제거 → 재출발 완주. 더미 객체는 AWSIM 씬에 실물 스폰 — LiDAR→인지→계획 전 파이프라인 검증 |
| 신호등 정지 | 빨강 강제 → 정지선 **0.09 m** 앞 정지 → 초록 → 통과. 자연 빨강 때 정지 위치와 0.23 m 차이로 재현 |
| 앞차 추종 | 접근(3.9 m/s) → 감속 → 앞차 2.0 m/s에 속도 일치, **차간 15.5 m 안정 유지** (개별 객체 추적으로 확인) |
| 교차로 좌회전 | yaw +62° 완주, `intersection` factor 20.6 m 앞부터 평가 |
| 우회전 (gap acceptance) | 양보 차선에서 대향차 평가 → 23 m 앞 `collision stop` 삽입 → **0.6 s 만에 해제**, 멈추지 않고 회전. 같은 교차로의 보호 우회전은 모듈이 아예 개입하지 않는다 — 양보 의무는 좌측통행이 아니라 **지도 `right_of_way` 규제요소의 역할**이 정한다 |

## 왜 이 수치를 믿을 수 있나

1. **반복 편차를 먼저 쟀다.** 같은 조건 5회의 폭(횡편차 RMS 0.59 cm)이 판정 기준선이다.
   그보다 작은 차이는 전부 "판단 보류"로 적는다.
2. **대조군을 뒀다.** 실험 도중 기본값을 재측정해 시간 드리프트가 없음을 확인했다.
3. **모르는 것은 모른다고 적는다.** 사유 미상 정지, NPC가 섞인 지표, 1회짜리 관측은
   전부 그렇게 표기돼 있다 (`docs/WORKLOG.md`).
4. **실패를 성공으로 세는 버그를 잡았다.** 차가 한 발짝도 안 간 주행이 배치에 섞여
   정반대 결론("가중치를 올리면 불안정")이 나올 뻔했다 — 러너 종료코드 판정으로 수정.

## 트러블슈팅 기록

튜토리얼을 따라해서는 만날 수 없는 것들. 전부 증상 → 실제 원인 → 조치로 기록돼 있다.

- **DDS 멀티캐스트 과금 사고** — 받는 사람 없는 152 GB가 인터넷 egress로 과금돼 시간당 $2.30.
  `AllowMulticast=spdp` 한 줄로 $0.002/hr ([PROGRESS.md](PROGRESS.md) 「대역폭 과금 사고」)
- **컨테이너 함정 5종** — venv의 python3 가로채기, 읽기 전용 Vulkan ICD가 ansible을 중단시켜
  ML 모델이 통째로 빠지는 문제, 0바이트 rsync, `SocketReceiveBufferSize`로 노드 전멸,
  fcitx XIM 세그폴트 (PROGRESS.md 「인스턴스 교체 → 전면 재설치」)
- **sim time 스택에 외부 데이터 주입** — 벽시계 스탬프는 조용히 버려진다 (WORKLOG §7)
- **TRANSIENT_LOCAL의 묵은 값** — 새 프로세스가 지난 주행의 상태를 받아 오판한다 (WORKLOG §4)
- **지도 다루기** — 곡선 도로에서 끝점 평균은 차선 밖이다 / 기하 인접 ≠ 라우팅 연결 /
  미션 플래너는 97-lanelet 우회를 경고 없이 수용한다 (WORKLOG §10) →
  러너가 출발 전 경로 길이를 직선거리와 대조해 거른다 (400 s 낭비 → 15 s 중단, §11)
- **재기동 후 조용히 주행 불가** — 러너에는 똑같이 "자율주행 준비 초과"로만 보이는 원인 4종:
  상류 크래시 / 노드 1개 로드 실패 / **속도 상한 발행자 없음** / 구 스택 잔존 노드 중복.
  계획 파이프라인을 상류부터 `topic hz` 로 좁히는 진단 순서까지 기록 (WORKLOG §12)

## 재현

```bash
# 기동 (각 스크립트에 이 환경 고유의 함정 처리가 주석과 함께 들어 있다)
bash scripts/start_awsim.sh        # 이후 Load 자동 클릭: DISPLAY=:20 xdotool mousemove 1394 937 click 1
bash scripts/start_autoware.sh
bash scripts/start_collector.sh

# 주행 1회 (무인)
bash scripts/run_scenario.sh scenarios/01_straight.yaml

# 반복 → 편차 → 판정
bash scripts/run_batch.sh scenarios/01_straight.yaml 5 baseline
python3 src/autoware_bench/scripts/compare_runs.py runs/batch_baseline.txt
python3 src/autoware_bench/scripts/check_criteria.py runs/batch_baseline.txt

# 파라미터 A/B
bash scripts/run_ab.sh scenarios/01_straight.yaml 5 test mpc_weight_lat_error 5.0
python3 src/autoware_bench/scripts/compare_runs.py runs/batch_baseline.txt runs/batch_test.txt
```

전체 설치 절차와 검증 명령은 [RESTART.md](RESTART.md), 작업별 전후 비교는 [docs/WORKLOG.md](docs/WORKLOG.md).
