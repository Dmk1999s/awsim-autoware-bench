# WORKING

앞으로 할 일. 위에서부터 순서대로.

---

## 발견 목록 (2026-08-23 · 문제를 먼저 찾는 방향으로 전환한 뒤)

측정으로 확인한 것만. 각 항목은 WORKLOG 에 재현 절차와 수치가 있다.

| # | 발견 | 성격 | 재현성 |
|---|---|---|---|
| 1 | 외부 신호 주입으로 **빨간불 통과 가능** (§23) | Autoware 동작 — 앞서 적은 반대 결론을 정정 | 3/3 (강제 통과) · **30회 중 4회** (강제 정지 실패, §32) |
| 2 | `behavior_planning` 컨테이너가 새 경로 수신 시 죽음 (§12) | Autoware 상류 버그 | 경로 설정 220여 회 중 4회 |
| 3 | AWSIM 더미 객체 **수명 30초** (§26) | 시뮬레이터 특성 | 2/2 (30.5·30.8 s) |
| 4 | AWSIM 더미 스폰 기능이 **조용히 죽음** (§24) | 시뮬레이터 고장 | 재기동으로만 복구 |
| 5 | 장애물 정지가 늦다 — 10 m 까지 거의 무감속 후 −3.0 m/s² (§31) | Autoware 동작 | 측정 중 (§27 의 −6.14 는 오염값, 철회) |
| 6 | 판정 임계값이 시나리오 간 이식되지 않음 (§28→**§32 해결**) | 우리 하네스 | 03 18회로 자기 분포 확보 → 18/18 합격 |
| 7 | **더미 DELETE 는 uuid 가 아니라 자리를 지운다** (§30) | 시뮬레이터 특성 | 2/2 (점 74→0) |
| 8 | 09 의 판정이 뒤집혀 있었다 — 관통=성공, 정지=실패 (§31) | 우리 하네스 | 고침 |
| 9 | **추적기 입력이 CenterPoint 하나뿐** — 군집이 100% 잡아도 계획은 52% 를 놓친다 (§33) | Autoware 구성 | 161초 연속 관측 |

## 다음 (2026-08-24 새벽 작업이 남긴 것)

- [ ] **추적기에 `lidar_clustering` 채널 붙이기** (§33). 군집은 정지 장애물을 100% 잡는데
      추적기 입력이 `lidar_centerpoint` 하나라 계획은 52% 를 놓친다. 배선이 파라미터가 아니라
      시스템 구조 JSON 에 있고 (`autoware_sample_designs/.../E2ESimulation_2_connections.json`)
      스택 재기동이 필요해 무인으로는 손대지 않았다. **붙인 뒤 09 를 다시 돌리면
      「4/13 회가 0.75 m 안에서 정지」가 사라지는지가 바로 전후 수치가 된다.**
- [ ] CenterPoint 재현율이 AWSIM 더미 특유인지, 실제 차량에도 그런지 (§34 가 못 갈랐다).
      정답 목록이 없으니 기준을 밖에서 가져와야 한다 — 점군 기반 자체 기준이나 신호 대기열
- [ ] `09_obstacle_approach` 를 `run_suite.sh` 목록에 넣기 (지금 7종에 빠져 있다)
- [ ] 리셋 성공 여부를 직접 검증 — 지금은 경로 길이로 간접 확인만 한다

## 무인 운용 (오늘 추가)

- [x] `preflight.sh` — 노드 수 / 중복 노드 / **behavior_planning 생존** / mrm 노드 적재 /
      속도 상한 발행자 / 수집기 / control_cmd 를 확인하고 고칠 수 있는 것은 고친다
- [x] `restart_stack.sh` — 잔존 프로세스까지 정리하고 재기동 (RViz 는 죽이지 않는다)
- [x] `run_batch.sh` 가 회차마다 사전 점검 → 실패하면 재기동 후 계속
- [x] AWSIM 리셋 클릭을 **창 기준 상대좌표**로 (WORKLOG 15) + 경로 20 m 미만이면 중단

## 알아둘 것 — 「자율주행 가능」 깜빡임

NDT 자세가 5~6.5 Hz 밖에 안 나와(기대 10 Hz) ekf_localizer 가 WARN 을 반복하고,
그것이 availability 를 초당 두 번 뒤집는다. **주행은 막지 않지만** 러너의 간헐적
engage 실패가 이것이다. 부하 문제라 RViz 를 끄면 완화된다 (WORKLOG 16).

---

## 최근 추가 — 실시간 모니터 (HMI)

- [x] `dashboard` 노드 + 웹 UI — 속도·모드·기록 상태·감속 사유·자체 모듈 상한·스택 Hz
- [x] 카메라 패널 — 기본 꺼짐, 프리셋(320p/480p/720p)과 **실시간 대역폭 표시** (WORKLOG 14)
- [x] 대역폭 실측 — 지표만 시간당 3.5 MB vs 데스크톱 WebRTC 0.9~4.5 GB
- 주소: 컨테이너 `10100` → vast.ai 매핑 포트. 기동은 `bash scripts/start_dashboard.sh`

---

## 지금 하는 것 — P1 주행 평가 하네스

`planning_evaluator`가 지표를 흘려보내지만 아무도 모으지 않는다. 그 빈자리를 채운다.

- [x] `autoware_bench` 패키지 뼈대 + colcon 빌드
- [x] `metrics_collector` — 주행 1회를 CSV로 기록 (경로 SET→ARRIVED 자동 감지)
- [x] 실주행 1회 검증 — 36,918행 / 94지표 / 38.5초
- [x] **`run_report.py`** — `src/autoware_bench/scripts/run_report.py`
  - 6분할 그림: 속도 프로파일, 경로(x-y, 속도 색), 횡편차, 저크 시계열·분포, 최근접 객체 거리
  - 요약 표: 시간·거리·속도·횡편차(max/p95/RMS)·저크(max/p95)·목표 도착 오차
  - 정지 구간을 **출발 대기 / 도착 정지 / 주행 중 정지**로 구분
  - 3회차 CSV로 검증 — 91.0 m, 38.54 s, |횡편차| max 3.5 cm, 주행 중 정지 0회
- [x] **정지 사유 수집** — `/api/planning/velocity_factors` 구독. 정지 구간마다 세운 모듈 귀속
- [x] **`criteria.yaml` + 판정** — 기본 파라미터 10회 관측치에서 임계값 도출, `check_criteria.py`
- [x] 회차 간 비교 — `compare_runs.py`. 반복 편차를 재고, 편차보다 작은 차이는 "판단 보류"로 적는다
- [x] **파라미터 A/B 1건 완료** — `mpc_weight_lat_error` 1→20: 추종 −41% / 조향 +170% (WORKLOG 4단계)

**남은 정리**

- [x] `.gitignore` — `build/`, `install/`, `log/`, `runs/*.csv`, `reports/`, `collector.log` 제외
- [x] 로컬 git 초기 커밋 (`697d265`)
- [x] **GitHub push** — `github.com/Dmk1999s/awsim-autoware-bench` (master)
      `/workspace`는 `df`상 `/`와 같은 overlay다. 별도 볼륨이 아니라서 인스턴스가 교체되면
      남는다는 보장이 없다. 2026-08-22에 `migration_backup.tar.gz`를 그렇게 잃었다 —
      작업 단위마다 push 할 것

---

## STEP 6 — 도심 기능 실험

각 실험은 P1으로 지표를 남기면서 진행한다. "봤다"가 아니라 "측정했다"가 되도록.

- [x] **실험 1 · 장애물 정지** — 앞면 4.5 m 정지, 제거 후 재출발, PASS (WORKLOG 6)
- [x] **실험 2 · 신호등 정지** — 외부 신호 강제로 재현 가능. 정지선 0.09 m, PASS (WORKLOG 7)
- [~] **실험 3 · 앞차 추종** — 부분: 추종 관측됐으나 차간 지표에 NPC 교통이 섞임.
      개별 객체 기록(`/perception/object_recognition/objects`)이 다음 확장 (WORKLOG 8)
- [x] **실험 4 · 교차로 좌회전** — 완주 + intersection factor 확인.
      부산물: 신호 arbiter 는 충돌 시 빨강 우선 (WORKLOG 9)
- [x] **실험 5 · 우회전** — 보호 우회전(lanelet 332) 3회 완주, 양보 우회전(lanelet 405)에서
      gap acceptance 관측: 23 m 앞 `collision stop` 삽입 → 0.6 s 만에 해제 (WORKLOG 11).
      발견: 양보 의무는 좌측통행이 아니라 지도 right_of_way 규제요소의 역할이 정한다

관련 지도 정보 (조사 완료): 신호등 regulatory element **164개**, `turn_direction` 보유 lanelet **387개**
(left 103 / straight 149 / right 135), 전체 lanelet 979개.

---

## P2 — 시나리오 러너 + 회귀 테스트 (핵심 완료)

- [x] YAML 시나리오 (goal, objects, traffic_override, criteria 덮어쓰기) — `scenarios/` 5종
- [x] 러너 — 리셋 → 위치추정 재초기화(오차 검증) → 경로 → engage → 도착 (`scenario_runner.py`)
- [x] 배치 실행 (`run_batch.sh`, 러너 종료코드로 성공 판정) + A/B (`run_ab.sh`)
- [x] 파라미터 전후 비교 1건 (mpc_weight_lat_error)
- [x] 앞차 특정 차간 추적 — 개별 객체 기록 추가 (WORKLOG 8b)
- [x] 우회전 시나리오 2종 — `06`(보호 회전, 대조군) / `07`(양보 회전, gap acceptance)
- [x] 경로 길이 검증 — 직선거리의 3.0배를 넘으면 출발 전 중단 (WORKLOG 11)
- [ ] 간격이 좁아 실제로 **멈추는** 회차 확보 — 반복 실행 또는 대향차 타이밍 스폰

---

## P3 — behavior_velocity 커스텀 모듈

`autoware_behavior_velocity_template_module`이 공식 템플릿으로 들어있다. 복제해서 자체 모듈 작성.

- [x] 템플릿 구조 파악 (`src/`, `plugins.xml`, `config/`)
- [x] 모듈 주제 — **곡률 기반 선행 감속** (`curve_slowdown`)
- [x] 구현 + `plugins.xml` 등록 + 파라미터 + 런치 등록 (`scripts/register_curve_module.sh`)
- [x] P1으로 before/after — 횡가속 최대 −50%, 횡편차 p95 −41% (WORKLOG 13)
- [x] 선행 거리 스윕 5/20/40 m — **효과 대부분이 5 m 에서 나온다**(가설 반증)
- [x] 다른 시나리오(06·07 우회전)에서도 같은 효과 — 세 시나리오 9개 커브에서 재현 (`d9bbd0b`)
- [x] `max_lateral_accel` 스윕 — 파라미터와 결과가 2배 어긋나는 이유까지 규명 (`6842a27`, WORKLOG 17)

---

## P4 — 한국 지도 파이프라인 (선택)

지역 이슈 해결이 아니라 **"지도 파이프라인까지 다뤘다"**는 별개 가산점으로 접근.
도시 전체가 아니라 교차로 하나 / 캠퍼스 한 블록 규모로.

- [ ] Vector Map Builder로 lanelet2 제작 (우측통행)
- [ ] AWSIM `PointCloudMapping.PointCloudMapper`로 가상 LiDAR 주행 → PCD 생성
- [ ] Unity 3D 환경 제작 후 재빌드

> 좌측통행은 스택 설정이 아니라 지도의 `turn_direction` 속성이다.
> Autoware 전체에 `left_hand` 같은 파라미터가 존재하지 않으며, 한국 지도를 넣으면 코드 수정 없이 우측통행으로 동작한다.
> 이 사실 자체가 README에 쓸 만한 내용.

---

## README에 넣을 것

- [x] 지도 선택 이유 한 줄 — AWSIM 공식 샘플 맵(도쿄 니시신주쿠), 좌측통행은 지도 데이터 속성
- [x] **트러블슈팅 섹션** — `PROGRESS.md`의 환경 진단 4종 + 호스트 포트 포워딩 진단
- [x] "무엇을 바꿨고 무엇을 측정했는가"를 한 줄로

---

## 알려진 자잘한 문제

- **속도 슬라이더** — RViz `Set Velocity Limit`이 원본이라 CLI로 덮어써도 패널이 되돌린다.
  값 변경은 RViz에서 직접 해야 한다
- **RViz 2개 동시 실행** — `:20`(런치, respawn) + `:21`(직접 띄운 것). CPU 각 ~185%.
  여유가 필요하면 Autoware를 `rviz:=false`로 재기동
- **noVNC 스케일링** — 최초 1회 `Settings → Scaling Mode → Local Scaling` 필요
- **`Views` Target Frame** — `viewer`로 바뀌면 카메라가 차를 안 따라간다. `base_link`로 되돌릴 것
- **목적지 클릭 거부** — `Goal's footprint exceeds lane!`. 드래그 방향을 차선과 나란히,
  넓은 직선 차선 중앙을 고를 것
- **재기동 후 주행 불가 4종** — 러너에는 전부 `자율주행 준비 60s 초과` 로만 보인다.
  원인과 진단 경로는 WORKLOG 12. 특히 **속도 상한 발행자가 없으면 계획이 통째로 멈춘다**
  (`scripts/start_velocity_limit.sh` 를 기동 절차에 넣은 이유)

---

## 재시작 절차

```bash
/root/start_awsim.sh &        # AWSIM 창에서 Load 클릭
/root/start_autoware.sh &     # 노드 188개까지 1~2분
/root/start_display21.sh      # :21 + noVNC (비밀번호 /root/.vnc21passwd.txt)
/root/start_rviz21.sh &       # :21 에 RViz
bash /workspace/scripts/start_velocity_limit.sh   # 속도 상한 — 없으면 trajectory 가 안 나온다
/workspace/run_collector.sh & # 지표 수집
```
