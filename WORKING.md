# WORKING

앞으로 할 일. 위에서부터 순서대로.

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
- [ ] **GitHub 저장소 생성 후 push — 우선순위 상**
      `/workspace`는 `df`상 `/`와 같은 overlay다. 별도 볼륨이 아니라서 인스턴스가 교체되면
      남는다는 보장이 없다. 2026-08-22에 `migration_backup.tar.gz`를 그렇게 잃었다

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

- [ ] 템플릿 구조 파악 (`src/`, `plugins.xml`, `config/`)
- [ ] 모듈 주제 결정 — 스쿨존 감속 / 특정 lanelet 태그 서행 / 정지선 여유 확보 등
- [ ] 구현 + `plugins.xml` 등록 + 파라미터
- [ ] P1으로 before/after 정량 비교

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
