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
- [ ] **정지 사유 수집 — 우선순위 최상** — "어느 모듈이 왜 세웠는지"는
      `/planning/velocity_factors`에 있고 `metrics_collector`가 아직 구독하지 않는다.
      2026-08-22 검증 주행에서 **주행 중 정지가 5회(최장 21.2초)** 나왔는데
      전부 이유를 모른 채로 남았다. STEP 6(신호등·장애물·교차로)은 이것 없이는 측정이 안 된다
- [ ] **`criteria.yaml` + 판정** — 임계값은 3회 이상 주행 데이터를 본 뒤 정한다.
      근거 없는 숫자를 먼저 박지 않기 위함
- [ ] 회차 간 비교 — "이번 주행이 지난번보다 나아졌나"에 답하기

**남은 정리**

- [x] `.gitignore` — `build/`, `install/`, `log/`, `runs/*.csv`, `reports/`, `collector.log` 제외
- [x] 로컬 git 초기 커밋 (`697d265`)
- [ ] **GitHub 저장소 생성 후 push — 우선순위 상**
      `/workspace`는 `df`상 `/`와 같은 overlay다. 별도 볼륨이 아니라서 인스턴스가 교체되면
      남는다는 보장이 없다. 2026-08-22에 `migration_backup.tar.gz`를 그렇게 잃었다

---

## STEP 6 — 도심 기능 실험

각 실험은 P1으로 지표를 남기면서 진행한다. "봤다"가 아니라 "측정했다"가 되도록.

- [ ] **실험 1 · 장애물 정지** — `2D Dummy Car`를 전방 30~50m에 소환.
      virtual wall이 서는지, 어느 모듈이 세웠는지 확인
- [ ] **실험 2 · 신호등 정지** — 신호 교차로를 통과하는 경로.
      `RecognitionResultOnImage`에 신호등 박스 → 빨간불 정지선 정지 → 초록불 출발 전환 관찰
- [ ] **실험 3 · 앞차 추종** — AWSIM `☰` → Traffic Control · Play 켜고 NPC 교통 속 주행.
      차간거리 유지, 앞차 감속 추종, `Perception`의 NPC 예측 경로 확인
- [ ] **실험 4 · 교차로 판단** — `2D Checkpoint Pose`로 좌회전 포함 경로 구성.
      `Debug` 켜서 `intersection` / `blind_spot` 모듈 판단 근거 확인

관련 지도 정보 (조사 완료): 신호등 regulatory element **164개**, `turn_direction` 보유 lanelet **387개**
(left 103 / straight 149 / right 135), 전체 lanelet 979개.

---

## P2 — 시나리오 러너 + 회귀 테스트

`scenario_simulator_v2`는 **설치돼 있지 않다** (adapter 패키지만 존재).
표준 OpenSCENARIO를 도입하는 것보다, 직접 만드는 쪽이 코드량 대비 포폴 가치가 높다.

- [ ] YAML 시나리오 정의 (초기 pose, goal, NPC 스폰 시각·위치·속도, 기대 조건)
- [ ] 러너 — ADAPI로 초기화 → 목적지 설정 → engage → P1으로 지표 수집
- [ ] N개 시나리오 배치 실행 + 리포트
- [ ] 파라미터 변경 전후 비교 (회귀 감지)

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

- [ ] 지도 선택 이유 한 줄 — AWSIM 공식 샘플 맵(도쿄 니시신주쿠), 좌측통행은 지도 데이터 속성
- [ ] **트러블슈팅 섹션** — `PROGRESS.md`의 환경 진단 4종 + 호스트 포트 포워딩 진단.
      튜토리얼 따라한 사람은 쓸 수 없는 내용이라 차별점이 된다
- [ ] "무엇을 바꿨고 무엇을 측정했는가"를 한 줄로

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

---

## 재시작 절차

```bash
/root/start_awsim.sh &        # AWSIM 창에서 Load 클릭
/root/start_autoware.sh &     # 노드 188개까지 1~2분
/root/start_display21.sh      # :21 + noVNC (비밀번호 /root/.vnc21passwd.txt)
/root/start_rviz21.sh &       # :21 에 RViz
/workspace/run_collector.sh & # 지표 수집
```
