#!/bin/bash
# 시나리오 하나를 N회 반복한다.  예: run_batch.sh scenarios/01_straight.yaml 5 [태그]
#
# 반복의 목적은 "같은 조건에서 얼마나 흔들리는가"를 재는 것이다.
# 이 편차를 모르면 파라미터를 바꿨을 때 그 차이가 의미 있는지 판단할 수 없다.
set -eo pipefail
# 배치가 조용히 죽는 일이 있었다 — 회차는 완주했는데 목록에 안 적히고 프로세스가 사라졌다.
# set -e 는 죽은 자리를 안 알려주므로, 어디서 왜 끝났는지 남긴다.
trap 'rc=$?; [ $rc -ne 0 ] && echo "!! run_batch 종료: 라인 $LINENO, 코드 $rc" >&2' EXIT

SCENARIO="$1"; N="${2:-5}"; TAG="${3:-$(basename "$SCENARIO" .yaml)}"
OUT=/workspace/runs/batch_${TAG}.txt
: > "$OUT"

for i in $(seq 1 "$N"); do
  echo "=== [$i/$N] $(date -u +%H:%M:%S) ==="
  # 회차마다 사전 점검. 상류 크래시(behavior_planning)나 노드 로드 실패가 나면
  # 남은 회차가 전부 같은 이유로 실패한다 — 실측 2회, 각각 배치의 뒤쪽을 통째로 날렸다.
  if ! bash /workspace/scripts/preflight.sh > /tmp/preflight.log 2>&1; then
    echo "  ! 사전 점검 실패 — 스택을 재기동한다"
    cat /tmp/preflight.log | sed 's/^/    /'
    if ! bash /workspace/scripts/restart_stack.sh 2>&1 | sed 's/^/    /'; then
      echo "  ✗ 재기동 후에도 점검 실패 — 배치를 중단한다"
      break
    fi
  fi
  BEFORE=$(ls -1 /workspace/runs/run_*.csv 2>/dev/null | wc -l)
  # 러너가 실패해도 수집기는 CSV 를 남긴다 (경로 SET 은 됐으므로).
  # CSV 존재로 성공을 세면 차가 한 발짝도 안 간 주행이 배치에 섞인다 — 실제로 그렇게 셌었다.
  # 러너의 종료코드로 판정한다.
  set +e
  bash /workspace/scripts/run_scenario.sh "$SCENARIO" 2>&1 \
    | grep -E 'scenario_runner\]' | sed 's/.*\[scenario_runner\]: //'
  RC=${PIPESTATUS[0]}
  set -e
  sleep 4
  # 파이프 안에서 head 가 먼저 닫히면 pipefail+set -e 로 스크립트가 죽을 수 있다.
  # 실패해도 배치는 계속돼야 하므로 실패를 흡수한다.
  AFTER=$(ls -1 /workspace/runs/run_*.csv 2>/dev/null | wc -l || true)
  NEW=$(ls -1t /workspace/runs/run_*.csv 2>/dev/null | head -1 || true)
  [ -n "$AFTER" ] || AFTER=$BEFORE
  if [ "$RC" -eq 0 ] && [ "$AFTER" -gt "$BEFORE" ]; then
    echo "$NEW" >> "$OUT"
    echo "  → $(basename "$NEW")"
  else
    echo "  → 실패 (종료코드 $RC) — 배치에서 제외"
    [ "$AFTER" -gt "$BEFORE" ] && echo "$NEW" >> "${OUT%.txt}_failed.txt"
  fi
done
echo "=== 완료: $(wc -l < "$OUT")/$N 회 성공 → $OUT ==="
