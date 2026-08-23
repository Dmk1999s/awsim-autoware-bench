#!/bin/bash
# 시나리오 하나를 N회 반복한다.  예: run_batch.sh scenarios/01_straight.yaml 5 [태그]
#
# 반복의 목적은 "같은 조건에서 얼마나 흔들리는가"를 재는 것이다.
# 이 편차를 모르면 파라미터를 바꿨을 때 그 차이가 의미 있는지 판단할 수 없다.
set -eo pipefail
SCENARIO="$1"; N="${2:-5}"; TAG="${3:-$(basename "$SCENARIO" .yaml)}"
OUT=/workspace/runs/batch_${TAG}.txt
: > "$OUT"

for i in $(seq 1 "$N"); do
  echo "=== [$i/$N] $(date -u +%H:%M:%S) ==="
  BEFORE=$(ls -1 /workspace/runs/run_*.csv 2>/dev/null | wc -l)
  if bash /workspace/scripts/run_scenario.sh "$SCENARIO" 2>&1 | grep -E 'scenario_runner\]' | sed 's/.*\[scenario_runner\]: //'; then
    :
  fi
  sleep 4
  AFTER=$(ls -1 /workspace/runs/run_*.csv 2>/dev/null | wc -l)
  if [ "$AFTER" -gt "$BEFORE" ]; then
    NEW=$(ls -1t /workspace/runs/run_*.csv | head -1)
    echo "$NEW" >> "$OUT"
    echo "  → $(basename "$NEW")"
  else
    echo "  → CSV 없음 (실패)"
  fi
done
echo "=== 완료: $(wc -l < "$OUT")/$N 회 성공 → $OUT ==="
