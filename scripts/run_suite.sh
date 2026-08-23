#!/bin/bash
# 전 시나리오를 N회씩 돌리고, 실패와 판정 위반을 한 장으로 모은다.
#   run_suite.sh [회차수] [태그]
#
# 목적은 "잘 되는 것을 보여주기"가 아니라 **안 되는 것을 찾기**다. 그래서 성공률만이 아니라
# 아래를 함께 모은다:
#   - 러너 종료코드 (완주 실패의 종류)
#   - criteria.yaml 판정 (시나리오별 덮어쓰기 적용)
#   - 주행 중 정지와 그 사유 모듈
#   - 스택 사고 (preflight 이 잡아 재기동한 횟수)
set -o pipefail
# 이 컨테이너의 /venv/main 이 python3 를 가로챈다 — 그 venv 에는 numpy 가 없어서
# 판정 스크립트가 ModuleNotFoundError 로 죽는다 (PROGRESS.md 진단 #3). 시스템 파이썬을 쓴다.
unset VIRTUAL_ENV
export PATH="/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

N="${1:-3}"; TAG="${2:-suite}"
OUT=/workspace/reports/${TAG}
mkdir -p "$OUT"
LOG="$OUT/run.log"
: > "$LOG"

SCENARIOS=(01_straight 02_obstacle 03_traffic_light 04_follow 05_left_turn 06_right_turn 07_right_turn_yield)

echo "# 시나리오 스위트 — $(date -u +%Y-%m-%d\ %H:%M) UTC, 각 ${N}회" | tee "$OUT/summary.md"
echo "" >> "$OUT/summary.md"
echo "| 시나리오 | 성공 | 판정 | 비고 |" >> "$OUT/summary.md"
echo "|---|---:|---|---|" >> "$OUT/summary.md"

for sc in "${SCENARIOS[@]}"; do
  echo "===== $sc =====" | tee -a "$LOG"
  bash /workspace/scripts/run_batch.sh "/workspace/scenarios/${sc}.yaml" "$N" "${TAG}_${sc}" 2>&1 | tee -a "$LOG"
  LIST=/workspace/runs/batch_${TAG}_${sc}.txt
  OK=$(wc -l < "$LIST" 2>/dev/null || echo 0)
  VERDICT="—"
  if [ "$OK" -gt 0 ]; then
    VERDICT=$(python3 /workspace/src/autoware_bench/scripts/check_criteria.py "$LIST" \
                --scenario "/workspace/scenarios/${sc}.yaml" 2>&1 | tail -1)
  fi
  NOTE=$(grep -c "실패 (종료코드" "$LOG" 2>/dev/null || echo 0)
  echo "| \`$sc\` | ${OK}/${N} | ${VERDICT} | |" >> "$OUT/summary.md"
done

echo "" >> "$OUT/summary.md"
echo "재기동 횟수: $(grep -c '사전 점검 실패' "$LOG" 2>/dev/null || echo 0)" >> "$OUT/summary.md"
echo "=== 스위트 완료 → $OUT/summary.md ==="
cat "$OUT/summary.md"
