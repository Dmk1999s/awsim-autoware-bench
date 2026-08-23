#!/bin/bash
# traffic_light_arbiter 의 source_priority 를 바꾼다.  사용: set_signal_priority.sh external|confidence
#
# 왜 필요한가: 주행 시간의 반복 편차가 30~55 초인데 그 대부분이 신호 대기다. 이 노이즈가
# 비용(시간) 비교를 통째로 "판단 보류"로 만든다. 외부 신호를 우선하게 하면 신호를 완전히
# 통제할 수 있어 회차가 반복 가능해진다.
#
# 주의 — 이건 **측정 전용 설정**이다. 기본값 "confidence" 에서는 카메라가 빨강을 보면
# 빨강이 이긴다(WORKLOG 9). 그 규칙이 외부 신호 주입으로 "강제 통과"를 만들 수 없게 막는
# 방어선인데, "external" 은 그 방어선을 끈다. 신호 자체를 다루는 실험(실험 2·4)이나
# 안전 성질을 논할 때는 반드시 confidence 로 되돌릴 것.
#
# 파라미터는 노드 생성 시점에만 읽으므로 바꾼 뒤 스택을 재기동해야 한다.
set -eo pipefail
MODE="${1:-}"
case "$MODE" in
  external|confidence|perception) ;;
  *) echo "사용: $0 external|confidence|perception"; exit 1 ;;
esac

YAML=/root/autoware/src/launcher/autoware_launch/autoware_launch/config/perception/traffic_light_recognition/traffic_light_arbiter/traffic_light_arbiter.param.yaml
cp -n "$YAML" "$YAML.orig"
sed -i -E "s|^(\s*source_priority:\s*)\"[a-z]+\"|\1\"$MODE\"|" "$YAML"
grep -n "source_priority" "$YAML"
echo "→ 스택을 재기동해야 반영된다: bash scripts/restart_stack.sh"
