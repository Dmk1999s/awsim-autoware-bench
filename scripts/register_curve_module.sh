#!/bin/bash
# 자체 모듈(curve_slowdown)을 Autoware 의 behavior_velocity_planner 에 등록한다.
#
# 이 파일은 Autoware 쪽 런치 XML 을 건드리므로 우리 저장소의 코드가 아니다 —
# 그래서 "패치를 코드로" 남긴다. 몇 번 돌려도 결과가 같다(멱등).
# install/ 의 런치 파일은 src/ 로 향하는 심볼릭 링크라 재빌드는 필요 없다.
set -eo pipefail

XML=/root/autoware/src/launcher/autoware_launch/tier4_universe_launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml
PARAM=/workspace/install/autoware_behavior_velocity_curve_slowdown_module/share/autoware_behavior_velocity_curve_slowdown_module/config/curve_slowdown.param.yaml

[ -f "$PARAM" ] || { echo "먼저 빌드해야 한다: colcon build --packages-select autoware_behavior_velocity_curve_slowdown_module"; exit 1; }

if grep -q "CurveSlowdownModulePlugin" "$XML"; then
  echo "이미 등록돼 있다: $XML"
  exit 0
fi
cp -n "$XML" "$XML.orig"   # 최초 1회만 원본 보관

python3 - "$XML" "$PARAM" <<'PY'
import sys
xml, param = sys.argv[1], sys.argv[2]
s = open(xml).read()

# 1) 플러그인을 모듈 목록에 덧붙인다 (목록을 만들기 시작하는 arg 바로 뒤)
anchor = '  <arg name="behavior_velocity_planner_launch_modules" default="["/>\n'
assert anchor in s, "모듈 목록 arg 를 못 찾았다 — Autoware 버전이 달라졌을 수 있다"
plugin = (anchor +
          '  <!-- 자체 모듈: 곡률 기반 선행 감속 (/workspace/src) -->\n'
          '  <let\n'
          '    name="behavior_velocity_planner_launch_modules"\n'
          '    value="$(eval &quot;\'$(var behavior_velocity_planner_launch_modules)\' + '
          '\'autoware::behavior_velocity_planner::CurveSlowdownModulePlugin, \'&quot;)"\n'
          '  />\n')
s = s.replace(anchor, plugin, 1)

# 2) 파라미터 파일을 노드에 물린다 (템플릿 모듈 주석 자리 옆)
anchor2 = '      <!-- <param from="$(var template_param_path)"/> -->\n'
assert anchor2 in s, "파라미터 삽입 위치를 못 찾았다"
s = s.replace(anchor2, anchor2 + f'      <param from="{param}"/>\n', 1)

open(xml, "w").write(s)
print("패치 완료")
PY
grep -n "CurveSlowdown\|curve_slowdown" "$XML"
