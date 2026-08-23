#!/usr/bin/env python3
"""주행 CSV(1개 또는 배치 목록)를 criteria.yaml 로 판정한다.

사용:
    check_criteria.py runs/run_xxx.csv
    check_criteria.py runs/batch_baseline.txt [--criteria criteria.yaml] [--scenario scenarios/x.yaml]

종료코드: 전부 합격 0, 하나라도 불합격 1. (CI/배치에서 그대로 쓴다)
"""

import argparse
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).parent))
from compare_runs import summarize, read_list  # noqa: E402

LABELS = {
    "lat_p95": ("수렴후 |횡편차| p95", "cm"),
    "lat_rms": ("수렴후 |횡편차| RMS", "cm"),
    "jerk_p95": ("|저크| p95", "m/s³"),
    "settle_t": ("수렴 시간", "s"),
    "goal_lon": ("목표 오차(종)", "m"),
    "mid_stops": ("주행 중 정지", "회"),
}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("runs")
    p.add_argument("--criteria", default="/workspace/criteria.yaml")
    p.add_argument("--scenario", help="시나리오 yaml — criteria: 블록이 있으면 덮어쓴다")
    args = p.parse_args()

    crit = yaml.safe_load(open(args.criteria))
    limits = dict(crit.get("max", {}))
    mins = dict(crit.get("min", {}))
    if args.scenario:
        sc = yaml.safe_load(open(args.scenario)).get("criteria", {})
        limits.update(sc.get("max", {}))
        mins.update(sc.get("min", {}))

    failed = 0
    for f in read_list(args.runs):
        r = summarize(f)
        rows = []
        ok_all = True
        for key, limit in limits.items():
            v = r.get(key)
            ok = v is not None and v == v and v <= limit
            ok_all &= ok
            name, unit = LABELS.get(key, (key, ""))
            rows.append(f"  {'✅' if ok else '❌'} {name:<20} {v:8.2f} ≤ {limit:g} {unit}")
        for key, limit in mins.items():
            v = r.get(key)
            ok = v is not None and v == v and v >= limit
            ok_all &= ok
            name, unit = LABELS.get(key, (key, ""))
            rows.append(f"  {'✅' if ok else '❌'} {name:<20} {v:8.2f} ≥ {limit:g} {unit}")
        print(f"{'PASS' if ok_all else 'FAIL'}  {r['name']}")
        for line in rows:
            print(line)
        failed += 0 if ok_all else 1

    print(f"\n{'전부 합격' if not failed else f'{failed}건 불합격'}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
