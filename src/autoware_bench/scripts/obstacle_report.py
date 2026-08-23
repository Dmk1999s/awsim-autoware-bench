#!/usr/bin/env python3
"""장애물 접근 구간을 회차별로 뽑고, 회차 간 편차를 같이 적는다.

편차를 모르면 한 회차의 값을 대표값으로 쓰게 된다 (WORKLOG 27·30).
그래서 항상 여러 회차를 받아 중앙값과 범위를 함께 낸다.

핵심 축은 **언제 잡았는가**다. 같은 장애물인데 50 m 밖부터 잡는 회차와 10 m 안에서야
잡는 회차가 있고, 감속의 세기와 정지 간격이 그것으로 갈린다.

    obstacle_report.py --obstacle 81422.11,49946.84 runs/run_*.csv
"""

import argparse
import bisect
import csv
import math
import statistics

MARKS = [40, 30, 20, 15, 10, 7, 5]
MATCH_M = 2.5      # 기하 거리와 이만큼 안에서 일치하면 그 장애물로 본다
FRONT_M = 3.79     # base_link → 앞범퍼 (wheel_base 2.79 + front_overhang 1.0)


def load(path, ox, oy):
    ego, acc, jerk, objs, near = [], [], [], {}, []
    for r in csv.DictReader(open(path)):
        t, s, n, v = float(r["t"]), r["source"], r["name"], r["value"]
        if s == "ego" and n == "x":
            ego.append([t, float(v), None, None])
        elif s == "ego" and n == "y" and ego:
            ego[-1][2] = float(v)
        elif s == "ego" and n == "vel" and ego:
            ego[-1][3] = float(v)
        elif s == "control" and n == "acceleration":
            acc.append((t, float(v)))
        elif s == "control" and n == "jerk":
            jerk.append((t, float(v)))
        elif s == "control" and n == "closest_object_distance":
            near.append((t, float(v)))
        elif s == "object" and n.endswith("/dist"):
            objs.setdefault(round(t, 2), []).append(float(v))
    ego = [(t, math.hypot(x - ox, y - oy), v) for t, x, y, v in ego
           if x is not None and y is not None and v is not None]
    return ego, acc, jerk, objs, near


def summarize(path, ox, oy):
    ego, acc, jerk, objs, near = load(path, ox, oy)
    if not ego:
        return None
    moving = [t for t, d, v in ego if abs(v) > 0.2]
    if not moving:
        return None
    t0 = moving[0]
    app = [(t, d, v) for t, d, v in ego if t >= t0]
    tmin = min(app, key=lambda r: r[1])[0]
    seg = [(t, d, v) for t, d, v in app if t <= tmin]

    # 인지 여부: 그 순간 객체 목록에 기하 거리와 맞는 것이 있는가.
    # (수집기는 거리만 남기므로 위치 대신 거리로 맞춘다. 같은 거리의 다른 객체가
    #  섞일 수 있어 절대적이지는 않지만, 접근 축 위의 물체는 사실상 이것뿐이다.)
    # 자세는 50 Hz, 인지는 9 Hz 로 들어온다. 같은 시각으로 찾으면 대부분 빈손이 되므로
    # 가장 가까운 인지 샘플을 쓴다 — 이걸 안 해서 인지율이 1/5 로 나왔었다.
    otimes = sorted(objs)

    def seen(t, d):
        i = bisect.bisect_left(otimes, t)
        cand = [otimes[j] for j in (i - 1, i) if 0 <= j < len(otimes)]
        cand = [c for c in cand if abs(c - t) < 0.2]
        return any(abs(o - d) < MATCH_M for c in cand for o in objs[c])

    # 자차와 장애물 사이에 다른 물체가 있었는가 (가림). 접근 축 위의 3 m ~ d-3 m 구간.
    def blocked(t, d):
        i = bisect.bisect_left(otimes, t)
        cand = [otimes[j] for j in (i - 1, i) if 0 <= j < len(otimes)]
        cand = [c for c in cand if abs(c - t) < 0.2]
        return any(3.0 < o < d - 3.0 for c in cand for o in objs[c])

    marks = [(t, d, v, seen(t, d)) for t, d, v in seg]
    approach = [m for m in marks if m[1] <= 50]
    seen_frac = (sum(1 for m in approach if m[3]) / len(approach)) if approach else None
    occl = (sum(1 for t, d, v in seg if d <= 50 and blocked(t, d)) / len(approach)
            if approach else None)

    # 마지막까지 이어지는 인지가 시작된 거리 — "얼마나 멀리서부터 붙잡았나".
    # 1 초 미만의 끊김은 같은 붙잡음으로 본다 (인지는 9 Hz 라 한두 프레임은 늘 빠진다).
    first_hold, run_start, last_ok = None, None, None
    for t, d, v, ok in approach:
        if ok:
            if run_start is None or (last_ok is not None and t - last_ok > 1.0):
                run_start = d
            last_ok = t
            first_hold = run_start
    # 정지: 접근이 끝난 뒤 3초 이상 멈춰 있는 구간
    stop_gap = None
    tail = [(t, d, v) for t, d, v in app if t >= tmin]
    still = [t for t, d, v in tail if abs(v) < 0.1]
    if len(still) > 20:
        ts0, ts1 = still[0], still[-1]
        g = [x for t, x in near if ts0 <= t <= ts1]
        stop_gap = statistics.median(g) if g else None

    a_win = [a for t, a in acc if t0 <= t <= tmin]
    j_win = [j for t, j in jerk if t0 <= t <= tmin]
    at = {}
    for m in MARKS:
        cand = [(abs(d - m), v) for _, d, v in seg if abs(d - m) < 1.5]
        at[m] = min(cand)[1] if cand else None
    return dict(
        run=path.split("/")[-1].replace("run_", "").replace(".csv", ""),
        a_min=min(a_win) if a_win else None,
        j_max=max((abs(j) for j in j_win), default=None),
        v_max=max(v for _, _, v in seg),
        seen_frac=seen_frac,
        occl=occl,
        hold_from=first_hold,
        stop_gap=stop_gap,
        at=at,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csvs", nargs="+")
    ap.add_argument("--obstacle", required=True, help="x,y (map)")
    a = ap.parse_args()
    ox, oy = (float(v) for v in a.obstacle.split(","))
    rows = [r for r in (summarize(p, ox, oy) for p in a.csvs) if r]
    if not rows:
        raise SystemExit("쓸 수 있는 회차가 없다")

    def f(v, fmt="{:.2f}"):
        return fmt.format(v) if v is not None else "—"

    print(f"{'회차':<16}{'인지율':>7}{'가림':>6}{'붙잡은거리':>11}{'최고속':>7}"
          f"{'최대감속':>9}{'최대저크':>9}{'정지간격':>9}")
    for r in sorted(rows, key=lambda r: -(r["hold_from"] or 0)):
        print(f"{r['run']:<16}{f(r['seen_frac'] and r['seen_frac']*100, '{:.0f}%'):>7}"
              f"{f(r['occl'] and r['occl']*100, '{:.0f}%'):>6}"
              f"{f(r['hold_from'], '{:.1f} m'):>11}{f(r['v_max']):>7}"
              f"{f(r['a_min']):>9}{f(r['j_max']):>9}{f(r['stop_gap'], '{:.2f} m'):>9}")

    def spread(key, sel=None):
        vals = [r[key] for r in (sel or rows) if r[key] is not None]
        if len(vals) < 2:
            return f"{vals[0]:.2f}" if vals else "—"
        return (f"중앙값 {statistics.median(vals):6.2f}   범위 {min(vals):6.2f} ~ {max(vals):6.2f}"
                f"   표준편차 {statistics.stdev(vals):.2f}")

    early = [r for r in rows if (r["hold_from"] or 0) >= 25]
    late = [r for r in rows if r["hold_from"] is not None and r["hold_from"] < 25]
    print(f"\n  전체 {len(rows)}회")
    for label, key in [("최대 감속 [m/s²]", "a_min"), ("최대 저크 [m/s³]", "j_max"),
                       ("정지 간격 [m]", "stop_gap"), ("접근 최고속 [m/s]", "v_max")]:
        print(f"    {label:<18} {spread(key)}")
    for name, sel in [("25 m 밖에서 붙잡은 회차", early), ("25 m 안에서야 붙잡은 회차", late)]:
        if not sel:
            continue
        print(f"\n  {name} — {len(sel)}회")
        for label, key in [("최대 감속 [m/s²]", "a_min"), ("정지 간격 [m]", "stop_gap")]:
            print(f"    {label:<18} {spread(key, sel)}")

    print("\n  장애물까지 거리별 속도 [m/s]")
    print(f"   {'회차':<16}" + "".join(f"{m:>7} m" for m in MARKS))
    for r in sorted(rows, key=lambda r: -(r["hold_from"] or 0)):
        print(f"   {r['run']:<16}" + "".join(
            (f"{r['at'][m]:>9.2f}" if r["at"][m] is not None else f"{'—':>9}") for m in MARKS))


if __name__ == "__main__":
    main()
