#!/usr/bin/env python3
"""여러 주행을 나란히 놓고 비교한다.

두 가지 용도가 있고 둘 다 같은 표로 답한다.

  1. 반복 편차   — 같은 시나리오를 N회 돌려 "가만히 둬도 얼마나 흔들리는가"를 잰다.
                   이 값을 모르면 2번의 차이가 의미 있는지 판단할 수 없다.
  2. A/B 비교    — 파라미터를 바꾸기 전후를 비교한다. 차이가 1번의 편차보다 커야
                   "바꿔서 달라졌다"고 말할 수 있다.

사용:
    compare_runs.py runs/batch_baseline.txt                    # 편차
    compare_runs.py runs/batch_before.txt runs/batch_after.txt # A/B
"""

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

# 비교할 지표. (표시이름, 계산함수, 단위, 낮을수록 좋은가)
# 자율주행 품질을 판단하는 데 실제로 쓰이는 것만 고른다.
METRICS = [
    ("주행 시간",          lambda r: r["duration"],      "s",     True),
    ("주행 거리",          lambda r: r["distance"],      "m",     None),
    ("평균 속도",          lambda r: r["mean_speed"],    "m/s",   False),
    ("스폰 오프셋",        lambda r: r["spawn_offset"],  "cm",    True),
    ("수렴 시간",          lambda r: r["settle_t"],      "s",     True),
    ("|횡편차| p95(수렴후)", lambda r: r["lat_p95"],     "cm",    True),
    ("|횡편차| RMS(수렴후)", lambda r: r["lat_rms"],     "cm",    True),
    ("|저크| p95",         lambda r: r["jerk_p95"],      "m/s³",  True),
    ("최근접 객체 최소",   lambda r: r["min_obj"],       "m",     False),
    ("목표 오차(종)",      lambda r: r["goal_lon"],      "m",     True),
    ("주행 중 정지",       lambda r: r["mid_stops"],     "회",    True),
    ("정지 총 시간",       lambda r: r["stop_time"],     "s",     True),
]

STOP_VEL = 0.1
STOP_MIN_SEC = 0.5


def load(path):
    series = defaultdict(lambda: ([], []))
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            t, v = series[(row["source"], row["name"])]
            t.append(float(row["t"]))
            v.append(float(row["value"]))
    return {k: (np.array(t), np.array(v)) for k, (t, v) in series.items()}


def get(series, source, name):
    return series.get((source, name), (np.array([]), np.array([])))


def drive_window(series):
    """자율주행 구간의 시작 시각. 없으면 0.

    기록에는 engage 전 대기가 섞여 있고, 그 구간의 자세 오차는 스폰 위치 탓이지
    추종 품질이 아니다. 품질 지표는 이 시점 이후만 봐야 한다.
    """
    t, mode = series.get(("system", "operation_mode"), (np.array([]), np.array([])))
    auto = t[mode == 2]
    return float(auto[0]) if len(auto) else 0.0


def clip(t, v, t0):
    sel = t >= t0
    return t[sel], v[sel]


SETTLE_M = 0.10   # |횡편차| 가 이 아래로 내려오면 수렴한 것으로 본다


def settle_point(t, lat):
    """스폰 오프셋을 되찾기까지의 과도구간 끝.

    차는 차선 중앙에서 벗어난 자세로 스폰된다(실측 28.6 cm). 그 오프셋을 되찾는
    구간이 |횡편차| p95 를 통째로 지배해서, 그대로 재면 추종 품질이 아니라
    스폰 자세를 재게 된다 — 반복 5회 모두 p95 가 28.5~29.0 cm 로 같았던 이유다.
    임의로 "N초 이후"를 자르는 대신 수렴 시점을 찾고, 과도구간 자체도 지표로 남긴다.
    """
    if not len(lat):
        return 0.0, float("nan")
    offset = abs(lat[0]) * 100.0
    below = np.where(np.abs(lat) < SETTLE_M)[0]
    return (float(t[below[0]]), offset) if len(below) else (float(t[-1]), offset)


def summarize(path):
    """주행 1회를 숫자 한 줄로 줄인다."""
    s = load(path)
    t_auto = drive_window(s)
    t_v, vel = get(s, "ego", "vel")
    _, x = get(s, "ego", "x")
    _, y = get(s, "ego", "y")
    t_lat_all, lat_all = get(s, "control", "lateral_deviation")
    t_settle, spawn_offset = settle_point(t_lat_all, lat_all)

    # 추종 품질은 수렴 이후만 본다. 과도구간은 위에서 따로 지표로 뽑았다.
    _, lat = clip(t_lat_all, lat_all, t_settle)
    _, jerk = clip(*get(s, "control", "jerk"), max(t_auto, t_settle))
    _, obj = clip(*get(s, "control", "closest_object_distance"), t_auto)
    _, goal = get(s, "control", "goal_longitudinal_deviation_abs")

    duration = float(t_v[-1]) if len(t_v) else float("nan")
    moving = vel[vel >= STOP_VEL] if len(vel) else np.array([])

    # 정지 구간. 출발 대기와 도착 정지는 주행 품질이 아니므로 뺀다.
    stopped = vel < STOP_VEL if len(vel) else np.array([], bool)
    events, start = [], None
    for i, st in enumerate(stopped):
        if st and start is None:
            start = t_v[i]
        elif not st and start is not None:
            if t_v[i] - start >= STOP_MIN_SEC:
                events.append((start, t_v[i]))
            start = None
    if start is not None and duration - start >= STOP_MIN_SEC:
        events.append((start, duration))
    mid = [e for e in events if e[0] > 0.01 and e[1] < duration - 0.01]

    return {
        "name": Path(path).stem,
        "t_auto": t_auto,
        "spawn_offset": spawn_offset,
        "settle_t": t_settle,
        "duration": duration,
        "distance": float(np.sum(np.hypot(np.diff(x), np.diff(y)))) if len(x) > 1 else float("nan"),
        "mean_speed": float(moving.mean()) if len(moving) else float("nan"),
        "lat_p95": float(np.percentile(np.abs(lat), 95) * 100) if len(lat) else float("nan"),
        "lat_rms": float(np.sqrt((lat ** 2).mean()) * 100) if len(lat) else float("nan"),
        "jerk_p95": float(np.percentile(np.abs(jerk), 95)) if len(jerk) else float("nan"),
        "min_obj": float(obj.min()) if len(obj) else float("nan"),
        "goal_lon": float(goal[-1]) if len(goal) else float("nan"),
        "mid_stops": float(len(mid)),
        "stop_time": float(sum(e - s for s, e in mid)),
        "blamed": blame_counts(s, mid),
    }


def blame_counts(series, mid_events):
    """주행 중 정지를 낸 모듈별 횟수."""
    counts = defaultdict(int)
    for s0, s1 in mid_events:
        for (src, name), (t, v) in series.items():
            if src != "factor" or not name.endswith("/status"):
                continue
            sel = (t >= s0 - 0.5) & (t <= s1 + 0.5) & (v == 2)
            if sel.any():
                counts[name[: -len("/status")]] += 1
    return dict(counts)


def read_list(path):
    p = Path(path)
    if p.suffix == ".txt":
        return [l.strip() for l in p.read_text().splitlines() if l.strip()]
    return [str(p)]


def fmt(v, unit):
    if v != v:  # NaN
        return "—"
    return f"{v:.2f}" if unit != "회" else f"{v:.0f}"


def spread_table(runs, title):
    """N회 반복의 편차. 이 폭이 A/B 판정의 기준선이 된다."""
    L = [f"## {title} — {len(runs)}회", "",
         "| 지표 | 평균 | 표준편차 | 최소 | 최대 | 폭(최대-최소) |",
         "|---|---:|---:|---:|---:|---:|"]
    for label, f, unit, _ in METRICS:
        vals = np.array([f(r) for r in runs], dtype=float)
        vals = vals[~np.isnan(vals)]
        if not len(vals):
            continue
        L.append(f"| {label} [{unit}] | {vals.mean():.2f} | {vals.std():.2f} | "
                 f"{vals.min():.2f} | {vals.max():.2f} | {vals.max()-vals.min():.2f} |")
    return L


def ab_table(a, b, label_a, label_b):
    """A/B 비교. 차이가 A 쪽 편차보다 작으면 '판단 보류'로 적는다 — 노이즈와 구분되지 않는다."""
    L = ["## A/B 비교", "",
         f"A = {label_a} ({len(a)}회) · B = {label_b} ({len(b)}회)", "",
         "| 지표 | A 평균 | B 평균 | 변화 | 판정 |",
         "|---|---:|---:|---:|---|"]
    for lab, f, unit, lower_better in METRICS:
        va = np.array([f(r) for r in a], dtype=float); va = va[~np.isnan(va)]
        vb = np.array([f(r) for r in b], dtype=float); vb = vb[~np.isnan(vb)]
        if not len(va) or not len(vb):
            continue
        ma, mb = va.mean(), vb.mean()
        diff = mb - ma
        noise = va.max() - va.min()          # A 의 반복 편차
        if abs(diff) <= noise:
            verdict = "판단 보류 (편차 이내)"
        elif lower_better is None:
            verdict = "변화"
        else:
            better = (diff < 0) == lower_better
            verdict = "**개선**" if better else "**악화**"
        L.append(f"| {lab} [{unit}] | {ma:.2f} | {mb:.2f} | {diff:+.2f} | {verdict} |")
    L += ["", "> 판정 기준: A 의 반복 편차(최대-최소)보다 차이가 커야 변화로 본다.",
          "> 그보다 작으면 같은 조건에서도 그만큼 흔들리므로 노이즈와 구분되지 않는다.", ""]
    return L


def blame_section(runs, title):
    total = defaultdict(int)
    for r in runs:
        for k, v in r["blamed"].items():
            total[k] += v
    if not total:
        return []
    L = [f"### {title} — 주행 중 정지를 낸 모듈", "", "| 모듈 | 정지 횟수 |", "|---|---:|"]
    for k, v in sorted(total.items(), key=lambda kv: -kv[1]):
        L.append(f"| `{k}` | {v} |")
    return L + [""]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("a", help="배치 목록(.txt) 또는 CSV 1개")
    p.add_argument("b", nargs="?", help="A/B 비교 대상")
    p.add_argument("--out", default="/workspace/reports/compare.md")
    args = p.parse_args()

    runs_a = [summarize(f) for f in read_list(args.a)]
    if not runs_a:
        sys.exit(f"{args.a} 에 주행이 없다")

    L = ["# 주행 비교", ""]
    if args.b:
        runs_b = [summarize(f) for f in read_list(args.b)]
        la, lb = Path(args.a).stem, Path(args.b).stem
        L += ab_table(runs_a, runs_b, la, lb)
        L += spread_table(runs_a, f"A ({la}) 반복 편차")
        L += [""] + spread_table(runs_b, f"B ({lb}) 반복 편차")
        L += [""] + blame_section(runs_a, f"A ({la})") + blame_section(runs_b, f"B ({lb})")
    else:
        L += spread_table(runs_a, f"{Path(args.a).stem} 반복 편차")
        L += [""] + blame_section(runs_a, Path(args.a).stem)
        L += ["### 회차별", "", "| 주행 | 시간 [s] | 거리 [m] | 횡편차 p95 [cm] | 저크 p95 | 정지 |",
              "|---|---:|---:|---:|---:|---:|"]
        for r in runs_a:
            L.append(f"| {r['name']} | {r['duration']:.2f} | {r['distance']:.1f} | "
                     f"{r['lat_p95']:.1f} | {r['jerk_p95']:.2f} | {r['mid_stops']:.0f} |")

    text = "\n".join(L)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text)
    print(text)
    print(f"\n→ {out}")


if __name__ == "__main__":
    main()
