#!/usr/bin/env python3
"""metrics_collector가 남긴 주행 1회 CSV를 그림과 요약으로 바꾼다.

사용:
    ./run_report.py                       # runs/ 안의 최신 CSV
    ./run_report.py runs/run_...csv       # 특정 주행
    ./run_report.py --out-dir reports     # 출력 위치 (기본 reports/)

출력: reports/<run 이름>/figures.png, summary.md
"""

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

# 라벨이 한글이라 CJK 폰트가 없으면 네모로 깨진다
_installed = {f.name for f in font_manager.fontManager.ttflist}
for _f in ("Noto Sans CJK KR", "Noto Sans CJK JP", "NanumGothic"):
    if _f in _installed:
        plt.rcParams["font.family"] = _f
        break
plt.rcParams["axes.unicode_minus"] = False

STOP_VEL = 0.1      # m/s 미만이면 정지로 본다
STOP_MIN_SEC = 0.5  # 이보다 짧은 정지는 노이즈로 무시


def load(path):
    """long format CSV → {(source, name): (t배열, value배열)}"""
    series = defaultdict(lambda: ([], []))
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            t, v = series[(row["source"], row["name"])]
            t.append(float(row["t"]))
            v.append(float(row["value"]))
    return {k: (np.array(t), np.array(v)) for k, (t, v) in series.items()}


def get(series, source, name):
    return series.get((source, name), (np.array([]), np.array([])))


def blamed_modules(series, t0, t1):
    """[t0, t1] 구간에서 STOPPED(2) 를 낸 모듈들을 (이름, 최소거리) 로 돌려준다.

    수집기가 `<모듈>/status` 와 `<모듈>/distance` 로 나눠 기록하므로 이름은 키에서 뽑는다.
    """
    # 1순위: 그 구간에 STOPPED(2) 를 낸 모듈.
    # 2순위: STOPPED 는 없지만 근거리(15 m 이내)에서 APPROACHING(1) 이던 모듈 — "(접근중)" 으로
    #        구분해 적는다. 실측에서 route-obstacle 이 차가 서 있는 동안에도 status 를
    #        1 로만 유지하는 경우가 있었다 (장애물 실험, 앞면 4.5 m 정지).
    stopped, approaching = [], []
    for (src, name), (t, v) in series.items():
        if src != "factor" or not name.endswith("/status"):
            continue
        mod = name[: -len("/status")]
        win = (t >= t0 - 0.5) & (t <= t1 + 0.5)
        td, d = series.get(("factor", f"{mod}/distance"), (np.array([]), np.array([])))
        dwin = (td >= t0 - 0.5) & (td <= t1 + 0.5) if len(d) else np.array([], bool)
        near = f" ({d[dwin].min():.1f} m)" if dwin.any() else ""
        if (win & (v == 2)).any():
            stopped.append(f"`{mod}`{near}")
        elif (win & (v == 1)).any() and dwin.any() and d[dwin].min() < 15.0:
            approaching.append(f"`{mod}` (접근중{near.strip() and near or ''})")
    return ", ".join(stopped + approaching)


def stop_events(t, vel):
    """정지 구간을 (시작, 끝) 목록으로. 어느 모듈이 세웠는지는 이 CSV에 없다."""
    stopped = vel < STOP_VEL
    events, start = [], None
    for i, s in enumerate(stopped):
        if s and start is None:
            start = t[i]
        elif not s and start is not None:
            if t[i] - start >= STOP_MIN_SEC:
                events.append((start, t[i]))
            start = None
    if start is not None and t[-1] - start >= STOP_MIN_SEC:
        events.append((start, t[-1]))
    return events


def summarize(series, path):
    t_v, vel = get(series, "ego", "vel")
    t_x, x = get(series, "ego", "x")
    _, y = get(series, "ego", "y")
    _, lat = get(series, "control", "lateral_deviation")
    _, jerk = get(series, "control", "jerk")
    t_o, obj = get(series, "control", "closest_object_distance")

    dist = float(np.sum(np.hypot(np.diff(x), np.diff(y)))) if len(x) > 1 else 0.0
    duration = float(t_v[-1]) if len(t_v) else 0.0
    moving = vel[vel >= STOP_VEL]

    L = [f"# {path.name}", ""]
    L += ["| 항목 | 값 |", "|---|---|"]
    L += [f"| 주행 시간 | {duration:.2f} s |",
          f"| 주행 거리 | {dist:.1f} m |",
          f"| 평균 속도 (정지 제외) | {moving.mean():.2f} m/s ({moving.mean()*3.6:.1f} km/h) |" if len(moving) else "| 평균 속도 | — |",
          f"| 최고 속도 | {vel.max():.2f} m/s ({vel.max()*3.6:.1f} km/h) |" if len(vel) else "| 최고 속도 | — |"]

    if len(lat):
        a = np.abs(lat)
        L += [f"| \\|횡편차\\| 최대 / p95 / RMS | {a.max()*100:.1f} / {np.percentile(a,95)*100:.1f} / {np.sqrt((lat**2).mean())*100:.1f} cm |"]
    if len(jerk):
        a = np.abs(jerk)
        L += [f"| \\|저크\\| 최대 / p95 | {a.max():.2f} / {np.percentile(a,95):.2f} m/s³ |"]
    if len(obj):
        L += [f"| 최근접 객체 거리 최소 | {obj.min():.2f} m |"]

    for key, label in [("goal_longitudinal_deviation_abs", "종방향"),
                       ("goal_lateral_deviation_abs", "횡방향"),
                       ("goal_yaw_deviation_abs", "방위각")]:
        _, g = get(series, "control", key)
        if len(g):
            unit = "rad" if "yaw" in key else "m"
            L += [f"| 목표 도착 오차 ({label}) | {g[-1]:.3f} {unit} |"]

    events = stop_events(t_v, vel) if len(t_v) else []
    mid = [e for e in events if e[0] > 0.01 and e[1] < duration - 0.01]
    L += ["", f"## 정지 이벤트 (총 {len(events)}회 · 주행 중 {len(mid)}회)", ""]
    if events:
        L += ["| # | 시작 | 종료 | 지속 | 성격 | 세운 모듈 | 그때 최근접 객체 |",
              "|---|---|---|---|---|---|---|"]
        for i, (s, e) in enumerate(events, 1):
            near = f"{obj[np.argmin(np.abs(t_o - s))]:.1f} m" if len(t_o) else "—"
            if s <= 0.01:
                kind = "출발 대기 (engage 전)"
            elif e >= duration - 0.01:
                kind = "도착 정지"
            else:
                kind = "**주행 중 정지**"
            why = blamed_modules(series, s, e) or "—"
            L += [f"| {i} | {s:.2f} s | {e:.2f} s | {e-s:.2f} s | {kind} | {why} | {near} |"]
    else:
        L += ["없음 (전 구간 주행)."]
    mods = sorted({n.split("/")[0] for (src, n) in series if src == "factor"})
    if mods:
        L += ["", f"이 주행에서 감속·정지를 요청한 모듈: {', '.join('`'+m+'`' for m in mods)}",
              "(`/api/planning/velocity_factors` 기준. 「세운 모듈」 열은 그 구간에 STOPPED 를 낸 것만 적는다.)", ""]
    else:
        L += ["", "> 정지 사유 데이터 없음 — 수집기가 `/api/planning/velocity_factors` 를", 
              "> 구독하기 전에 기록된 CSV다.", ""]
    return "\n".join(L), events


def figures(series, events, out_png):
    t_v, vel = get(series, "ego", "vel")
    t_x, x = get(series, "ego", "x")
    _, y = get(series, "ego", "y")
    t_p, vtgt = get(series, "planning", "velocity/mean")
    t_l, lat = get(series, "control", "lateral_deviation")
    # 시간축을 공유하지 않는다 — MetricArray 는 메시지마다 담기는 지표가 달라
    # 두 계열의 표본 수가 어긋날 수 있다 (실측: 1097 vs 1144).
    t_lc, latc = get(series, "control", "lateral_deviation_centerline")
    t_j, jerk = get(series, "control", "jerk")
    t_o, obj = get(series, "control", "closest_object_distance")

    fig, ax = plt.subplots(2, 3, figsize=(18, 9))
    fig.suptitle(out_png.parent.name, fontsize=13)

    a = ax[0][0]
    a.plot(t_v, vel, lw=1.5, label="ego")
    if len(t_p):
        a.plot(t_p, vtgt, lw=1.0, alpha=.7, label="planning 목표(mean)")
    for s, e in events:
        a.axvspan(s, e, color="tab:red", alpha=.15)
    a.set(title="속도 프로파일", xlabel="t [s]", ylabel="v [m/s]")
    a.legend(prop={"size": 8}); a.grid(alpha=.3)

    a = ax[0][1]
    if len(x):
        sc = a.scatter(x - x[0], y - y[0], c=np.interp(t_x, t_v, vel) if len(t_v) else None,
                       cmap="viridis", s=6)
        a.plot(0, 0, "go", ms=8, label="출발")
        a.plot(x[-1] - x[0], y[-1] - y[0], "r*", ms=13, label="도착")
        fig.colorbar(sc, ax=a, label="v [m/s]")
        a.legend(prop={"size": 8})
    a.set(title="경로 (출발점 기준 상대좌표)", xlabel="Δx [m]", ylabel="Δy [m]")
    a.axis("equal"); a.grid(alpha=.3)

    a = ax[0][2]
    a.plot(t_l, lat * 100, lw=1.2, label="lateral_deviation")
    if len(latc):
        a.plot(t_lc, latc * 100, lw=1.0, alpha=.7, label="centerline 기준")
    a.axhline(0, color="k", lw=.6)
    a.set(title="횡편차", xlabel="t [s]", ylabel="편차 [cm]")
    a.legend(prop={"size": 8}); a.grid(alpha=.3)

    a = ax[1][0]
    a.plot(t_j, jerk, lw=1.0)
    a.set(title="저크 시계열", xlabel="t [s]", ylabel="jerk [m/s³]")
    a.grid(alpha=.3)

    a = ax[1][1]
    if len(jerk):
        a.hist(jerk, bins=50, color="tab:purple", alpha=.8)
        a.axvline(np.percentile(np.abs(jerk), 95), color="tab:red", ls="--", lw=1,
                  label=f"|p95| = {np.percentile(np.abs(jerk),95):.2f}")
        a.legend(prop={"size": 8})
    a.set(title="저크 분포", xlabel="jerk [m/s³]", ylabel="샘플 수")
    a.grid(alpha=.3)

    a = ax[1][2]
    a.plot(t_o, obj, lw=1.2, color="tab:orange")
    for s, e in events:
        a.axvspan(s, e, color="tab:red", alpha=.15)
    a.set(title="최근접 객체 거리", xlabel="t [s]", ylabel="거리 [m]")
    a.grid(alpha=.3)

    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("csv", nargs="?", help="주행 CSV (생략하면 runs/ 최신)")
    p.add_argument("--runs-dir", default="/workspace/runs")
    p.add_argument("--out-dir", default="/workspace/reports")
    args = p.parse_args()

    if args.csv:
        path = Path(args.csv)
    else:
        found = sorted(Path(args.runs_dir).glob("run_*.csv"))
        if not found:
            sys.exit(f"{args.runs_dir} 에 run_*.csv 가 없다")
        path = found[-1]

    series = load(path)
    out = Path(args.out_dir) / path.stem
    out.mkdir(parents=True, exist_ok=True)

    text, events = summarize(series, path)
    (out / "summary.md").write_text(text)
    figures(series, events, out / "figures.png")

    print(text)
    print(f"→ {out}/figures.png")


if __name__ == "__main__":
    main()
