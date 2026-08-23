#!/usr/bin/env python3
"""배치 묶음들을 그림 한 장으로 비교한다. 표(compare_runs.py)로는 안 보이는
"언제 어디서 달라졌는가"를 보려는 것이다. 두 개든 네 개든 받는다.

사용:
    ./plot_ab.py runs/batch_a.txt runs/batch_b.txt --labels "모듈 끔" "모듈 켬" \
                 --out docs/img/ab.png
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

sys.path.insert(0, str(Path(__file__).parent))
from compare_runs import load, get, drive_window, lateral_acceleration, read_list  # noqa: E402

_installed = {f.name for f in font_manager.fontManager.ttflist}
for _f in ("Noto Sans CJK KR", "Noto Sans CJK JP", "NanumGothic"):
    if _f in _installed:
        plt.rcParams["font.family"] = _f
        break
plt.rcParams["axes.unicode_minus"] = False

COLORS = ("#888888", "#d1495b", "#2a9d8f", "#3f6fb5", "#c77dff")


def arm(paths):
    """배치 한 묶음에서 그릴 것만 뽑는다. 시간축은 자율주행 시작을 0 으로 맞춘다."""
    out = []
    for p in paths:
        s = load(p)
        t0 = drive_window(s)
        tv, vel = get(s, "ego", "vel")
        tl, lat = get(s, "control", "lateral_deviation")
        ty, yaw = get(s, "ego", "yaw")
        alat = lateral_acceleration(s, t0)
        ta = ty[1:][(np.diff(ty) > 1e-3) & (np.diff(ty) < 0.5) & (ty[1:] >= t0)
                    & (np.abs(np.interp(ty, tv, vel)[1:]) > 0.5)] if len(ty) > 2 else np.array([])
        out.append({
            "t_v": tv - t0, "vel": vel,
            "t_l": tl - t0, "lat": np.abs(lat) * 100,
            "t_a": ta - t0, "alat": alat,
        })
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("batches", nargs="+")
    ap.add_argument("--labels", nargs="+")
    ap.add_argument("--title", default="")
    ap.add_argument("--out", default="reports/ab.png")
    args = ap.parse_args()

    labels = args.labels or [Path(b).stem for b in args.batches]
    if len(labels) != len(args.batches):
        raise SystemExit("--labels 개수가 배치 개수와 다르다")
    arms = [arm(read_list(b)) for b in args.batches]

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))
    for ai, (runs, label, color) in enumerate(zip(arms, labels, COLORS)):
        for i, r in enumerate(runs):
            kw = dict(color=color, lw=1.0, alpha=0.75, label=label if i == 0 else None)
            axes[0].plot(r["t_v"], r["vel"], **kw)
            if len(r["t_a"]) == len(r["alat"]) and len(r["alat"]):
                axes[1].plot(r["t_a"], r["alat"], **kw)
        # 회차별 최댓값을 점으로. 주행의 대부분은 직선이라 분포 중앙값은 0 에 붙는다 —
        # 이 모듈이 겨냥하는 것은 회전 구간의 봉우리다.
        peaks = [r["alat"].max() for r in runs if len(r["alat"])]
        jitter = np.linspace(-0.12, 0.12, len(peaks))
        axes[2].scatter(ai + jitter, peaks, color=color, s=45, zorder=3, alpha=0.85)
        axes[2].hlines(np.mean(peaks), ai - 0.28, ai + 0.28, color=color, lw=2.5)
        axes[2].annotate(f"평균 {np.mean(peaks):.2f}", (ai + 0.3, np.mean(peaks)),
                         fontsize=9, va="center")

    axes[0].set(xlabel="자율주행 시작 후 [s]", ylabel="속도 [m/s]", title="속도 프로파일")
    axes[0].legend(fontsize=9)
    axes[0].grid(alpha=0.3)
    axes[1].set(xlabel="자율주행 시작 후 [s]", ylabel="|횡가속도| [m/s²]",
                title="횡가속도 (자차 궤적에서 계산)")
    axes[1].grid(alpha=0.3)
    axes[2].set(xticks=range(len(labels)), xticklabels=labels, ylabel="|횡가속도| [m/s²]",
                title="회차별 최댓값", xlim=(-0.6, len(labels) - 0.3 + 0.4), ylim=(0, None))
    axes[2].tick_params(axis="x", labelsize=9)
    axes[2].grid(alpha=0.3, axis="y")

    if args.title:
        fig.suptitle(args.title, fontsize=12)
        fig.tight_layout(rect=(0, 0, 1, 0.94))
    else:
        fig.tight_layout()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f"→ {out}")


if __name__ == "__main__":
    main()
