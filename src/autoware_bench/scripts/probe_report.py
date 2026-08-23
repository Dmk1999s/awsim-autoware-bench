#!/usr/bin/env python3
"""perception_probe 의 CSV 를 1초 단위 표로 읽는다.

각 단계에서 「목표 위치의 물체가 그 초에 한 번이라도 보였는가」를 본다.
점군 단계는 상자 안 점 개수(그 초의 최대), 물체 단계는 목표까지 최근접 거리.
"""

import argparse
import collections
import csv

STAGES = ["cluster", "centerpoint", "track", "predict"]


def load(path, radius):
    b = collections.defaultdict(dict)
    for r in csv.DictReader(open(path)):
        t = int(float(r["t"]))
        s, n, v = r["source"], r["name"], r["value"]
        if s in ("concat", "mapfilt") and n == "near":
            b[t][s] = max(b[t].get(s, 0), int(v))
        elif s in STAGES and n == "near":
            d = float(v) if v else 1e9
            b[t][s] = min(b[t].get(s, 1e9), d)
        elif s == "ego":
            if n == "dist":
                b[t]["d"] = float(v)
            elif n == "vel":
                b[t]["v"] = max(b[t].get("v", 0.0), abs(float(v)))
    return b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--radius", type=float, default=3.5)
    ap.add_argument("--from-dist", type=float, help="자차–목표 거리가 이 값 이하인 구간만")
    a = ap.parse_args()

    b = load(a.csv, a.radius)
    hdr = f"{'t':>4} {'거리':>6} {'속도':>5} {'concat':>7} {'mapfilt':>8}" + \
          "".join(f"{s:>12}" for s in STAGES)
    print(hdr)
    print("-" * len(hdr))
    seen = collections.Counter()
    total = 0
    for t in sorted(b):
        r = b[t]
        d = r.get("d")
        if d is None:
            continue
        if a.from_dist and d > a.from_dist:
            continue
        total += 1
        cells = ""
        for s in STAGES:
            v = r.get(s, 1e9)
            ok = v <= a.radius
            seen[s] += ok
            cells += f"{('O %.1f' % v) if ok else ('  · %.0f' % v if v < 1e8 else '   -'):>12}"
        print(f"{t:>4} {d:6.1f} {r.get('v', 0):5.1f} {r.get('concat', 0):7d} "
              f"{r.get('mapfilt', 0):8d}{cells}")
    print("-" * len(hdr))
    if total:
        print(f"관측 {total} 초 중 목표 물체가 보인 초: " +
              "  ".join(f"{s} {seen[s]}({100*seen[s]/total:.0f}%)" for s in STAGES))


if __name__ == "__main__":
    main()
