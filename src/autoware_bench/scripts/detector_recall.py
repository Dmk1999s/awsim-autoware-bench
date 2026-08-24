#!/usr/bin/env python3
"""ML 검출기(CenterPoint)의 재현율을 **군집을 기준선 삼아** 잰다.

문제는 정답이 없다는 것이다 — AWSIM 은 물체 정답 목록을 발행하지 않는다 (WORKLOG 34).
그래서 LiDAR 군집을 독립 관측자로 쓰되, 「군집이 잡은 것은 다 물체」라고 하면 벽·기둥까지
세게 된다. 여기서는 **움직인 적이 있는 군집만 실제 차량으로 인정한다** —
가만히 있는 구조물은 절대 움직이지 않으므로.

    군집 위치를 프레임 간 최근접으로 이어 궤적을 만든다
    → 궤적의 이동 거리가 MOVE_M 를 넘으면 「실제 차량」
    → 그 궤적의 각 프레임에서 ML 검출이 같은 자리에 있었는지 센다

더미는 위치를 알고 있으므로 따로 센다 (움직이지 않으니 위 규칙으로는 차량이 안 된다).

    detector_recall.py runs/probe_x.csv --dummy 81592.01,50020.21
"""

import argparse
import bisect
import csv
import math
import statistics as st

LINK_M = 3.0      # 프레임 간 같은 물체로 잇는 거리
MOVE_MS = 3.0     # 실제 차량 조건 ①: 프레임 간 속도의 중앙값이 이보다 빠를 것
MOVE_M = 12.0     # 실제 차량 조건 ②: 순이동 거리가 이보다 클 것.
                  # 둘 다 걸어야 한다 — 하나만으로는 못 가른다:
                  #   거리만 보면 → 벽·건물 군집 중심이 초당 0.3~0.9 m 로 흘러 20 초에 8~10 m
                  #   속도만 보면 → 구조물 군집이 프레임마다 튀어 속도가 10 m/s 로 나온다
                  # 실측(WORKLOG 40): 실제 차량은 속도 11~14 m/s 이면서 순이동 13~38 m 였다.
MATCH_M = 3.0     # ML 검출이 이 안에 있으면 「잡았다」
MIN_FRAMES = 10   # 너무 짧은 궤적은 버린다


def load(path):
    cl, cp, ego = {}, {}, {}
    for r in csv.DictReader(open(path)):
        t = round(float(r["t"]), 2)
        s, n, v = r["source"], r["name"], r["value"]
        if s == "o_cluster":
            x, y, la, sh = (float(z) for z in v.split())
            cl.setdefault(t, []).append((x, y, la, sh))
        elif s == "o_centerpoint":
            x, y, *_ = (float(z) for z in v.split())
            cp.setdefault(t, []).append((x, y))
        elif s == "ego" and n in ("x", "y"):
            ego.setdefault(t, {})[n] = float(v)
    ego = {t: (d["x"], d["y"]) for t, d in ego.items() if len(d) == 2}
    return cl, cp, ego


def link(cl):
    """프레임 간 최근접으로 군집을 이어 궤적을 만든다 (단순 최근접 연결)."""
    tracks = []          # [{"pts": [(t, x, y)], "last": (t, x, y)}]
    for t in sorted(cl):
        used = set()
        for (x, y, la, sh) in cl[t]:
            best, bd = None, LINK_M
            for i, tr in enumerate(tracks):
                if i in used or t - tr["last"][0] > 1.0:
                    continue
                d = math.hypot(x - tr["last"][1], y - tr["last"][2])
                if d < bd:
                    best, bd = i, d
            if best is None:
                tracks.append({"pts": [(t, x, y)], "last": (t, x, y), "size": [la]})
                used.add(len(tracks) - 1)
            else:
                tracks[best]["pts"].append((t, x, y))
                tracks[best]["last"] = (t, x, y)
                tracks[best]["size"].append(la)
                used.add(best)
    return tracks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--dummy", help="x,y — 더미 위치 (따로 센다)")
    a = ap.parse_args()
    cl, cp, ego = load(a.csv)
    dx, dy = ((float(v) for v in a.dummy.split(",")) if a.dummy else (None, None))

    cpt = sorted(cp)
    et = sorted(ego)

    def ml_at(t, x, y):
        i = bisect.bisect_left(cpt, t)
        for j in (i - 1, i, i + 1):
            if 0 <= j < len(cpt) and abs(cpt[j] - t) < 0.2:
                if any(math.hypot(px - x, py - y) < MATCH_M for px, py in cp[cpt[j]]):
                    return True
        return False

    def dist_ego(t, x, y):
        i = min(bisect.bisect_left(et, t), len(et) - 1)
        ex, ey = ego[et[i]]
        return math.hypot(x - ex, y - ey)

    tracks = link(cl)
    moving, dummy = [], []
    for tr in tracks:
        pts = tr["pts"]
        if len(pts) < MIN_FRAMES:
            continue
        vs = [math.hypot(b[1] - a[1], b[2] - a[2]) / max(b[0] - a[0], 1e-3)
              for a, b in zip(pts, pts[1:]) if b[0] - a[0] < 1.0]
        speed = st.median(vs) if vs else 0.0
        near_dummy = (dx is not None
                      and st.mean([math.hypot(p[1] - dx, p[2] - dy) for p in pts]) < 4.0)
        if near_dummy:
            dummy.append(pts)
        span = max(math.hypot(p[1] - pts[0][1], p[2] - pts[0][2]) for p in pts)
        if near_dummy:
            pass
        elif speed >= MOVE_MS and span >= MOVE_M:
            # 크기로는 거르지 않는다. 30 m 밖 차량의 군집은 LiDAR 가 본 한쪽 면뿐이라
            # 긴 변이 1.4 m 로 나오기도 한다 — 크기 조건을 걸면 실제 차량이 통째로 빠진다.
            moving.append(pts)

    def rate(group, label):
        if not group:
            print(f"{label:<26} 궤적 없음")
            return
        n = sum(len(p) for p in group)
        hit = sum(1 for p in group for (t, x, y) in p if ml_at(t, x, y))
        ds = [dist_ego(t, x, y) for p in group for (t, x, y) in p]
        print(f"{label:<26} 궤적 {len(group):3d}개 · 관측 {n:5d}프레임 · "
              f"**ML 재현율 {100*hit/n:3.0f}%** · 자차거리 중앙값 {st.median(ds):4.0f} m")
        # 거리 구간별
        for lo, hi in [(0, 20), (20, 40), (40, 60)]:
            sel = [(t, x, y) for p in group for (t, x, y) in p if lo <= dist_ego(t, x, y) < hi]
            if len(sel) < 20:
                continue
            h = sum(1 for (t, x, y) in sel if ml_at(t, x, y))
            print(f"      {lo:>2}~{hi:<2} m  {len(sel):5d}프레임  {100*h/len(sel):3.0f}%")

    print(f"군집 궤적 {len(tracks)}개 "
          f"(프레임 {MIN_FRAMES} 이상 · 중앙 속도 {MOVE_MS} m/s 이상 · 순이동 {MOVE_M} m 이상을 "
          f"차량으로 본다)\n")
    rate(moving, "실제 차량 (움직인 군집)")
    rate(dummy, "AWSIM 더미 (정지)")


if __name__ == "__main__":
    main()
