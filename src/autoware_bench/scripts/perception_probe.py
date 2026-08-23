#!/usr/bin/env python3
"""인지 파이프라인 단계별로 「특정 위치의 물체가 지금 보이는가」를 기록한다.

WORKLOG 29 에서 장애물이 접근 구간의 30% 만 보인다는 것까지는 알았지만,
그것이 ① 시뮬레이터가 물체를 안 그리는 것인지 ② LiDAR 가 못 맞히는 것인지
③ 인지가 떨어뜨리는 것인지 구분하지 못했다. 단계마다 같은 질문을 던져 구분한다.

    concat      /sensing/lidar/concatenated/pointcloud          점군에 점이 있는가
    mapfilt     .../detection/pointcloud_map_filtered/pointcloud  지도 대조 뒤에도 남는가
    cluster     .../detection/clustering/objects                군집화가 물체로 묶는가
    centerpoint .../detection/centerpoint/objects               ML 검출이 잡는가
    track       .../tracking/objects                            추적이 유지하는가
    predict     /perception/object_recognition/objects          계획이 보는 최종 결과

출력은 collector 와 같은 long format (t, source, name, value).
    <stage>/near   목표 위치까지 최근접 물체 거리 [m] (점군 단계는 상자 안 점 개수)
    <stage>/n      그 메시지의 물체 총수
    ego/dist       자차–목표 거리
"""

import argparse
import csv
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from autoware_perception_msgs.msg import DetectedObjects, TrackedObjects, PredictedObjects


def xyz_of(msg):
    """PointCloud2 에서 x,y,z 만 뽑는다.

    sensor_msgs_py.read_points_numpy 는 필드 자료형이 섞여 있으면 못 읽는다
    (AWSIM 점군은 intensity 가 uint8 이다). offset 으로 직접 잘라 쓴다.
    """
    off = {f.name: f.offset for f in msg.fields if f.name in ("x", "y", "z")}
    if len(off) != 3:
        return None
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    raw = raw.reshape(-1, msg.point_step)
    cols = [raw[:, off[k]:off[k] + 4].copy().view(np.float32).ravel() for k in ("x", "y", "z")]
    pts = np.stack(cols, axis=1)
    return pts[np.isfinite(pts).all(axis=1)]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self, tx, ty, out, radius, clouds):
        super().__init__("perception_probe")
        self.tx, self.ty, self.radius = tx, ty, radius
        self.ego = None          # (x, y, yaw)
        self.t0 = None
        self.f = open(out, "w", newline="")
        self.w = csv.writer(self.f, lineterminator="\n")
        self.w.writerow(["t", "source", "name", "value"])

        self.create_subscription(Odometry, "/localization/kinematic_state", self.on_odom, 10)
        rel = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(
            DetectedObjects, "/perception/object_recognition/detection/clustering/objects",
            lambda m: self.on_objects(m, "cluster"), rel)
        self.create_subscription(
            DetectedObjects, "/perception/object_recognition/detection/centerpoint/objects",
            lambda m: self.on_objects(m, "centerpoint"), rel)
        self.create_subscription(
            TrackedObjects, "/perception/object_recognition/tracking/objects",
            lambda m: self.on_objects(m, "track"), rel)
        self.create_subscription(
            PredictedObjects, "/perception/object_recognition/objects",
            lambda m: self.on_objects(m, "predict"), rel)

        if clouds:
            # 점군은 무겁다. 부하 자체가 이 실험의 변수라 기본으로는 끈다.
            be = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(
                PointCloud2, "/sensing/lidar/concatenated/pointcloud",
                lambda m: self.on_cloud(m, "concat"), be)
            self.create_subscription(
                PointCloud2,
                "/perception/object_recognition/detection/pointcloud_map_filtered/pointcloud",
                lambda m: self.on_cloud(m, "mapfilt"), be)

        self.get_logger().info(f"목표 ({tx:.1f}, {ty:.1f}) · 반경 {radius} m → {out}")

    # ---- 기록 ----

    def write(self, stamp, source, name, value):
        t = stamp.sec + stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        self.w.writerow([f"{t - self.t0:.3f}", source, name, value])

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.ego = (p.x, p.y, yaw_of(msg.pose.pose.orientation))
        self.write(msg.header.stamp, "ego", "x", f"{p.x:.3f}")
        self.write(msg.header.stamp, "ego", "y", f"{p.y:.3f}")
        self.write(msg.header.stamp, "ego", "vel", f"{msg.twist.twist.linear.x:.3f}")
        self.write(msg.header.stamp, "ego", "dist",
                   f"{math.hypot(p.x - self.tx, p.y - self.ty):.2f}")

    def to_map(self, frame, x, y):
        """물체 좌표를 map 기준으로 바꾼다. 단계마다 frame 이 다르다."""
        if frame == "map":
            return x, y
        if self.ego is None:
            return None
        ex, ey, eyaw = self.ego
        c, s = math.cos(eyaw), math.sin(eyaw)
        return ex + c * x - s * y, ey + s * x + c * y

    def on_objects(self, msg, stage):
        best = None
        for o in msg.objects:
            k = o.kinematics
            pose = getattr(k, "pose_with_covariance", None) or k.initial_pose_with_covariance
            p = pose.pose.position
            xy = self.to_map(msg.header.frame_id, p.x, p.y)
            if xy is None:
                continue
            d = math.hypot(xy[0] - self.tx, xy[1] - self.ty)
            if best is None or d < best:
                best = d
            # 추적 단계만 전체 물체 위치를 남긴다 — 실제 NPC 의 연속성 비교에 쓴다
            if stage == "track":
                self.write(msg.header.stamp, "obj",
                           f"{bytes(o.object_id.uuid[:4]).hex()}/xy",
                           f"{xy[0]:.2f} {xy[1]:.2f}")
        self.write(msg.header.stamp, stage, "n", len(msg.objects))
        self.write(msg.header.stamp, stage, "near", f"{best:.2f}" if best is not None else "")

    def on_cloud(self, msg, stage):
        if self.ego is None:
            return
        pts = xyz_of(msg)
        if pts is None or pts.size == 0:
            self.write(msg.header.stamp, stage, "near", 0)
            return
        if msg.header.frame_id == "map":
            mx, my = pts[:, 0], pts[:, 1]
        else:
            ex, ey, eyaw = self.ego
            c, s = math.cos(eyaw), math.sin(eyaw)
            mx = ex + c * pts[:, 0] - s * pts[:, 1]
            my = ey + s * pts[:, 0] + c * pts[:, 1]
        inbox = (np.abs(mx - self.tx) < self.radius) & (np.abs(my - self.ty) < self.radius)
        self.write(msg.header.stamp, stage, "near", int(inbox.sum()))
        self.write(msg.header.stamp, stage, "n", len(pts))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", required=True, help="x,y (map)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--radius", type=float, default=3.5)
    ap.add_argument("--clouds", action="store_true")
    a = ap.parse_args()
    tx, ty = (float(v) for v in a.target.split(","))

    rclpy.init()
    node = Probe(tx, ty, a.out, a.radius, a.clouds)
    end = time.time() + a.seconds
    try:
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()
        node.get_logger().info(f"기록 종료 → {a.out}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
