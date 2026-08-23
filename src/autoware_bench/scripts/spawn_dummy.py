#!/usr/bin/env python3
"""더미 객체 하나를 씬에 심고 유지한다 (러너 없이 인지만 볼 때 쓴다).

AWSIM 더미는 약 30 초면 스스로 사라지므로(WORKLOG 26) 기본으로 25 초마다 다시 심는다.
러너와 같은 방식이다 — 먼저 새로 심고, 그다음 옛것을 지운다.
"""

import argparse
import math
import time
import uuid as uuid_lib

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tier4_simulation_msgs.msg import DummyObject


class Spawner(Node):
    def __init__(self, a):
        super().__init__("spawn_dummy")
        self.a = a
        self.ego = None
        self.live = []
        self.pub = self.create_publisher(
            DummyObject, "/simulation/dummy_perception_publisher/object_info", 10)
        self.create_subscription(Odometry, "/localization/kinematic_state", self.on_odom, 10)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.ego = (p.x, p.y, yaw)

    def add(self, x, y, yaw):
        msg = DummyObject()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        uid = [int(b) for b in uuid_lib.uuid4().bytes]
        msg.id.uuid = uid
        msg.action = DummyObject.ADD
        p = msg.initial_state.pose_covariance.pose
        p.position.x, p.position.y, p.position.z = x, y, self.a.z
        p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        msg.classification.label = self.a.label
        msg.classification.probability = 1.0
        msg.shape.type = 0
        msg.shape.dimensions.x, msg.shape.dimensions.y, msg.shape.dimensions.z = 4.5, 1.8, 1.6
        msg.initial_state.twist_covariance.twist.linear.x = self.a.velocity
        msg.max_velocity = self.a.velocity
        self.pub.publish(msg)
        self.live.append(uid)
        self.get_logger().info(f"ADD ({x:.2f}, {y:.2f}) z={self.a.z} yaw={yaw:.3f}")

    def delete(self, uid):
        msg = DummyObject()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.id.uuid = uid
        msg.action = DummyObject.DELETE
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--at", help="x,y,yaw (map). 없으면 --ahead 로 자차 앞에 놓는다")
    ap.add_argument("--ahead", type=float, help="자차 진행 방향으로 이 거리 앞")
    ap.add_argument("--lateral", type=float, default=0.0, help="자차 기준 좌(+)/우(-) 오프셋 [m]")
    ap.add_argument("--z", type=float, default=41.3)
    ap.add_argument("--label", type=int, default=1)
    ap.add_argument("--velocity", type=float, default=0.0)
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--refresh", type=float, default=25.0, help="0 이면 다시 심지 않는다")
    ap.add_argument("--keep", action="store_true", help="끝날 때 지우지 않는다 (수명으로 사라지게 둔다)")
    a = ap.parse_args()

    rclpy.init()
    node = Spawner(a)
    if a.at:
        x, y, yaw = (float(v) for v in a.at.split(","))
    else:
        end = time.time() + 10
        while node.ego is None and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.ego is None:
            raise SystemExit("자차 위치를 못 받았다")
        ex, ey, eyaw = node.ego
        x = ex + a.ahead * math.cos(eyaw) - a.lateral * math.sin(eyaw)
        y = ey + a.ahead * math.sin(eyaw) + a.lateral * math.cos(eyaw)
        yaw = eyaw
        node.get_logger().info(f"자차 ({ex:.2f}, {ey:.2f}) yaw={eyaw:.3f}")

    node.add(x, y, yaw)
    end = time.time() + a.seconds
    last = time.time()
    try:
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if a.refresh and time.time() - last >= a.refresh:
                old = list(node.live)
                node.add(x, y, yaw)
                time.sleep(0.5)
                for uid in old:
                    node.delete(uid)
                node.live = [u for u in node.live if u not in old]
                last = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        if not a.keep:
            for uid in node.live:
                node.delete(uid)
            node.get_logger().info("삭제 완료")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
