"""주행 1회(경로 설정 → 도착)를 하나의 CSV로 기록한다.

Autoware의 planning_evaluator / control_evaluator가 이미 지표를 실시간 발행하지만
아무도 그것을 주행 단위로 모아두지 않는다. 이 노드가 그 빈자리를 채운다.

출력은 long format (t, source, name, value) — MetricArray의 지표 이름이 매 메시지마다
달라질 수 있어 고정 컬럼 CSV는 깨지기 쉽다.
"""

import csv
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from tier4_metric_msgs.msg import MetricArray
from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState, VelocityFactorArray


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class MetricsCollector(Node):
    def __init__(self):
        super().__init__("metrics_collector")

        self.declare_parameter("output_dir", "/workspace/runs")
        self.output_dir = Path(self.get_parameter("output_dir").value)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.csv_file = None
        self.writer = None
        self.path = None
        self.t0 = None
        self.rows = 0

        self.create_subscription(
            MetricArray, "/planning/planning_evaluator/metrics",
            lambda m: self.on_metrics(m, "planning"), 10)
        self.create_subscription(
            MetricArray, "/control/control_evaluator/metrics",
            lambda m: self.on_metrics(m, "control"), 10)
        self.create_subscription(
            Odometry, "/localization/kinematic_state", self.on_odom, 10)

        # 정지 사유. 모듈별 /planning/planning_factors/* 가 41개 있지만,
        # ADAPI 가 그것들을 하나로 모아준다 — behavior 에 모듈 이름이 들어온다.
        self.create_subscription(
            VelocityFactorArray, "/api/planning/velocity_factors",
            self.on_velocity_factors, 10)

        # ADAPI 상태는 TRANSIENT_LOCAL 이라 구독자도 맞춰야 시작 시 현재 값을 받는다
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            RouteState, "/api/routing/state", self.on_route_state, latched)

        # 운행 모드. 기록 구간에는 engage 전 대기가 섞여 있는데, 그 구간의 자세 오차는
        # 스폰 위치 때문이지 추종 품질이 아니다. 실제로 반복 5회 모두 |횡편차| p95 가
        # 28.5~29.0 cm 로 같았고, 그 값은 전부 출발 직후 오프셋이었다.
        # 모드를 같이 남겨 분석에서 자율주행 구간만 잘라 쓸 수 있게 한다.
        self.create_subscription(
            OperationModeState, "/api/operation_mode/state",
            self.on_operation_mode, latched)

        self.get_logger().info(f"대기 중 — 경로가 설정되면 기록을 시작한다. 출력: {self.output_dir}")

    # ---- 주행 경계 ----

    def on_route_state(self, msg):
        if msg.state == RouteState.SET and self.writer is None:
            self.start_run()
        elif msg.state in (RouteState.ARRIVED, RouteState.UNSET) and self.writer is not None:
            self.stop_run("도착" if msg.state == RouteState.ARRIVED else "경로 해제")

    def start_run(self):
        name = time.strftime("run_%Y%m%d_%H%M%S.csv")
        self.path = self.output_dir / name
        self.csv_file = self.path.open("w", newline="")
        # csv 기본 dialect 는 줄바꿈이 \r\n 이다. csv.DictReader 는 알아서 처리하지만
        # awk·grep 같은 도구는 마지막 필드를 "2\r" 로 읽어 비교가 조용히 실패한다.
        self.writer = csv.writer(self.csv_file, lineterminator="\n")
        self.writer.writerow(["t", "source", "name", "value"])
        self.t0 = None
        self.rows = 0
        self.get_logger().info(f"기록 시작 → {self.path}")

    def stop_run(self, reason):
        self.csv_file.close()
        self.get_logger().info(f"기록 종료 ({reason}) — {self.rows}행 → {self.path}")
        self.csv_file = None
        self.writer = None
        self.path = None

    # ---- 기록 ----

    def write(self, stamp, source, name, value):
        if self.writer is None:
            return
        t = stamp.sec + stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        self.writer.writerow([f"{t - self.t0:.3f}", source, name, value])
        self.rows += 1

    def on_metrics(self, msg, source):
        for m in msg.metric_array:
            self.write(msg.stamp, source, m.name, m.value)

    def on_velocity_factors(self, msg):
        """어느 모듈이 무슨 이유로 세우려 하는지.

        long format 을 유지하려고 모듈 이름을 name 쪽에 넣는다:
            t, factor, <behavior>/status,   1=접근중 2=정지
            t, factor, <behavior>/distance, 정지점까지 남은 거리 [m]
        이렇게 두면 값이 계속 숫자라 기존 분석 코드가 그대로 돌아간다.
        (detail·sequence 는 문자열이라 지금은 버린다 — 필요해지면 그때 넣는다.)
        """
        for f in msg.factors:
            name = f.behavior or "unknown"
            self.write(msg.header.stamp, "factor", f"{name}/status", f.status)
            self.write(msg.header.stamp, "factor", f"{name}/distance", f"{f.distance:.3f}")

    def on_operation_mode(self, msg):
        # 1=STOP 2=AUTONOMOUS 3=LOCAL 4=REMOTE
        self.write(msg.stamp, "system", "operation_mode", msg.mode)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.write(msg.header.stamp, "ego", "x", f"{p.x:.4f}")
        self.write(msg.header.stamp, "ego", "y", f"{p.y:.4f}")
        self.write(msg.header.stamp, "ego", "yaw", f"{yaw_from_quaternion(msg.pose.pose.orientation):.6f}")
        self.write(msg.header.stamp, "ego", "vel", f"{msg.twist.twist.linear.x:.4f}")


def main():
    rclpy.init()
    node = MetricsCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.writer is not None:
            node.stop_run("중단")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
