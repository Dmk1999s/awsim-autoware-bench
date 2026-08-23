"""시나리오 YAML 하나를 받아 주행 1회를 무인으로 완주시킨다.

metrics_collector 가 경로 SET→ARRIVED 를 보고 알아서 CSV 를 남기므로,
이 노드는 "같은 조건에서 다시 출발시키는 것"만 책임진다.

반복 실험이 성립하려면 매번 같은 자리에서 출발해야 한다. 그 절차가 자명하지 않아 적어둔다:

  1. AWSIM 의 ego 리셋       — ROS 로 노출돼 있지 않다. GUI 버튼을 xdotool 로 누른다.
  2. 위치추정 재초기화        — 1번만 하면 Autoware 는 텔레포트를 모른다.
                               상태는 계속 Initialized(3) 인데 실제로는 옛 위치를 붙들고
                               210 m 어긋난 채로 있다. 조용한 실패라 반드시 오차를 확인한다.
  3. clear_route             — 앞 주행이 ARRIVED 로 남아 있으면 새 경로를 거부한다
                               (success=False, message 없음).

사용:
    ros2 run autoware_bench scenario_runner --ros-args -p scenario:=/path/to/x.yaml
"""

import math
import subprocess
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState
from autoware_adapi_v1_msgs.srv import (
    ClearRoute, ChangeOperationMode, InitializeLocalization, SetRoutePoints,
)

# AWSIM 메뉴(☰)의 버튼 좌표. 창 크기가 고정이라 좌표도 고정이다.
# ROS 인터페이스가 없어 GUI 를 누르는 수밖에 없다.
AWSIM_EGO_RESET_XY = (566, 504)      # Ego Vehicle 리셋 — 스폰 좌표로 복귀
AWSIM_TRAFFIC_RESET_XY = (643, 611)  # Traffic 리셋 — 시드대로 NPC 재배치
AWSIM_DISPLAY = ":20"

LOCALIZATION_TOLERANCE_M = 0.5   # 정답 대비 이보다 어긋나면 출발시키지 않는다


class ScenarioRunner(Node):
    def __init__(self):
        super().__init__("scenario_runner")

        self.declare_parameter("scenario", "")
        path = self.get_parameter("scenario").value
        if not path:
            raise SystemExit("scenario 파라미터가 필요하다")
        self.spec = yaml.safe_load(open(path))
        self.get_logger().info(f"시나리오: {self.spec['name']}")

        self.route_state = None
        self.auto_available = False
        self.est = None    # NDT 추정 위치
        self.truth = None  # AWSIM 정답 위치

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(RouteState, "/api/routing/state",
                                 lambda m: setattr(self, "route_state", m.state), latched)
        self.create_subscription(
            OperationModeState, "/api/operation_mode/state",
            lambda m: setattr(self, "auto_available", m.is_autonomous_mode_available), latched)
        self.create_subscription(Odometry, "/localization/kinematic_state",
                                 lambda m: setattr(self, "est", m.pose.pose.position), 10)
        # AWSIM 의 정답 위치는 BEST_EFFORT 로 발행된다. 기본값(RELIABLE)으로 구독하면
        # QoS 불일치로 메시지가 한 개도 오지 않는다 — 경고만 뜨고 조용히 비어 있다.
        sensor = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, "/awsim/ground_truth/localization/kinematic_state",
                                 lambda m: setattr(self, "truth", m.pose.pose.position), sensor)

        self.cli = {
            "init": self.create_client(InitializeLocalization, "/api/localization/initialize"),
            "clear": self.create_client(ClearRoute, "/api/routing/clear_route"),
            "route": self.create_client(SetRoutePoints, "/api/routing/set_route_points"),
            "auto": self.create_client(ChangeOperationMode,
                                       "/api/operation_mode/change_to_autonomous"),
        }

    # ---- 도구 ----

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_until(self, predicate, timeout, label):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        self.get_logger().error(f"시간 초과: {label} ({timeout}s)")
        return False

    def call(self, key, request, timeout=30.0):
        client = self.cli[key]
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"서비스 없음: {key}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            raise RuntimeError(f"응답 없음: {key}")
        return future.result().status

    def localization_error(self):
        if self.est is None or self.truth is None:
            return None
        return math.hypot(self.est.x - self.truth.x, self.est.y - self.truth.y)

    # ---- 절차 ----

    def _click(self, xy):
        subprocess.run(["xdotool", "mousemove", str(xy[0]), str(xy[1]), "click", "1"],
                       env={"DISPLAY": AWSIM_DISPLAY, "PATH": "/usr/bin:/bin"}, check=True)

    def reset_ego(self):
        """차와 NPC 교통을 둘 다 출발 상태로 되돌린다.

        교통까지 리셋하는 이유: 시드를 고정해도 앞 주행이 흘려놓은 NPC 배치가 남아 있으면
        회차마다 조건이 다르다. 실제로 교통을 두고 5회 돌렸을 때 주행 시간이
        40.3~65.1 s (폭 24.8 s) 로 흔들렸고, 그 폭이 파라미터 효과를 덮을 만큼 컸다.
        """
        self._click(AWSIM_TRAFFIC_RESET_XY)
        self.spin(1.0)
        self._click(AWSIM_EGO_RESET_XY)
        self.get_logger().info("AWSIM 리셋 (교통 + ego)")
        self.spin(5.0)

    def reinit_localization(self):
        """빈 pose = GNSS 자동 초기화. 그다음 정답과 대조해 실제로 수렴했는지 본다."""
        status = self.call("init", InitializeLocalization.Request(pose=[]))
        if not status.success:
            raise RuntimeError(f"위치추정 초기화 실패: {status.message}")
        ok = self.wait_until(
            lambda: (e := self.localization_error()) is not None and e < LOCALIZATION_TOLERANCE_M,
            timeout=60.0, label="위치추정 수렴")
        if not ok:
            raise RuntimeError(f"위치추정이 수렴하지 않음 (오차 {self.localization_error()})")
        self.get_logger().info(f"위치추정 수렴 — 오차 {self.localization_error():.3f} m")

    def set_route(self):
        self.call("clear", ClearRoute.Request())   # ARRIVED 가 남아 있으면 새 경로가 거부된다
        self.spin(2.0)

        g = self.spec["goal"]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = g["x"], g["y"], g.get("z", 41.5)
        pose.orientation.z = math.sin(g["yaw"] / 2.0)
        pose.orientation.w = math.cos(g["yaw"] / 2.0)

        req = SetRoutePoints.Request(goal=pose)
        req.header.frame_id = "map"
        req.option.allow_goal_modification = self.spec.get("allow_goal_modification", True)
        status = self.call("route", req)
        if not status.success:
            raise RuntimeError(f"경로 설정 거부: code={status.code} {status.message}")
        self.get_logger().info(f"경로 설정 → ({g['x']}, {g['y']})")

    def engage(self):
        # 경로를 넣은 직후에는 자율주행이 아직 준비되지 않는다. 계획이 trajectory 를
        # 내놓아야 is_autonomous_mode_available 이 켜지고, 그 전에 부르면
        # "The target mode is not available" 로 거부된다. 고정 대기 대신 준비를 기다린다.
        if not self.wait_until(lambda: self.auto_available, 60.0, "자율주행 준비"):
            raise RuntimeError("자율주행 모드가 준비되지 않음 — 계획이 경로를 못 풀었을 수 있다")
        status = self.call("auto", ChangeOperationMode.Request())
        if not status.success:
            raise RuntimeError(f"자율주행 전환 실패: {status.message}")
        self.get_logger().info("자율주행 전환")

    def drive(self):
        timeout = self.spec.get("timeout_s", 300)
        t0 = time.time()
        ok = self.wait_until(lambda: self.route_state == RouteState.ARRIVED,
                             timeout=timeout, label="목적지 도착")
        return ok, time.time() - t0

    def run(self):
        self.wait_until(lambda: self.truth is not None, 30.0, "AWSIM 연결")
        self.reset_ego()
        self.reinit_localization()
        self.set_route()
        self.engage()
        arrived, elapsed = self.drive()
        if arrived:
            self.get_logger().info(f"완주 — {elapsed:.1f} s")
        return arrived


def main():
    rclpy.init()
    node = ScenarioRunner()
    code = 0
    try:
        code = 0 if node.run() else 1
    except (RuntimeError, SystemExit) as e:
        node.get_logger().error(str(e))
        code = 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
