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
import uuid as uuid_lib

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from tier4_simulation_msgs.msg import DummyObject
from autoware_perception_msgs.msg import (
    TrafficLightGroupArray, TrafficLightGroup, TrafficLightElement,
)
from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState
from autoware_adapi_v1_msgs.srv import (
    ClearRoute, ChangeOperationMode, InitializeLocalization, SetRoutePoints,
)

# AWSIM 메뉴(☰)의 버튼 좌표. 창 크기가 고정이라 좌표도 고정이다.
# ROS 인터페이스가 없어 GUI 를 누르는 수밖에 없다.
AWSIM_EGO_RESET_XY = (566, 504)      # Ego Vehicle 리셋 — 스폰 좌표로 복귀
AWSIM_TRAFFIC_RESET_XY = (643, 611)  # Traffic 리셋 — 시드대로 NPC 재배치
AWSIM_DISPLAY = ":20"

LOCALIZATION_TOLERANCE_M = 0.5
ENGAGE_ATTEMPTS = 5              # 전환이 간헐적으로 거부된다 — 아래 주석 참고   # 정답 대비 이보다 어긋나면 출발시키지 않는다


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
        self.est_vel = 0.0
        self.auto_seq = 0   # 새 메시지인지 가리기 위한 카운터
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
            OperationModeState, "/api/operation_mode/state", self.on_operation_mode, latched)
        self.create_subscription(Odometry, "/localization/kinematic_state",
                                 self.on_kinematic, 10)
        # AWSIM 의 정답 위치는 BEST_EFFORT 로 발행된다. 기본값(RELIABLE)으로 구독하면
        # QoS 불일치로 메시지가 한 개도 오지 않는다 — 경고만 뜨고 조용히 비어 있다.
        sensor = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, "/awsim/ground_truth/localization/kinematic_state",
                                 lambda m: setattr(self, "truth", m.pose.pose.position), sensor)

        # 더미 객체. 이 토픽의 구독자는 dummy_perception_publisher 가 아니라 AWSIM 자신이다
        # (GID 로 확인). 즉 여기로 ADD 를 보내면 AWSIM 씬에 실제 NPC 차량이 스폰되고,
        # LiDAR → 인지 → 계획 전체 파이프라인이 그것을 감지한다 — 인지를 우회하지 않는다.
        self.obj_pub = self.create_publisher(
            DummyObject, "/simulation/dummy_perception_publisher/object_info", 10)

        # 외부 신호 오버라이드. traffic_light_arbiter 가 카메라 인지와 병합한다.
        # 시뮬레이터의 실제 신호와 무관하게 특정 신호를 강제해, 신호 정지 실험을
        # "도착 시점의 신호 운"에서 떼어내 재현 가능하게 만든다.
        self.tl_pub = self.create_publisher(
            TrafficLightGroupArray,
            "/perception/traffic_light_recognition/external/traffic_signals", 10)

        self.cli = {
            "init": self.create_client(InitializeLocalization, "/api/localization/initialize"),
            "clear": self.create_client(ClearRoute, "/api/routing/clear_route"),
            "route": self.create_client(SetRoutePoints, "/api/routing/set_route_points"),
            "auto": self.create_client(ChangeOperationMode,
                                       "/api/operation_mode/change_to_autonomous"),
        }

    def on_kinematic(self, msg):
        self.est = msg.pose.pose.position
        self.est_vel = msg.twist.twist.linear.x

    def on_operation_mode(self, msg):
        self.auto_available = msg.is_autonomous_mode_available
        self.auto_seq += 1

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
        # routing/state 는 변화할 때만 발행된다. 구독 시점에 받은 지난 주행의 ARRIVED 가
        # 남아 있으면 drive() 가 즉시 "도착"으로 판정한다. 서비스가 성공했으므로 SET 으로 덮는다.
        self.route_state = RouteState.SET
        self.get_logger().info(f"경로 설정 → ({g['x']}, {g['y']})")

    def engage(self):
        # 경로를 넣은 직후에는 자율주행이 아직 준비되지 않는다. 계획이 trajectory 를
        # 내놓아야 is_autonomous_mode_available 이 켜지고, 그 전에 부르면
        # "The target mode is not available" 로 거부된다.
        #
        # 그런데 이 토픽은 TRANSIENT_LOCAL 이라 **구독하는 순간 지난 주행의 마지막 값**이
        # 먼저 들어온다. 러너는 회차마다 새 프로세스로 뜨므로, 그 묵은 true 를 보고
        # 대기를 통과해버린 뒤 전환이 거부됐다 (w20 배치 5회 중 2회가 이렇게 실패).
        # 그래서 "지금 시점 이후에 새로 온 메시지"만 인정한다.
        # 게다가 이 플래그가 true 여도 서비스가 거부하는 경우가 있다 — 플래그와 실제
        # 상태머신 사이에 잠깐 틈이 있다. 관측상 mpc_weight_lat_error 를 20 으로 올렸을 때만
        # 나타났고(1.0/5.0 배치에서는 0회), 제어 검증이 걸리는 것으로 보인다.
        # 몇 번 만에 붙었는지를 남겨 가중치별로 비교할 수 있게 한다.
        for attempt in range(1, ENGAGE_ATTEMPTS + 1):
            seq0 = self.auto_seq
            if not self.wait_until(lambda: self.auto_seq > seq0 + 1 and self.auto_available,
                                   60.0, "자율주행 준비"):
                raise RuntimeError("자율주행 모드가 준비되지 않음 — 계획이 경로를 못 풀었을 수 있다")
            status = self.call("auto", ChangeOperationMode.Request())
            if status.success:
                self.get_logger().info(f"자율주행 전환 (시도 {attempt}회)")
                return
            self.get_logger().warn(f"전환 거부 ({attempt}/{ENGAGE_ATTEMPTS}): {status.message}")
            self.spin(3.0)
        raise RuntimeError(f"자율주행 전환 실패 — {ENGAGE_ATTEMPTS}회 시도")

    def clear_objects(self):
        """앞 주행이 남긴 더미 객체 제거. 이걸 빼먹으면 회차 간 조건이 달라진다."""
        msg = DummyObject()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.action = DummyObject.DELETEALL if hasattr(DummyObject, "DELETEALL") else 3
        self.obj_pub.publish(msg)
        self.spin(1.0)

    def spawn_objects(self, objs=None):
        """시나리오의 objects: 목록(또는 지정 목록)을 씬에 스폰한다."""
        if objs is None:
            objs = self.spec.get("objects", [])
        for o in objs:
            msg = DummyObject()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.id.uuid = list(uuid_lib.uuid4().bytes)
            msg.action = DummyObject.ADD
            p = msg.initial_state.pose_covariance.pose
            p.position.x, p.position.y = float(o["x"]), float(o["y"])
            p.position.z = float(o.get("z", 41.5))
            yaw = float(o.get("yaw", 0.0))
            p.orientation.z = math.sin(yaw / 2.0)
            p.orientation.w = math.cos(yaw / 2.0)
            msg.classification.label = int(o.get("label", 1))   # 1 = CAR
            msg.classification.probability = 1.0
            msg.shape.type = 0                                   # BOUNDING_BOX
            d = o.get("dimensions", {})
            msg.shape.dimensions.x = float(d.get("x", 4.5))
            msg.shape.dimensions.y = float(d.get("y", 1.8))
            msg.shape.dimensions.z = float(d.get("z", 1.6))
            msg.initial_state.twist_covariance.twist.linear.x = float(o.get("velocity", 0.0))
            msg.max_velocity = float(o.get("velocity", 0.0))
            msg.min_velocity = 0.0
            self.obj_pub.publish(msg)
            self.get_logger().info(
                f"객체 스폰: label={msg.classification.label} ({o['x']:.1f}, {o['y']:.1f}) "
                f"v={o.get('velocity', 0.0)}")
        if objs:
            self.spin(2.0)

    def publish_signal(self, group_ids, color):
        msg = TrafficLightGroupArray()
        msg.stamp = self.get_clock().now().to_msg()
        for gid in group_ids:
            g = TrafficLightGroup()
            g.traffic_light_group_id = int(gid)
            e = TrafficLightElement()
            e.color = color
            e.shape = TrafficLightElement.CIRCLE
            e.status = TrafficLightElement.SOLID_ON
            e.confidence = 1.0
            g.elements.append(e)
            msg.traffic_light_groups.append(g)
        self.tl_pub.publish(msg)

    def drive(self):
        timeout = self.spec.get("timeout_s", 300)
        t0 = time.time()

        tl = self.spec.get("traffic_override")

        # 상시 강제 (hold_stop_s 없음): 지정 색을 도착까지 계속 발행.
        # 앞차 추종 실험에서 쓴다 — 더미 앞차는 신호를 무시하므로, 자차만 빨간불에
        # 걸리면 추종이 아니라 추격이 된다 (실측: 15초 정차 동안 앞차가 22 m 도망).
        if tl and "hold_stop_s" not in tl:
            color = getattr(TrafficLightElement, tl.get("color", "GREEN"))
            self.get_logger().info(f"신호 상시 강제: 그룹 {tl['group_ids']} {tl.get('color','GREEN')}")
            last_pub = 0.0
            while time.time() - t0 < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                if time.time() - last_pub > 0.4:
                    self.publish_signal(tl["group_ids"], color)
                    last_pub = time.time()
                if self.route_state == RouteState.ARRIVED:
                    return True, time.time() - t0
            self.get_logger().error(f"시간 초과 ({timeout}s)")
            return False, time.time() - t0

        # 신호 강제 실험: 빨강을 계속 발행 → 정지 확인 → 초록으로 전환 → 통과 관찰.
        if tl:
            gids = tl["group_ids"]
            hold = tl.get("hold_stop_s", 8)
            stop_started = None
            last_pub = 0.0
            phase = "red"
            self.get_logger().info(f"신호 강제: 그룹 {gids} 빨강")
            while time.time() - t0 < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                now = time.time()
                if now - last_pub > 0.4:   # arbiter 는 갱신이 끊긴 외부 신호를 무시한다
                    self.publish_signal(
                        gids,
                        TrafficLightElement.RED if phase == "red" else TrafficLightElement.GREEN)
                    last_pub = now
                if phase == "red" and now - t0 > 5.0:
                    moving = abs(self.est_vel) > 0.1
                    if moving:
                        stop_started = None
                    elif stop_started is None:
                        stop_started = now
                    elif now - stop_started >= hold:
                        self.get_logger().info(f"빨간불 정지 {hold}s 확인 — 초록으로 전환")
                        phase = "green"
                if self.route_state == RouteState.ARRIVED:
                    if phase == "red":
                        self.get_logger().error("빨간불인데 도착 — 신호를 무시하고 통과했다")
                        return False, time.time() - t0
                    return True, time.time() - t0
            self.get_logger().error(f"시간 초과 ({timeout}s, phase={phase})")
            return False, time.time() - t0

        # 장애물 시나리오: 차선을 막은 객체 앞에서는 영원히 도착하지 못한다.
        # "hold_stop_s 초 연속 정지"를 확인하면 장애물을 치우고 재출발까지 본다 —
        # 정지(계획이 세우는가)와 재출발(치우면 다시 가는가)을 한 주행에서 모두 검증한다.
        hold = self.spec.get("hold_stop_s")
        if hold and self.spec.get("objects"):
            self.spin(3.0)   # 출발 직후의 정지(스폰 대기 잔여)를 정지로 오인하지 않도록
            stop_started = None
            while time.time() - t0 < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                moving = abs(self.est_vel) > 0.1
                if moving:
                    stop_started = None
                elif stop_started is None:
                    stop_started = time.time()
                elif time.time() - stop_started >= hold:
                    self.get_logger().info(f"장애물 앞 정지 {hold}s 확인 — 객체 제거, 재출발 관찰")
                    self.clear_objects()
                    break
            else:
                self.get_logger().error(f"정지가 확인되지 않음 ({timeout}s)")
                return False, time.time() - t0

        ok = self.wait_until(lambda: self.route_state == RouteState.ARRIVED,
                             timeout=timeout - (time.time() - t0), label="목적지 도착")
        return ok, time.time() - t0

    def run(self):
        self.wait_until(lambda: self.truth is not None, 30.0, "AWSIM 연결")
        self.reset_ego()
        self.clear_objects()
        self.reinit_localization()
        self.set_route()
        # 정지 객체는 engage 전에 — 출발 시점부터 인지가 보고 있어야 한다.
        # 움직이는 객체는 engage 후에 — 스폰 즉시 달리기 시작하므로, engage 대기(10초 안팎)
        # 동안 도망가 버린다. 실측: 앞차가 차간 27→52 m 로 벌어져 추종 구간이 아예 없었다.
        static_objs = [o for o in self.spec.get("objects", []) if not o.get("velocity")]
        moving_objs = [o for o in self.spec.get("objects", []) if o.get("velocity")]
        self.spawn_objects(static_objs)
        self.engage()
        self.spawn_objects(moving_objs)
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
