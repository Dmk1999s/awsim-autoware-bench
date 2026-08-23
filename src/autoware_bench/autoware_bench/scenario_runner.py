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
import xml.etree.ElementTree as ET

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from tier4_simulation_msgs.msg import DummyObject
from autoware_perception_msgs.msg import (
    TrafficLightGroupArray, TrafficLightGroup, TrafficLightElement, PredictedObjects,
)
from autoware_planning_msgs.msg import LaneletRoute
from autoware_vehicle_msgs.msg import VelocityReport
from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState
from autoware_adapi_v1_msgs.srv import (
    ClearRoute, ChangeOperationMode, InitializeLocalization, SetRoutePoints,
)

# AWSIM 메뉴(☰)의 버튼 위치. ROS 인터페이스가 없어 GUI 를 누르는 수밖에 없다.
# 좌표는 **창 좌상단 기준 상대값**이다 — 절대좌표로 박아두면 창을 최대화하거나 옮긴 순간
# 클릭이 3D 화면에 떨어지고, 리셋이 조용히 실패한다. 그러면 차가 지난 주행의 도착지에
# 선 채로 다음 회차가 시작돼 "0.0 초 완주"가 성공으로 기록된다 (실측 8회차 손실).
AWSIM_EGO_RESET_REL = (115, 318)      # Ego Vehicle 의 ↻ — 스폰 좌표로 복귀
AWSIM_TRAFFIC_RESET_REL = (194, 426)  # Traffic Control 의 ↻ — 시드대로 NPC 재배치
AWSIM_MENU_REL = (25, 23)             # ☰ — 재기동 직후에는 메뉴가 접혀 있어 버튼이 없다
AWSIM_DISPLAY = ":20"
# AWSIM 런처의 Ego Position 기본값. 리셋이 실제로 먹었는지 확인하는 기준점이다 —
# 클릭이 빗나가도 아무 일도 일어나지 않으므로 결과를 봐야 안다 (WORKLOG 15·25).
AWSIM_SPAWN_XY = (81380.72, 49918.78)
SPAWN_TOLERANCE_M = 3.0
MIN_STRAIGHT_M = 20.0                 # 목적지가 이보다 가까우면 리셋 실패를 의심한다

MAP_OSM = "/root/awsim/nishishinjuku_autoware_map/lanelet2_map.osm"
# 미션 플래너는 짧은 경로가 불가능하면 거부하지 않고 우회를 조용히 받아들인다.
# 실험 5 에서 직행 150 m 목적지에 97-lanelet 블록 일주가 잡혔고, 400 s 를 다 쓰고서야
# 실패로 드러났다 (WORKLOG 10). 출발 전에 경로 길이를 직선거리와 대조해 거른다.
MAX_ROUTE_RATIO = 3.0

LOCALIZATION_TOLERANCE_M = 0.5
ENGAGE_ATTEMPTS = 5              # 전환이 간헐적으로 거부된다 — 아래 주석 참고   # 정답 대비 이보다 어긋나면 출발시키지 않는다


def lanelet_lengths(path):
    """lanelet id → 길이(m). osm 의 node 가 local_x/local_y 를 들고 있어 투영이 필요 없다.
    좌·우 경계 길이의 평균을 중심선 길이로 쓴다 — 경로 길이 검증에는 이 정밀도면 된다."""
    root = ET.parse(path).getroot()
    pt = {}
    for n in root.findall("node"):
        t = {g.get("k"): g.get("v") for g in n.findall("tag")}
        if "local_x" in t:
            pt[n.get("id")] = (float(t["local_x"]), float(t["local_y"]))
    way = {}
    for w in root.findall("way"):
        pts = [pt[nd.get("ref")] for nd in w.findall("nd") if nd.get("ref") in pt]
        way[w.get("id")] = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
    out = {}
    for r in root.findall("relation"):
        t = {g.get("k"): g.get("v") for g in r.findall("tag")}
        if t.get("type") != "lanelet":
            continue
        b = [way.get(m.get("ref"), 0.0)
             for m in r.findall("member") if m.get("role") in ("left", "right")]
        if b:
            out[int(r.get("id"))] = sum(b) / len(b)
    return out


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
        self.truth_vel = 0.0
        self.wheel_vel = 0.0   # 차량 인터페이스가 보고하는 속도 — 위치추정 API 가 보는 값
        self.spawned = []      # (uuid, 스펙) — 수명이 다하기 전에 다시 심으려고 들고 있는다
        self.spawn_time = 0.0
        self.route = None
        self.route_seq = 0  # TRANSIENT_LOCAL 이라 지난 주행의 경로가 먼저 온다 — 새 것만 인정

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(RouteState, "/api/routing/state",
                                 lambda m: setattr(self, "route_state", m.state), latched)
        self.create_subscription(
            OperationModeState, "/api/operation_mode/state", self.on_operation_mode, latched)
        self.create_subscription(LaneletRoute, "/planning/mission_planning/route",
                                 self.on_route, latched)
        self.create_subscription(Odometry, "/localization/kinematic_state",
                                 self.on_kinematic, 10)
        # 위치추정 초기화는 "The vehicle is not stopped" 로 거부되는데, 그 판단은 여기서 온다.
        # 시뮬레이터 정답 속도가 0 이어도 이 값이 남아 있으면 거부된다 (실측 5회 중 4회 실패).
        self.create_subscription(VelocityReport, "/vehicle/status/velocity_status",
                                 lambda m: setattr(self, "wheel_vel", m.longitudinal_velocity), 10)
        # AWSIM 의 정답 위치는 BEST_EFFORT 로 발행된다. 기본값(RELIABLE)으로 구독하면
        # QoS 불일치로 메시지가 한 개도 오지 않는다 — 경고만 뜨고 조용히 비어 있다.
        sensor = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, "/awsim/ground_truth/localization/kinematic_state",
                                 self.on_truth, sensor)
        # 스폰한 객체가 실제로 씬에 생겼는지 확인하려고 인지 결과를 본다 (아래 verify_spawn)
        self.objects = []
        self.create_subscription(
            PredictedObjects, "/perception/object_recognition/objects",
            lambda m: setattr(self, "objects", [
                (o.kinematics.initial_pose_with_covariance.pose.position.x,
                 o.kinematics.initial_pose_with_covariance.pose.position.y) for o in m.objects]), 10)

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

    def on_truth(self, msg):
        self.truth = msg.pose.pose.position
        # 리셋 직후 Autoware 의 속도 추정은 텔레포트를 몰라 못 믿는다 — 시뮬레이터 실측을 쓴다
        self.truth_vel = msg.twist.twist.linear.x

    def on_route(self, msg):
        self.route = msg
        self.route_seq += 1

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
            self.refresh_objects()
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

    def _awsim_origin(self):
        """AWSIM 창의 좌상단. 창을 옮기거나 최대화해도 버튼을 제대로 누르기 위한 것."""
        env = {"DISPLAY": AWSIM_DISPLAY, "PATH": "/usr/bin:/bin"}
        wid = subprocess.run(["xdotool", "search", "--onlyvisible", "--name", "AWSIM Labs"],
                             env=env, capture_output=True, text=True, check=True)
        ids = wid.stdout.split()
        if not ids:
            raise RuntimeError("AWSIM 창을 찾지 못했다 — 시뮬레이터가 떠 있는지 확인할 것")
        geo = subprocess.run(["xdotool", "getwindowgeometry", "--shell", ids[0]],
                             env=env, capture_output=True, text=True, check=True)
        vals = dict(line.split("=", 1) for line in geo.stdout.splitlines() if "=" in line)
        return int(vals["X"]), int(vals["Y"])

    def _click(self, rel):
        ox, oy = self._awsim_origin()
        subprocess.run(["xdotool", "mousemove", str(ox + rel[0]), str(oy + rel[1]), "click", "1"],
                       env={"DISPLAY": AWSIM_DISPLAY, "PATH": "/usr/bin:/bin"}, check=True)

    def reset_ego(self):
        """차와 NPC 교통을 둘 다 출발 상태로 되돌리고, **정말 되돌아갔는지 확인한다**.

        교통까지 리셋하는 이유: 시드를 고정해도 앞 주행이 흘려놓은 NPC 배치가 남아 있으면
        회차마다 조건이 다르다. 실제로 교통을 두고 5회 돌렸을 때 주행 시간이
        40.3~65.1 s (폭 24.8 s) 로 흔들렸고, 그 폭이 파라미터 효과를 덮을 만큼 컸다.

        확인하는 이유: 이 클릭은 두 번 조용히 빗나갔다 — 창을 최대화했을 때(WORKLOG 15)와
        재기동 직후 메뉴가 접혀 있을 때(WORKLOG 25). 둘 다 아무 일도 일어나지 않고,
        차는 지난 주행의 도착지에 그대로 서 있는다.
        """
        for attempt in (1, 2):
            self._click(AWSIM_TRAFFIC_RESET_REL)
            self.spin(1.0)
            self._click(AWSIM_EGO_RESET_REL)
            self.spin(4.0)
            if self.at_spawn():
                self.get_logger().info(f"AWSIM 리셋 확인 (시도 {attempt}회)")
                return
            if attempt == 1:
                # 메뉴가 접혀 있으면 버튼이 화면에 없다. ☰ 를 눌러 펴고 다시 시도한다.
                self.get_logger().warn("리셋이 안 먹었다 — 메뉴가 접혀 있는지 확인하고 재시도")
                self._click(AWSIM_MENU_REL)
                self.spin(1.5)
        raise RuntimeError(
            "AWSIM 리셋 실패 — 차가 스폰 위치로 돌아가지 않았다. "
            "메뉴 위치나 창 상태를 확인할 것")

    def at_spawn(self):
        end = time.time() + 6.0
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.truth is not None:
                d = math.hypot(self.truth.x - AWSIM_SPAWN_XY[0], self.truth.y - AWSIM_SPAWN_XY[1])
                if d < SPAWN_TOLERANCE_M:
                    return True
        return False

    def reinit_localization(self):
        """빈 pose = GNSS 자동 초기화. 그다음 정답과 대조해 실제로 수렴했는지 본다.

        리셋 직후에는 차가 아직 구르고 있어 초기화가 "The vehicle is not stopped" 로
        거부된다 (실측 3회 중 2회). 멈출 때까지 기다린다.
        """
        if not self.wait_until(
                lambda: abs(self.truth_vel) < 0.05 and abs(self.wheel_vel) < 0.05,
                20.0, "리셋 후 정차"):
            self.get_logger().warn("정차 확인 실패 — 그대로 초기화를 시도한다")
        for attempt in (1, 2, 3):
            status = self.call("init", InitializeLocalization.Request(pose=[]))
            if status.success:
                break
            self.get_logger().warn(f"위치추정 초기화 거부 ({attempt}/3): {status.message}")
            self.spin(3.0)
        else:
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
        seq0 = self.route_seq
        status = self.call("route", req)
        if not status.success:
            raise RuntimeError(f"경로 설정 거부: code={status.code} {status.message}")
        self.check_route_length(seq0)
        # routing/state 는 변화할 때만 발행된다. 구독 시점에 받은 지난 주행의 ARRIVED 가
        # 남아 있으면 drive() 가 즉시 "도착"으로 판정한다. 서비스가 성공했으므로 SET 으로 덮는다.
        self.route_state = RouteState.SET
        self.get_logger().info(f"경로 설정 → ({g['x']}, {g['y']})")

    def check_route_length(self, seq0):
        """잡힌 경로가 직선거리에 비해 터무니없이 길면 출발 전에 세운다."""
        if not self.wait_until(lambda: self.route_seq > seq0, 20.0, "경로 수신"):
            raise RuntimeError("경로 메시지가 오지 않음")
        ids = [seg.preferred_primitive.id for seg in self.route.segments]
        L = lanelet_lengths(MAP_OSM)
        length = sum(L.get(i, 0.0) for i in ids)
        a, b = self.route.start_pose.position, self.route.goal_pose.position
        straight = math.hypot(b.x - a.x, b.y - a.y)
        if straight < MIN_STRAIGHT_M:
            self.call("clear", ClearRoute.Request())
            raise RuntimeError(
                f"출발지와 목적지가 {straight:.1f} m 밖에 안 떨어져 있다 — "
                f"AWSIM 리셋이 실패해 차가 지난 주행의 도착지에 서 있을 수 있다")
        ratio = length / straight
        self.get_logger().info(
            f"경로 {len(ids)} lanelet / {length:.0f} m — 직선 {straight:.0f} m ({ratio:.1f}배)")
        limit = self.spec.get("max_route_ratio", MAX_ROUTE_RATIO)
        if ratio > limit:
            self.call("clear", ClearRoute.Request())   # 경로를 두고 죽으면 수집기가 계속 기록한다
            raise RuntimeError(
                f"경로가 직선거리의 {ratio:.1f}배 (한도 {limit}) — 우회 경로다. 출발하지 않는다")

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

    def spawn_objects(self, objs=None, quiet=False):
        """시나리오의 objects: 목록(또는 지정 목록)을 씬에 스폰한다."""
        if objs is None:
            objs = self.spec.get("objects", [])
        for o in objs:
            msg = DummyObject()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            # 메시지에 넣은 뒤 다시 읽으면 numpy 타입이 되어 재사용할 때 검증에 걸린다.
            # 원본 리스트를 그대로 보관한다.
            uid = [int(b) for b in uuid_lib.uuid4().bytes]
            msg.id.uuid = uid
            self.spawned.append((uid, o))
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
            if not quiet:
                self.get_logger().info(
                    f"객체 스폰: label={msg.classification.label} ({o['x']:.1f}, {o['y']:.1f}) "
                    f"v={o.get('velocity', 0.0)}")
        if objs:
            self.spawn_time = time.time()
            self.spin(2.0)

    def verify_spawn(self, objs, timeout=15.0):
        """스폰한 정지 객체가 인지에 잡히는지 확인한다.

        AWSIM 의 더미 객체 기능이 **조용히 죽는다**. ADD 를 발행해도 씬에 아무것도 안 생기고
        경고도 없다. 그 상태로 장애물 시나리오를 돌리면 차는 빈 도로를 달려 "완주"하고,
        배치는 그것을 성공으로 센다 — 실측으로 5회 연속 그렇게 집계됐다 (WORKLOG 24).
        AWSIM 재기동 외에는 복구되지 않으므로, 여기서 잡아 배치를 세운다.
        """
        targets = [(float(o["x"]), float(o["y"])) for o in objs]
        if not targets:
            return
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            seen = [t for t in targets
                    if any(math.hypot(ox - t[0], oy - t[1]) < 4.0 for ox, oy in self.objects)]
            if len(seen) == len(targets):
                self.get_logger().info(f"스폰 확인 — 객체 {len(seen)}개 인지됨")
                return
        raise RuntimeError(
            f"스폰한 객체가 인지되지 않는다 ({len(targets)}개 중 "
            f"{len(seen)}개) — AWSIM 더미 기능이 죽었을 수 있다. AWSIM 재기동 필요")

    def refresh_objects(self):
        """AWSIM 더미 객체는 **스폰 후 약 30 초면 스스로 사라진다** (실측 30.5·30.8 s).
        MODIFY 로도 갱신되지 않는다. 자차가 도달하기 전에 없어지면 차는 빈 도로를 달리고,
        그 주행이 "장애물 시나리오 통과"로 기록된다 — 실측 5회 연속 그랬다 (WORKLOG 26).
        그래서 25 초마다 지우고 다시 심는다. 정지 객체만 대상이다 (움직이는 객체를 다시
        심으면 위치가 되돌아가 추종 실험이 깨진다).
        """
        statics = [(u, o) for u, o in self.spawned if not o.get("velocity")]
        if not statics or time.time() - self.spawn_time < 20.0:
            return
        # **먼저 새로 심고, 그다음 옛것을 지운다.** 반대로 하면 그 사이에 객체가 없는 순간이
        # 생기고, 그때 계획이 정지를 풀어 차가 그대로 지나간다 (실측 5회 중 3회 관통).
        self.spawn_objects([o for _, o in statics], quiet=True)
        self.spin(0.5)
        for uid, _ in statics:
            msg = DummyObject()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.id.uuid = uid
            msg.action = DummyObject.DELETE
            self.obj_pub.publish(msg)
        self.spawned = [(u, o) for u, o in self.spawned
                        if o.get("velocity") or (u, o) not in statics]
        self.get_logger().info(f"객체 다시 심음 ({len(statics)}개) — 수명 30 초 대응")

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
                self.refresh_objects()
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
                self.refresh_objects()
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
                self.refresh_objects()
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
        self.verify_spawn(static_objs)
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
    except Exception as e:  # noqa: BLE001 — 예상 밖 예외도 로그로 남겨야 배치에서 진단된다
        # 로그를 grep 으로 걸러 보기 때문에, 트레이스백만 남으면 배치 로그에서 사라진다.
        # 실측: xdotool 실패로 러너가 죽었는데 배치에는 "실패 (종료코드 1)" 만 남았다.
        node.get_logger().error(f"예상 밖 예외: {type(e).__name__}: {e}")
        code = 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
