"""주행을 실시간으로 들여다보는 HMI 형태의 웹 대시보드.

수집기(metrics_collector)는 "나중에 분석할 기록"을 남기고, 이 노드는 "지금 무슨 일이
일어나는지"를 보여준다. 둘의 목적이 다르므로 노드를 분리했다 — 대시보드가 죽어도
기록은 계속돼야 한다.

브라우저는 /data 를 주기적으로 읽고, 이 노드는 ROS 콜백에서 상태만 갱신한다.
rclpy 스핀과 HTTP 서버는 별도 스레드다.

사용:
    ros2 run autoware_bench dashboard [--ros-args -p port:=6100]
"""

import json
import math
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from autoware_perception_msgs.msg import PredictedObjects
from autoware_planning_msgs.msg import Trajectory
from tier4_metric_msgs.msg import MetricArray
from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState, VelocityFactorArray
from autoware_internal_planning_msgs.msg import PlanningFactorArray

HTML = Path(__file__).with_name("dashboard.html")

MODE = {0: "UNKNOWN", 1: "STOP", 2: "AUTONOMOUS", 3: "LOCAL", 4: "REMOTE"}
ROUTE = {0: "UNKNOWN", 1: "UNSET", 2: "SET", 3: "ARRIVED", 4: "CHANGING"}
FACTOR_STATUS = {0: "UNKNOWN", 1: "접근중", 2: "정지"}


class Rate:
    """토픽이 살아 있는지 보려고 최근 도착 시각만 들고 있는다 (벽시계 기준)."""

    def __init__(self, window=3.0):
        self.window = window
        self.stamps = []

    def tick(self):
        now = time.monotonic()
        self.stamps.append(now)
        self.stamps = [t for t in self.stamps if now - t < self.window]

    def hz(self):
        now = time.monotonic()
        recent = [t for t in self.stamps if now - t < self.window]
        return round(len(recent) / self.window, 1) if len(recent) > 1 else 0.0


class Dashboard(Node):
    def __init__(self):
        super().__init__("bench_dashboard")
        self.declare_parameter("port", 10100)
        self.declare_parameter("runs_dir", "/workspace/runs")
        self.port = int(self.get_parameter("port").value)
        self.runs_dir = Path(self.get_parameter("runs_dir").value)

        self.lock = threading.Lock()
        self.state = {
            "speed": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0,
            "mode": "-", "auto_available": False, "route": "-",
            "factors": [], "curve": None,
            "lat_dev": None, "steer": None, "objects": 0, "closest": None,
            "rates": {}, "run": None,
        }
        self.rate_traj = Rate()
        self.rate_odom = Rate()
        self.frame = None          # (bytes, width, height, encoding) — 최신 카메라 원본
        self.rate_cam = Rate()

        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, "/localization/kinematic_state", self.on_odom, 10)
        self.create_subscription(OperationModeState, "/api/operation_mode/state",
                                 self.on_mode, latched)
        self.create_subscription(RouteState, "/api/routing/state", self.on_route, latched)
        self.create_subscription(VelocityFactorArray, "/api/planning/velocity_factors",
                                 self.on_factors, 10)
        # 자체 모듈은 ADAPI 집계에 안 들어간다 (구독 목록이 소스에 하드코딩돼 있다)
        self.create_subscription(PlanningFactorArray,
                                 "/planning/planning_factors/curve_slowdown",
                                 self.on_curve, 10)
        self.create_subscription(MetricArray, "/control/control_evaluator/metrics",
                                 self.on_metrics, 10)
        self.create_subscription(PredictedObjects, "/perception/object_recognition/objects",
                                 self.on_objects, 10)
        self.create_subscription(Trajectory, "/planning/trajectory",
                                 lambda _m: self.rate_traj.tick(), 1)
        # 카메라는 보는 사람이 있을 때만 구독한다. 1920x1080 BGR 한 장이 6 MB 라
        # 상시 구독하면 아무도 안 봐도 초당 수십 MB 를 복사한다 (실측 CPU 23%).
        self.cam_sub = None
        self.cam_wanted = 0.0     # 마지막 /camera.jpg 요청 시각 (monotonic)

        self.create_timer(0.5, self.on_timer)

        threading.Thread(target=self.serve, daemon=True).start()
        self.get_logger().info(f"대시보드 http://0.0.0.0:{self.port}")

    # ---- 구독 ----

    def on_odom(self, msg):
        self.rate_odom.tick()
        q = msg.pose.pose.orientation
        with self.lock:
            self.state["speed"] = msg.twist.twist.linear.x
            self.state["x"] = msg.pose.pose.position.x
            self.state["y"] = msg.pose.pose.position.y
            self.state["yaw"] = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_mode(self, msg):
        with self.lock:
            self.state["mode"] = MODE.get(msg.mode, str(msg.mode))
            self.state["auto_available"] = bool(msg.is_autonomous_mode_available)

    def on_route(self, msg):
        with self.lock:
            self.state["route"] = ROUTE.get(msg.state, str(msg.state))

    def on_factors(self, msg):
        items = [{"module": f.behavior or "unknown",
                  "status": FACTOR_STATUS.get(f.status, str(f.status)),
                  "distance": round(f.distance, 1)} for f in msg.factors]
        with self.lock:
            self.state["factors"] = items

    def on_curve(self, msg):
        item = None
        for f in msg.factors:
            for cp in f.control_points:
                item = {"velocity": round(cp.velocity, 2), "distance": round(cp.distance, 1)}
        with self.lock:
            self.state["curve"] = item

    def on_metrics(self, msg):
        with self.lock:
            for m in msg.metric_array:
                if m.name == "lateral_deviation":
                    self.state["lat_dev"] = float(m.value)
                elif m.name == "steering_angle":
                    self.state["steer"] = float(m.value)

    def on_objects(self, msg):
        with self.lock:
            ex, ey = self.state["x"], self.state["y"]
        best = None
        for o in msg.objects:
            p = o.kinematics.initial_pose_with_covariance.pose.position
            d = math.hypot(p.x - ex, p.y - ey)
            best = d if best is None or d < best else best
        with self.lock:
            self.state["objects"] = len(msg.objects)
            self.state["closest"] = round(best, 1) if best is not None else None

    def cam_subscribe(self, on):
        """구독 생성·해제는 타이머(실행기 스레드)에서만 한다 — HTTP 스레드에서 만지면 위험하다."""
        if on and self.cam_sub is None:
            # AWSIM 카메라는 BEST_EFFORT 다. 기본 QoS 로 구독하면 한 장도 안 온다
            # (경고만 뜨고 조용히 빈다 — 러너에서 겪은 것과 같은 함정).
            sensor = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                durability=QoSDurabilityPolicy.VOLATILE)
            self.cam_sub = self.create_subscription(
                Image, "/sensing/camera/traffic_light/image_raw", self.on_image, sensor)
            self.get_logger().info("카메라 구독 시작")
        elif not on and self.cam_sub is not None:
            self.destroy_subscription(self.cam_sub)
            self.cam_sub = None
            with self.lock:
                self.frame = None
            self.get_logger().info("카메라 구독 해제 (보는 사람 없음)")

    def on_image(self, msg):
        """원본만 들고 있다가 브라우저가 요청할 때 인코딩한다 — 아무도 안 보면 비용 0."""
        if msg.encoding not in ("bgr8", "rgb8"):
            return
        self.rate_cam.tick()
        with self.lock:
            self.frame = (bytes(msg.data), msg.width, msg.height, msg.encoding)

    def jpeg(self, width=720, quality=6):
        self.cam_wanted = time.monotonic()
        """ffmpeg 로 인코딩한다. cv2 는 이 컨테이너에서 못 쓴다 — apt 의 cv2(4.5.4)가
        numpy 1.x 로 빌드됐는데 /usr/local 에 numpy 2.2.5 가 깔려 있어 import 가 깨진다.
        전역 numpy 를 내리면 분석 스크립트(compare_runs 등)가 깨지므로 건드리지 않는다."""
        with self.lock:
            frame = self.frame
        if frame is None:
            return None
        data, w, h, encoding = frame
        pix = "bgr24" if encoding == "bgr8" else "rgb24"
        try:
            out = subprocess.run(
                ["ffmpeg", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", pix,
                 "-s", f"{w}x{h}", "-i", "pipe:0", "-vf", f"scale={width}:-1",
                 "-frames:v", "1", "-q:v", str(quality), "-f", "mjpeg", "pipe:1"],
                input=data, capture_output=True, timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            return None
        return out.stdout or None

    def on_timer(self):
        """기록 상태는 토픽이 아니라 파일에서 본다 — 수집기와 결합하지 않으려고."""
        # 마지막 요청 후 10초가 지나면 카메라 구독을 끊는다 (탭을 닫으면 요청도 끊긴다)
        self.cam_subscribe(time.monotonic() - self.cam_wanted < 10.0)
        run = None
        try:
            files = sorted(self.runs_dir.glob("run_*.csv"), key=lambda p: p.stat().st_mtime)
            if files:
                f = files[-1]
                age = time.time() - f.stat().st_mtime
                run = {"name": f.name, "mb": round(f.stat().st_size / 1e6, 1),
                       "recording": age < 3.0}
        except OSError:
            pass
        with self.lock:
            self.state["run"] = run
            self.state["rates"] = {"trajectory": self.rate_traj.hz(),
                                   "odometry": self.rate_odom.hz(),
                                   "camera": self.rate_cam.hz()}

    # ---- HTTP ----

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def serve(self):
        dash = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def do_GET(self):
                if self.path.startswith("/camera.jpg"):
                    # 화면 폭·품질을 요청 쪽에서 정한다. egress 가 과금되는 환경이라
                    # "얼마나 쓸지"를 보는 사람이 고르게 한다 (PROGRESS.md 대역폭 과금 사고).
                    qs = dict(p.split("=", 1) for p in self.path.partition("?")[2].split("&")
                              if "=" in p)
                    try:
                        width = min(1280, max(160, int(qs.get("w", 720))))
                        quality = min(31, max(2, int(qs.get("q", 6))))
                    except ValueError:
                        width, quality = 720, 6
                    img = dash.jpeg(width, quality)
                    if img is None:
                        self.send_error(503, "no frame")
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(img)))
                    self.end_headers()
                    self.wfile.write(img)
                    return
                if self.path.startswith("/data"):
                    body = json.dumps(dash.snapshot()).encode()
                    ctype = "application/json"
                else:
                    body = HTML.read_bytes()
                    ctype = "text/html; charset=utf-8"
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        ThreadingHTTPServer(("0.0.0.0", dash.port), Handler).serve_forever()


def main():
    rclpy.init()
    node = Dashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
