#!/usr/bin/env python3
"""
SIH-Drone Command Center backend.

Runs real subprocesses for the Gazebo/ArduPilot sim, the MAVLink/ROS2
mission bridge, and the YOLO person detector; bridges ROS2 topics and
services to a small REST/SSE/MJPEG API for the command_center/frontend
web page.
"""
import json
import math
import os
import random
import re
import signal
import subprocess
import threading
import time
from collections import deque

import cv2
import rclpy
from cv_bridge import CvBridge
from flask import Flask, Response, request, send_from_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from vision_msgs.msg import Detection2DArray

from alerts import Notifier

HOME = os.path.expanduser("~")
BACKEND_DIR = os.path.dirname(os.path.abspath(__file__))
FRONTEND_DIR = os.path.join(os.path.dirname(BACKEND_DIR), "frontend")
PROJECT_ROOT = os.path.dirname(os.path.dirname(BACKEND_DIR))  # project root
ROS_ENV = os.path.join(BACKEND_DIR, "ros_env.sh")

# Worlds in ardupilot_gz_gazebo/worlds the dashboard can launch. Both share
# the frame, casualty and lights that the bridge, detector and lighting
# below rely on; only the Gazebo world name differs.
WORLDS = {
    "disaster": "Post-disaster zone",
    "runway": "Runway (flat)",
    "region": "Region 200 m (city, forest, village, river)",
    "forest": "Forest edge 200 m (elephant corridor, camera towers, village)",
}
# Mission intel per world (the bridge reads RESQ_MISSION_FILE); worlds not
# listed use the default resq_mavlink/mission.json.
MISSION_FILES = {
    "region": os.path.join(PROJECT_ROOT, "resq_mavlink", "mission_region.json"),
    "forest": os.path.join(PROJECT_ROOT, "resq_mavlink", "mission_forest.json"),
}
# Keeps moving casualties' heat bodies on their actors (region world).
HEAT_FOLLOWER_CMD = [ROS_ENV, "python3", os.path.join(PROJECT_ROOT, "worldgen", "heat_follower.py")]
HEAT_FOLLOWER_WORLDS = {"region"}
# Walks the forest world's animals along their scripted routes; the herd and
# the tusker wait for /api/wildlife/scenario.
ANIMAL_MOVER_CMD = [ROS_ENV, "python3", os.path.join(PROJECT_ROOT, "worldgen", "animal_mover.py")]
ANIMAL_WORLDS = {"forest"}
# Worlds generated per run: casualty placement comes from a seed (the
# forest's scenario is scripted; its generator ignores the seed).
GENERATORS = {
    "region": os.path.join(PROJECT_ROOT, "worldgen", "generate_region.py"),
    "forest": os.path.join(PROJECT_ROOT, "worldgen", "generate_forest.py"),
}
DEFAULT_WORLD = "disaster"


# RViz is off by default: the dashboard shows everything it did, and
# without it the sim runs a little faster (measured 0.85x vs 0.82x real
# time). RESQ_RVIZ=1 ./start.sh opens it anyway.
RVIZ = os.environ.get("RESQ_RVIZ", "0") == "1"
# RESQ_GZ_GUI=0: no Gazebo window. The cameras render in the sim server
# either way; the window alone took ~2 CPU cores, which on a laptop
# throttling at 94 C cost the sim speed and so the video frame rate.
GZ_GUI = os.environ.get("RESQ_GZ_GUI", "1") != "0"


def sim_launch_cmd(world):
    return [
        ROS_ENV, "ros2", "launch", "ardupilot_gz_bringup",
        "s500_quad_runway.launch.py", f"world:={world}", f"rviz:={'true' if RVIZ else 'false'}",
        "use_gz_sim_gui:=true" if GZ_GUI else "use_gz_sim_gui:=false",
        "use_gz_tf:=true",
    ]


current_world = DEFAULT_WORLD
BRIDGE_CMD = [ROS_ENV, "ros2", "run", "resq_mavlink_bridge", "mavlink_bridge"]

# Real-drone mode (./start.sh real [device] [baud]): no Gazebo. "Connect
# Drone" starts MAVProxy on the telemetry radio and splits the link:
# UDP 14550 for the mission bridge (the same port it uses with the
# simulator), 14551 for Mission Planner / QGroundControl as a safety
# monitor, 14552 for resq_mavlink/preflight.py.
REAL = os.environ.get("RESQ_REAL", "0") == "1"
LINK_DEVICE = os.environ.get("RESQ_LINK", "/dev/ttyUSB0")
LINK_BAUD = os.environ.get("RESQ_BAUD", "57600")


def link_cmd():
    return ["mavproxy.py", f"--master={LINK_DEVICE}", f"--baudrate={LINK_BAUD}",
            "--out=udp:127.0.0.1:14550", "--out=udp:127.0.0.1:14551", "--out=udp:127.0.0.1:14552",
            "--streamrate=10", "--non-interactive"]
FEED_CMD = [ROS_ENV, "python3", os.path.join(PROJECT_ROOT, "ardu_ws", "person_detector.py")]
# The forest world's "feed" is Sentinel: towers + the drone's forward camera.
FEED_CMDS = {"forest": [ROS_ENV, "python3", os.path.join(PROJECT_ROOT, "ardu_ws", "sentinel_detector.py")]}
def _tower_ids():
    """Camera ids of the forest world (towers and trail cameras), lower case."""
    try:
        with open(os.path.join(PROJECT_ROOT, "resq_mavlink", "mission_forest.json")) as f:
            return tuple(t["id"].lower() for t in json.load(f).get("towers", []))
    except (OSError, ValueError):
        return ("t1", "t2", "t3")


TOWER_IDS = _tower_ids()
SNAPSHOT_DIR = os.path.join(PROJECT_ROOT, "logs", "sentinel")

# Day/night lighting for the worlds' "sun" and "fill" lights (identical in
# runway.sdf and disaster.sdf), applied live through Gazebo's
# /world/<world>/light_config service. That service replaces the whole
# light, so every field (direction, attenuation, ...) is sent.
SUN_BASE = (
    'type: DIRECTIONAL pose {{ position {{ z: 10 }} }} direction {{ x: -0.5 y: 0.1 z: -0.9 }} '
    'range: 1000 attenuation_constant: 0.9 attenuation_linear: 0.01 attenuation_quadratic: 0.001 '
    'intensity: 1 '
)
FILL_BASE = 'type: DIRECTIONAL pose {{ position {{ z: 10 }} }} direction {{ z: -1 }} range: 1000 intensity: 1 '
LIGHTING = {
    # Matches the worlds' defaults.
    "day": [
        ("sun", SUN_BASE + 'diffuse {{ r: 0.7 g: 0.7 b: 0.7 a: 1 }} specular {{ r: 0.8 g: 0.8 b: 0.8 a: 1 }} cast_shadows: true'),
        ("fill", FILL_BASE + 'diffuse {{ r: 0.4 g: 0.4 b: 0.4 a: 1 }} specular {{ a: 1 }} cast_shadows: false'),
    ],
    # Sun becomes dim bluish moonlight, fill off.
    "night": [
        ("sun", SUN_BASE + 'diffuse {{ r: 0.05 g: 0.06 b: 0.09 a: 1 }} specular {{ a: 1 }} cast_shadows: false'),
        ("fill", FILL_BASE + 'diffuse {{ a: 1 }} specular {{ a: 1 }} cast_shadows: false'),
    ],
}
# Models moved for night, as {world: {model: (day xyz, night xyz)}}. A
# world's sky/background can't change while the sim runs, so each world
# has two sky domes (sky_day, sky_night) and the unused one is parked out
# of sight. The smoke plumes are hidden at night: particles are unlit and
# glow white against a dark sky. Must match the poses in the world files.
HIDDEN = (0, 0, -100000)
NIGHT_POSES = {
    "disaster": {
        "sky_day": ((0, 0, 0), HIDDEN),
        "sky_night": (HIDDEN, (0, 0, 0)),
        "smoke_industrial": ((46, 20, 6), (46, 20, -100000)),
        "smoke_industrial_2": ((54, 27, 3), (54, 27, -100000)),
        "smoke_fire_station": ((2, 56, 4), (2, 56, -100000)),
    },
    "runway": {
        "sky_day": ((0, 0, 0), HIDDEN),
        "sky_night": (HIDDEN, (0, 0, 0)),
    },
    "region": {
        "sky_day": ((0, 0, 0), HIDDEN),
        "sky_night": (HIDDEN, (0, 0, 0)),
        "smoke_fire_city": ((50, 92, 3), (50, 92, -100000)),
    },
    "forest": {
        "sky_day": ((0, 0, 0), HIDDEN),
        "sky_night": (HIDDEN, (0, 0, 0)),
    },
}
SENSOR_MODE_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
# Raw thermal view for the dashboard's split / thermal feed: same white-hot
# window and colour map as person_detector.py (0.01 K per unit).
THERMAL_WINDOW_K = (283.0, 313.0)
THERMAL_UNITS_PER_K = 100.0
THERMAL_FEED_HZ = 20.0


def _gz_service(service, reqtype, req):
    """Call a Gazebo service for the running world; returns (ok, detail)."""
    try:
        result = subprocess.run(
            [ROS_ENV, "gz", "service", "-s", f"/world/{current_world}/{service}",
             "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
            capture_output=True, text=True, timeout=10,
        )
    except subprocess.TimeoutExpired:
        return False, "timed out"
    if result.returncode != 0 or "true" not in result.stdout:
        return False, (result.stdout + result.stderr).strip()[:200]
    return True, ""


def apply_lighting(mode):
    """Switch the sim's lights and sky; returns (ok, message)."""
    for name, fields in LIGHTING[mode]:
        ok, detail = _gz_service("light_config", "gz.msgs.Light", f'name: "{name}" ' + fields.format())
        if not ok:
            return False, f"light '{name}' not updated: {detail}"
    for name, poses in NIGHT_POSES.get(current_world, {}).items():
        x, y, z = poses[1] if mode == "night" else poses[0]
        ok, detail = _gz_service("set_pose", "gz.msgs.Pose",
                                 f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}')
        if not ok:
            return False, f"'{name}' not moved: {detail}"
    return True, f"lighting and sky set to {mode}"

# Same processes as the pkill sequence run by hand between sim runs,
# but specific: a bare "gz" or "ardupilot" pattern (as in the manual
# version) kills ANY process whose command line merely contains that
# text, e.g. an editor with an ArduPilot source file open.
KILL_PATTERNS = [
    "ros2 launch ardupilot_gz_bringup", "gz sim", "arducopter", "rviz2",
    "robot_state_publisher", "parameter_bridge", "micro_ros_agent", "mavproxy.py",
    "resq_mavlink_bridge", "person_detector.py", "heat_follower.py", "animal_mover.py",
]

STATE_RE = re.compile(
    r"Armed:\s*(?P<armed>\w+),\s*Mode:\s*(?P<mode>\S+),\s*"
    r"Battery:\s*(?P<battery_pct>-?\d+)%,\s*Voltage:\s*(?P<voltage>[\d.]+)\s*V,\s*"
    r"Altitude:\s*(?P<altitude>-?[\d.]+)\s*m,\s*Mission:\s*(?P<mission_stage>\w+)"
    r"(?:,\s*Search:\s*(?P<search_done>\d+)/(?P<search_total>\d+))?"
)
POSITION_RE = re.compile(
    r"Latitude:\s*(?P<lat>-?[\d.]+),\s*Longitude:\s*(?P<lon>-?[\d.]+),\s*"
    r"Altitude:\s*(?P<altitude>-?[\d.]+)\s*m"
)


class LogHub:
    """Shared ring buffer of log lines, fanned out to SSE clients."""

    def __init__(self, maxlen=2000):
        self.lines = deque(maxlen=maxlen)
        self.total = 0
        self.cond = threading.Condition()

    def push(self, source, line):
        with self.cond:
            self.lines.append(f"[{source}] {line}")
            self.total += 1
            self.cond.notify_all()

    def stream(self):
        # Track a running line count, not a buffer index: once the ring
        # buffer is full its length stops growing, and an index-based
        # cursor would never see another line.
        seen = 0
        while True:
            with self.cond:
                self.cond.wait_for(lambda: self.total > seen, timeout=15.0)
                new_count = self.total - seen
                pending = list(self.lines)[-new_count:] if new_count else []
                seen = self.total
            if not pending:
                yield ": keepalive\n\n"
            for line in pending:
                yield f"data: {json.dumps(line)}\n\n"


log_hub = LogHub()


class ProcessManager:
    """Tracks the long-running subprocesses the GUI can start/stop."""

    def __init__(self):
        self.procs = {}
        self.lock = threading.Lock()

    def _reader(self, name, proc):
        for raw in iter(proc.stdout.readline, b""):
            line = raw.decode("utf-8", errors="replace").rstrip()
            if line:
                log_hub.push(name, line)
        log_hub.push(name, "[process exited]")

    def start(self, name, cmd, env=None):
        with self.lock:
            existing = self.procs.get(name)
            if existing and existing.poll() is None:
                return False, f"{name} is already running."
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
                cwd=HOME,
                env=env,
            )
            self.procs[name] = proc
        threading.Thread(target=self._reader, args=(name, proc), daemon=True).start()
        log_hub.push(name, f"started (pid {proc.pid})")
        return True, f"{name} launched (pid {proc.pid})."

    def stop(self, name):
        with self.lock:
            proc = self.procs.pop(name, None)
        if not proc:
            return False, f"{name} is not running."
        was_running = proc.poll() is None
        # Signal the whole group (pgid == leader pid, via setsid) even if
        # the leader already exited: e.g. `ros2 launch` dying still leaves
        # gz sim / arducopter alive in its group.
        for sig in (signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            try:
                proc.wait(timeout=5)
                time.sleep(0.5)
            except subprocess.TimeoutExpired:
                pass
        return True, f"{name} stopped." if was_running else f"{name} cleaned up."

    def is_running(self, name):
        with self.lock:
            proc = self.procs.get(name)
        return bool(proc and proc.poll() is None)

    def kill_all_fallback(self):
        """Safety net for orphaned gz/ardupilot/ros2 processes, mirroring
        the pkill sequence the user runs by hand between sim runs."""
        for pattern in KILL_PATTERNS:
            subprocess.run(["pkill", "-9", "-f", pattern], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)


procs = ProcessManager()
notifier = Notifier(log_hub.push)


class RosBridge(Node):
    def __init__(self):
        super().__init__("command_center_bridge")
        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        # Wakes MJPEG streams on every new frame (detector or thermal).
        self.frame_cond = threading.Condition(self.frame_lock)
        self.frame_seq = 0
        self.latest_jpeg = None
        self.state_lock = threading.Lock()
        self.telemetry = {
            "armed": False,
            "mode": "UNKNOWN",
            "battery_pct": -1,
            "battery_voltage": 0.0,
            "altitude": 0.0,
            "latitude": 0.0,
            "longitude": 0.0,
            "mission_stage": "IDLE",
            "search_done": 0,
            "search_total": 0,
            "persons_detected": 0,
            "person_confirmed": False,
            "payload_dropped": False,
            "sensor_mode": "day",
        }

        self.create_subscription(String, "/resq/drone/state", self._on_state, 10)
        self.create_subscription(String, "/resq/drone/position", self._on_position, 10)
        self.create_subscription(
            Detection2DArray, "/detections/persons", self._on_persons,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool, "/detections/person_confirmed", self._on_person_confirmed, 10,
        )
        # The detector publishes JPEGs; they go to the browser as they are.
        self.create_subscription(
            CompressedImage, "/detections/debug_image/compressed", self._on_frame,
            qos_profile_sensor_data,
        )
        self.latest_thermal_jpeg = None
        self.last_thermal_time = 0.0
        self.create_subscription(Image, "/thermal/image", self._on_thermal, qos_profile_sensor_data)

        self.map_lock = threading.Lock()
        self.latest_map = None  # JSON text from the bridge's /resq/map
        self.create_subscription(String, "/resq/map", self._on_map, 10)

        # Sentinel (forest world): tower feeds, sightings, and reports from
        # people / imported alerts going the other way onto the same topic.
        self.tower_jpegs = {}
        for tid in TOWER_IDS:
            self.create_subscription(CompressedImage, f"/sentinel/{tid}/image/compressed",
                                     lambda msg, t=tid: self._on_tower(t, msg), qos_profile_sensor_data)
        self.create_subscription(String, "/sentinel/events", self._on_sentinel_event, 50)
        self.sentinel_pub = self.create_publisher(String, "/sentinel/events", 50)
        # Which feeds someone is watching: Sentinel encodes those at full
        # rate and the others at 1 fps (enough for thumbnails), so eight
        # cameras don't cost eight live JPEG encoders.
        self.viewing_pub = self.create_publisher(String, "/sentinel/viewing", SENSOR_MODE_QOS)
        self.viewers = {}
        self.viewers_lock = threading.Lock()
        self.viewing_pub.publish(String(data="[]"))

        self.mission_start_client = self.create_client(SetBool, "/resq/mission/start")
        self.mission_land_client = self.create_client(Trigger, "/resq/mission/land")
        self.mission_return_client = self.create_client(Trigger, "/resq/mission/return")
        self.payload_drop_client = self.create_client(Trigger, "/resq/payload/drop")
        self.mode_pub = self.create_publisher(String, "/resq/sensor_mode", SENSOR_MODE_QOS)
        self.config_pub = self.create_publisher(String, "/resq/mission/config", SENSOR_MODE_QOS)
        self.set_mode("day")

    def _on_map(self, msg):
        with self.map_lock:
            self.latest_map = msg.data
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if data.get("payload_dropped", False):
            self.set_payload_dropped(True)
        if "wildlife" in data:
            notifier.on_map(data)

    def _on_tower(self, tid, msg):
        with self.frame_lock:
            self.tower_jpegs[tid] = bytes(msg.data)
            self.frame_seq += 1
            self.frame_cond.notify_all()

    def get_tower_frame(self, tid):
        with self.frame_lock:
            return self.tower_jpegs.get(tid)

    def _on_sentinel_event(self, msg):
        try:
            notifier.on_event(json.loads(msg.data))
        except ValueError:
            pass

    def watch(self, key, delta):
        with self.viewers_lock:
            self.viewers[key] = max(0, self.viewers.get(key, 0) + delta)
            watched = sorted(k for k, n in self.viewers.items() if n > 0)
        self.viewing_pub.publish(String(data=json.dumps(watched)))

    def publish_sentinel_event(self, event):
        self.sentinel_pub.publish(String(data=json.dumps(event)))

    def get_map(self):
        with self.map_lock:
            return self.latest_map

    def clear_map(self):
        with self.map_lock:
            self.latest_map = None

    def publish_mission_config(self, config):
        self.config_pub.publish(String(data=json.dumps(config)))

    def set_mode(self, mode):
        with self.state_lock:
            self.telemetry["sensor_mode"] = mode
        self.mode_pub.publish(String(data=mode))

    def get_mode(self):
        with self.state_lock:
            return self.telemetry["sensor_mode"]

    def _on_state(self, msg):
        match = STATE_RE.search(msg.data)
        if not match:
            return
        with self.state_lock:
            self.telemetry["armed"] = match.group("armed") == "True"
            self.telemetry["mode"] = match.group("mode")
            self.telemetry["battery_pct"] = int(match.group("battery_pct"))
            self.telemetry["battery_voltage"] = float(match.group("voltage"))
            self.telemetry["altitude"] = float(match.group("altitude"))
            self.telemetry["mission_stage"] = match.group("mission_stage")
            if match.group("search_total") is not None:
                self.telemetry["search_done"] = int(match.group("search_done"))
                self.telemetry["search_total"] = int(match.group("search_total"))

    def _on_position(self, msg):
        match = POSITION_RE.search(msg.data)
        if not match:
            return
        with self.state_lock:
            self.telemetry["latitude"] = float(match.group("lat"))
            self.telemetry["longitude"] = float(match.group("lon"))

    def _on_persons(self, msg):
        with self.state_lock:
            self.telemetry["persons_detected"] = len(msg.detections)

    def _on_person_confirmed(self, msg):
        with self.state_lock:
            self.telemetry["person_confirmed"] = msg.data

    def _on_frame(self, msg):
        with self.frame_lock:
            self.latest_jpeg = bytes(msg.data)
            self.frame_seq += 1
            self.frame_cond.notify_all()

    def _on_thermal(self, msg):
        now = time.time()
        if now - self.last_thermal_time < 1.0 / THERMAL_FEED_HZ:
            return
        self.last_thermal_time = now
        try:
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001 - log and drop a bad frame
            self.get_logger().warn(f"thermal decode failed: {exc}")
            return
        lo, hi = THERMAL_WINDOW_K
        kelvin = raw.astype("float32") / THERMAL_UNITS_PER_K
        grey = ((kelvin - lo) / (hi - lo) * 255.0).clip(0, 255).astype("uint8")
        ok, jpeg = cv2.imencode(".jpg", cv2.applyColorMap(grey, cv2.COLORMAP_INFERNO),
                                [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self.frame_lock:
                self.latest_thermal_jpeg = jpeg.tobytes()
                self.frame_seq += 1
                self.frame_cond.notify_all()

    def get_thermal_frame(self):
        with self.frame_lock:
            return self.latest_thermal_jpeg

    def get_telemetry(self):
        with self.state_lock:
            return dict(self.telemetry)

    def set_payload_dropped(self, value):
        with self.state_lock:
            self.telemetry["payload_dropped"] = value

    def get_frame(self):
        with self.frame_lock:
            return self.latest_jpeg

    def clear_frame(self):
        with self.frame_lock:
            self.latest_jpeg = None
            self.latest_thermal_jpeg = None
            self.tower_jpegs = {}

    def call_mission_start(self, start):
        if not self.mission_start_client.wait_for_service(timeout_sec=2.0):
            return False, "mission service unavailable (is the sim launched?)"
        req = SetBool.Request()
        req.data = start
        future = self.mission_start_client.call_async(req)
        return self._wait_future(future)

    def call_payload_drop(self):
        if not self.payload_drop_client.wait_for_service(timeout_sec=2.0):
            return False, "payload service unavailable (is the drone connected?)"
        return self._wait_future(self.payload_drop_client.call_async(Trigger.Request()))

    def call_mission_return(self):
        if not self.mission_return_client.wait_for_service(timeout_sec=2.0):
            return False, "return service unavailable (is the sim launched?)"
        return self._wait_future(self.mission_return_client.call_async(Trigger.Request()))

    def call_mission_land(self):
        if not self.mission_land_client.wait_for_service(timeout_sec=2.0):
            return False, "land service unavailable (is the sim launched?)"
        req = Trigger.Request()
        future = self.mission_land_client.call_async(req)
        return self._wait_future(future)

    @staticmethod
    def _wait_future(future, timeout_sec=10.0):
        deadline = time.time() + timeout_sec
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done():
            return False, "service call timed out"
        result = future.result()
        if result is None:
            return False, "service call failed"
        return bool(result.success), result.message


ros_bridge = None


def start_ros_bridge():
    global ros_bridge
    rclpy.init()
    ros_bridge = RosBridge()
    executor_thread = threading.Thread(
        target=rclpy.spin, args=(ros_bridge,), daemon=True
    )
    executor_thread.start()


app = Flask(__name__, static_folder=None)


@app.get("/")
def index():
    # The forest world (PS000183) gets the simple wildlife console; SAR
    # worlds keep the original dashboard (also at /classic).
    if current_world in ANIMAL_WORLDS:
        return send_from_directory(FRONTEND_DIR, "live.html")
    return send_from_directory(FRONTEND_DIR, "index.html")


@app.get("/<path:filename>")
def static_files(filename):
    return send_from_directory(FRONTEND_DIR, filename)


def _json_ok(message, **extra):
    return {"ok": True, "message": message, **extra}


def _json_err(message, **extra):
    return {"ok": False, "message": message, **extra}, 400


@app.post("/api/sim/launch")
def sim_launch():
    global current_world
    body = request.get_json(silent=True) or {}
    world = str(body.get("world", DEFAULT_WORLD)).lower()
    if world not in WORLDS:
        return _json_err(f"unknown world '{world}' (expected one of: {', '.join(WORLDS)})")
    if procs.is_running("sim"):
        return _json_err(f"sim already running ({current_world}); stop it first.")
    if REAL:
        ros_bridge.clear_frame()
        ros_bridge.clear_map()
        ros_bridge.set_payload_dropped(False)
        ok1, msg1 = procs.start("sim", link_cmd())
        time.sleep(2.0)
        env = dict(os.environ)
        env.pop("RESQ_MISSION_FILE", None)
        ok2, msg2 = procs.start("bridge", BRIDGE_CMD, env=env)
        log_hub.push("link", f"MAVProxy on {LINK_DEVICE} @ {LINK_BAUD}: bridge on UDP 14550, "
                             "ground station on UDP 14551, preflight check on UDP 14552")
        if not (ok1 or ok2):
            return _json_err(f"{msg1} {msg2}")
        return _json_ok(f"Drone link started ({LINK_DEVICE}). {msg2}")
    if world in GENERATORS:
        # New casualty placement for this run (scenery is fixed and prebuilt).
        seed = body.get("seed")
        try:
            seed = int(seed) if seed not in (None, "", "random") else random.randint(1, 99999)
        except (TypeError, ValueError):
            return _json_err(f"seed must be a number, not {seed!r}")
        result = subprocess.run(["python3", GENERATORS[world], "--seed", str(seed)],
                                capture_output=True, text=True, timeout=60)
        if result.returncode != 0:
            return _json_err(f"world generation failed: {(result.stderr or result.stdout).strip()[-300:]}")
        log_hub.push("worldgen", result.stdout.strip())
    current_world = world
    ros_bridge.clear_frame()
    ros_bridge.clear_map()
    ros_bridge.set_payload_dropped(False)
    ok1, msg1 = procs.start("sim", sim_launch_cmd(world))
    bridge_env = dict(os.environ)
    if world in MISSION_FILES:
        bridge_env["RESQ_MISSION_FILE"] = MISSION_FILES[world]
    else:
        bridge_env.pop("RESQ_MISSION_FILE", None)
    ok2, msg2 = procs.start("bridge", BRIDGE_CMD, env=bridge_env)
    if ok1 and world in HEAT_FOLLOWER_WORLDS:
        procs.start("heat", HEAT_FOLLOWER_CMD)
    if ok1 and world in ANIMAL_WORLDS:
        procs.start("animals", ANIMAL_MOVER_CMD)
    if ok1:
        threading.Thread(target=_reapply_lighting_when_ready, daemon=True).start()
    if not (ok1 or ok2):
        return _json_err(f"{msg1} {msg2}")
    return _json_ok(f"{msg1} {msg2}")


@app.post("/api/sim/stop")
def sim_stop():
    procs.stop("heat")
    procs.stop("animals")
    procs.stop("bridge")
    procs.stop("sim")
    procs.stop("feed")
    procs.kill_all_fallback()
    ros_bridge.clear_frame()
    return _json_ok("Simulation stack stopped.")


@app.post("/api/feed/start")
def feed_start():
    ok, msg = procs.start("feed", FEED_CMDS.get(current_world, FEED_CMD))
    return _json_ok(msg) if ok else _json_err(msg)


@app.post("/api/feed/stop")
def feed_stop():
    ok, msg = procs.stop("feed")
    ros_bridge.clear_frame()
    return _json_ok(msg) if ok else _json_err(msg)


PLANNERS = ("bayes", "grid")
ON_CONFIRM = ("continue", "land")
# Wildlife settings the dashboard changes between missions (auto-dispatch on
# or off). The bridge's config is one latched message, so they ride along
# with every config sent.
wildlife_settings = {}


@app.post("/api/wildlife/config")
def wildlife_config():
    body = request.get_json(silent=True) or {}
    if "auto_dispatch" in body:
        wildlife_settings["auto_dispatch"] = bool(body["auto_dispatch"])
    ros_bridge.publish_mission_config(dict(wildlife_settings))
    state = "armed" if wildlife_settings.get("auto_dispatch") else "off"
    log_hub.push("wildlife", f"auto-dispatch {state}")
    return _json_ok(f"Auto-dispatch {state}.", **wildlife_settings)


@app.post("/api/mission/start")
def mission_start():
    body = request.get_json(silent=True) or {}
    config = dict(wildlife_settings)
    if body.get("planner") in PLANNERS:
        config["planner"] = body["planner"]
    if body.get("on_confirm") in ON_CONFIRM:
        config["on_confirm"] = body["on_confirm"]
    if body.get("mission_type") in ("search", "hop", "wildlife"):
        config["mission_type"] = body["mission_type"]
    for key in ("hop_altitude_m", "hop_hold_s"):
        if body.get(key) not in (None, ""):
            try:
                config[key] = float(body[key])
            except (TypeError, ValueError):
                return _json_err(f"{key} must be a number")
    # Latched; the bridge reads it when the mission starts. Anything not
    # given here falls back to mission.json.
    ros_bridge.publish_mission_config(config)
    time.sleep(0.3)
    ok, msg = ros_bridge.call_mission_start(True)
    return _json_ok(msg) if ok else _json_err(msg)


@app.post("/api/mission/return")
def mission_return():
    ok, msg = ros_bridge.call_mission_return()
    return _json_ok(msg) if ok else _json_err(msg)


@app.post("/api/wildlife/broadcast")
def wildlife_broadcast():
    """Operator's "Warn village": a message to the village channel now."""
    body = request.get_json(silent=True) or {}
    latest = ros_bridge.get_map()
    herd = None
    if latest:
        try:
            herd = (json.loads(latest).get("wildlife") or {}).get("herd")
        except ValueError:
            pass
    notifier.manual(str(body.get("note", "")).strip()[:300], herd)
    log_hub.push("alerts", "manual village warning sent by the operator")
    return _json_ok("Warning sent to the village.")


@app.get("/snapshot/<src>")
def snapshot(src):
    """Latest frame of one camera as a single JPEG (cheap thumbnails)."""
    frame = ros_bridge.get_frame() if src == "drone" else ros_bridge.get_tower_frame(src) if src in TOWER_IDS else None
    resp = Response(frame or _placeholder_jpeg(), mimetype="image/jpeg")
    resp.headers["Cache-Control"] = "no-store"
    return resp


@app.get("/classic")
def classic_dashboard():
    """The original (SAR) dashboard, still available in the forest world."""
    return send_from_directory(FRONTEND_DIR, "index.html")


@app.post("/api/mission/land")
def mission_land():
    ok, msg = ros_bridge.call_mission_land()
    return _json_ok(msg) if ok else _json_err(msg)


def _reapply_lighting_when_ready(timeout_s=180):
    """A fresh sim starts with the world's daytime lights; if night mode
    is selected, re-apply it as soon as Gazebo's light service is up."""
    deadline = time.time() + timeout_s
    while time.time() < deadline and procs.is_running("sim"):
        mode = ros_bridge.get_mode()
        if mode == "day":
            return
        ok, msg = apply_lighting(mode)
        if ok:
            log_hub.push("mode", f"{msg} (re-applied after launch)")
            return
        time.sleep(3)


WILDLIFE_COMMANDS = {"start herd", "start tusker", "start leopard", "reset"}


@app.post("/api/wildlife/scenario")
def wildlife_scenario():
    """Forest world: start the scripted herd / tusker walk, or reset them
    (animal_mover.py listens on the gz topic /wildlife/scenario)."""
    command = str((request.get_json(silent=True) or {}).get("command", "")).lower().strip()
    if command not in WILDLIFE_COMMANDS:
        return _json_err(f"command must be one of: {', '.join(sorted(WILDLIFE_COMMANDS))}")
    if current_world not in ANIMAL_WORLDS or not procs.is_running("animals"):
        return _json_err("no scripted animals running (launch the forest world first)")
    result = subprocess.run(
        [ROS_ENV, "gz", "topic", "-t", "/wildlife/scenario", "-m", "gz.msgs.StringMsg", "-p", f'data: "{command}"'],
        capture_output=True, text=True, timeout=10,
    )
    if result.returncode != 0:
        return _json_err(f"scenario command not sent: {(result.stdout + result.stderr).strip()[:200]}")
    log_hub.push("wildlife", f"scenario: {command}")
    return _json_ok(f"scenario: {command}")


@app.get("/video_feed/tower/<tid>")
def tower_feed(tid):
    """Sentinel's annotated feed from one camera (t1, c1, ...)."""
    if tid not in TOWER_IDS:
        return _json_err(f"unknown camera '{tid}'")
    return _mjpeg(lambda: ros_bridge.get_tower_frame(tid), viewer=tid)


@app.get("/api/wildlife/outbox")
def wildlife_outbox():
    """Alerts sent (or, without a Telegram token, that would have been sent)."""
    return notifier.snapshot()


@app.get("/api/wildlife/snapshot/<name>")
def wildlife_snapshot(name):
    return send_from_directory(SNAPSHOT_DIR, os.path.basename(name))


@app.get("/cams")
def cams_page():
    """Camera wall: the drone's forward camera and the three towers, live."""
    return send_from_directory(FRONTEND_DIR, "cams.html")


@app.get("/community")
def community_page():
    """Villagers' page: safety tips and reporting a sighting (phone-sized)."""
    return send_from_directory(FRONTEND_DIR, "community.html")


REPORT_SPECIES = ("elephant", "leopard", "bear", "wild boar", "other")
EARTH_RADIUS_M = 6371000.0


def _report_position(body):
    """(e, n) from the body's e/n, or lat/lon via the map origin."""
    if body.get("e") not in (None, "") and body.get("n") not in (None, ""):
        return float(body["e"]), float(body["n"])
    if body.get("lat") not in (None, "") and body.get("lon") not in (None, ""):
        latest = ros_bridge.get_map()
        if not latest:
            raise ValueError("no map origin yet to place a GPS position")
        lat0, lon0 = json.loads(latest)["origin"]
        n = math.radians(float(body["lat"]) - lat0) * EARTH_RADIUS_M
        e = math.radians(float(body["lon"]) - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
        return e, n
    raise ValueError("a position is needed: tap the map, or allow location")


def _sighting_event(body, source):
    species = str(body.get("species", "elephant")).lower()
    if species not in REPORT_SPECIES:
        raise ValueError(f"species must be one of: {', '.join(REPORT_SPECIES)}")
    e, n = _report_position(body)
    count = max(1, min(50, int(body.get("count") or 1)))
    return {"type": "report", "source": source, "species": species, "count": count, "e": round(e, 1),
            "n": round(n, 1), "alert": species == "elephant", "note": str(body.get("note", ""))[:200],
            "source_label": str(body.get("source_label") or ("Villager report" if source == "community"
                                                              else "Imported alert"))[:60],
            "t": float(body.get("t") or time.time())}


@app.post("/api/community/report")
def community_report():
    """A villager's sighting: onto the same event stream as the towers."""
    try:
        event = _sighting_event(request.get_json(silent=True) or {}, "community")
    except (ValueError, TypeError) as exc:
        return _json_err(str(exc))
    ros_bridge.publish_sentinel_event(event)
    log_hub.push("community", f"report: {event['count']} {event['species']} at ({event['e']}, {event['n']}) "
                              f"{event['note']}")
    return _json_ok("Thank you - the forest team has your report.", event=event)


@app.post("/api/events/ingest")
def events_ingest():
    """Alerts from systems already in the field (TrailGuard-style camera
    alerts, camera-trap batches; see tools/ingest_*.py): a list of
    {species, count, e/n or lat/lon, t, source_label, note}."""
    body = request.get_json(silent=True)
    items = body if isinstance(body, list) else [body or {}]
    accepted = []
    for item in items:
        try:
            event = _sighting_event(item, "ingest")
        except (ValueError, TypeError) as exc:
            return _json_err(f"item {len(accepted)}: {exc}", accepted=len(accepted))
        ros_bridge.publish_sentinel_event(event)
        accepted.append(event)
    log_hub.push("ingest", f"{len(accepted)} external alert(s) imported")
    return _json_ok(f"{len(accepted)} alert(s) imported.", accepted=len(accepted))


@app.post("/api/mode")
def set_mode():
    mode = str((request.get_json(silent=True) or {}).get("mode", "")).lower()
    if mode not in LIGHTING:
        return _json_err("mode must be 'day' or 'night'")
    ros_bridge.set_mode(mode)
    log_hub.push("mode", f"sensor mode -> {mode} ({'thermal' if mode == 'night' else 'RGB'})")
    if REAL:
        return _json_ok(f"Sensor mode set to {mode} (real drone: no simulated lighting).", mode=mode)
    if not procs.is_running("sim"):
        return _json_ok(f"Mode set to {mode}; lighting will apply when the sim launches.", mode=mode)
    ok, msg = apply_lighting(mode)
    log_hub.push("mode", msg)
    if not ok:
        return _json_err(f"Sensor switched to {mode}, but {msg}", mode=mode)
    return _json_ok(f"Switched to {mode} mode.", mode=mode)


@app.post("/api/payload/drop")
def payload_drop():
    if not procs.is_running("sim"):
        return _json_err("sim is not running - nothing to drop.")
    if REAL:
        # The bridge owns the MAVLink link: it moves the release servo.
        ok, msg = ros_bridge.call_payload_drop()
        if ok:
            ros_bridge.set_payload_dropped(True)
            log_hub.push("payload", msg)
            return _json_ok(msg)
        return _json_err(msg)
    cmd = [
        ROS_ENV, "gz", "topic", "-t", "/model/s500_quad/payload/detach",
        "-m", "gz.msgs.Empty", "-p", "",
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, timeout=5, text=True)
    except subprocess.TimeoutExpired:
        return _json_err("gz topic call timed out (is the sim launched?)")
    if result.returncode != 0:
        return _json_err(f"drop failed: {result.stderr.strip() or result.stdout.strip()}")
    ros_bridge.set_payload_dropped(True)
    log_hub.push("payload", "release triggered")
    return _json_ok("Payload released.")


@app.post("/api/emergency_stop")
def emergency_stop():
    for name in ("heat", "animals", "bridge", "sim", "feed"):
        procs.stop(name)
    procs.kill_all_fallback()
    ros_bridge.clear_frame()
    return _json_ok("Emergency stop complete.")


@app.get("/api/status")
def status_stream():
    def generate():
        while True:
            payload = ros_bridge.get_telemetry()
            payload["sim_running"] = procs.is_running("sim")
            payload["bridge_running"] = procs.is_running("bridge")
            payload["feed_running"] = procs.is_running("feed")
            payload["world"] = current_world
            payload["real"] = REAL
            if not payload["feed_running"]:
                # Last values from a stopped detector are stale, not live.
                payload["persons_detected"] = 0
                payload["person_confirmed"] = False
            yield f"data: {json.dumps(payload)}\n\n"
            time.sleep(0.5)

    return Response(generate(), mimetype="text/event-stream")


@app.get("/api/map")
def map_stream():
    """The bridge's situational map (probability grid, trail, casualties,
    hazards, routes, alerts), relayed at 2 Hz."""
    def generate():
        last = None
        while True:
            data = ros_bridge.get_map()
            if data is not None and data is not last:
                yield f"data: {data}\n\n"
                last = data
            else:
                yield ": keepalive\n\n"
            time.sleep(0.5)

    return Response(generate(), mimetype="text/event-stream")


@app.get("/api/map/latest")
def map_latest():
    data = ros_bridge.get_map()
    if data is None:
        return _json_err("no map yet (is the mission bridge running?)")
    return Response(data, mimetype="application/json")


@app.get("/report")
def report():
    """Printable situation report (SITREP) of the latest mission map."""
    return send_from_directory(FRONTEND_DIR, "report.html")


@app.get("/api/logs")
def logs_stream():
    return Response(log_hub.stream(), mimetype="text/event-stream")


MJPEG_MAX_FPS = 30.0


def _mjpeg(get_frame, viewer=None):
    """Stream frames as they arrive (up to MJPEG_MAX_FPS), not on a fixed
    timer, so the browser sees the camera's full rate; a placeholder is
    re-sent every second while there is no feed. viewer: the camera this
    stream shows, counted while the client is connected (see watch())."""
    def generate():
        boundary = b"--frame"
        placeholder = _placeholder_jpeg()
        last_frame = None
        if viewer:
            ros_bridge.watch(viewer, +1)
        try:
            while True:
                with ros_bridge.frame_cond:
                    ros_bridge.frame_cond.wait(timeout=1.0)
                frame = get_frame()
                if frame is not None and frame is last_frame:
                    continue
                last_frame = frame
                yield (
                    boundary + b"\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + (frame or placeholder) + b"\r\n"
                )
                time.sleep(1.0 / MJPEG_MAX_FPS)
        finally:
            if viewer:
                ros_bridge.watch(viewer, -1)

    return Response(
        generate(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.get("/video_feed")
def video_feed():
    """Detector output: the active camera with detection boxes and banners."""
    return _mjpeg(ros_bridge.get_frame, viewer="uav")


@app.get("/video_feed/thermal")
def thermal_feed():
    """Raw thermal camera in false colour, independent of day/night mode."""
    return _mjpeg(ros_bridge.get_thermal_frame)


def _placeholder_jpeg():
    import numpy as np

    img = np.zeros((360, 640, 3), dtype="uint8")
    cv2.putText(
        img, "Waiting for feed...", (140, 190),
        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (120, 120, 120), 2, cv2.LINE_AA,
    )
    ok, jpeg = cv2.imencode(".jpg", img)
    return jpeg.tobytes() if ok else b""


if __name__ == "__main__":
    start_ros_bridge()
    # RESQ_PORT lets a second dashboard (e.g. a test run) use another port.
    app.run(host="0.0.0.0", port=int(os.environ.get("RESQ_PORT", "5000")), threaded=True)
