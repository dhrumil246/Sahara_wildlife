#!/usr/bin/env python3
"""Sentinel: animal detection for the forest world (PS000183).

Watches the three camera towers and the drone's forward camera pair (all
RGB + thermal, boresighted), finds animals, puts each one on the ground in
metres and tells an elephant from cattle and dogs.

- Day: COCO YOLOv8m on RGB (the same model as person_detector.py's day
  mode; it already knows "elephant", "cow", "dog"...). Measured in the
  forest world: elephants 0.60 at ~40 m to 0.96 crossing in front of T2.
- Night: warm regions in the radiometric thermal frame (animals ~302-304 K
  on ~288 K ground), classified by their size on the ground.
- Every box is projected to the ground (flat ground, camera pose known:
  towers from the mission intel, the drone from its pose in the sim) and
  measured in metres. Size settles what the class name alone doesn't:
  at 25-30 m COCO calls a cow "dog" or "sheep" now and then; a 1.4 m
  tall animal is cattle whatever the label says.
- Only elephants raise an alert. Cattle and dogs are logged "no alert":
  the decoys a tower at the forest edge sees all day.

Inputs come straight from Gazebo transport (raw ROS Images cost ~93 ms per
frame in Python, see person_detector.py); outputs are ROS:
  /sentinel/detections         std_msgs/String JSON, every analysed frame:
                               {source, t, mode, objects: [{label, conf, e, n, height_m, length_m, alert}]}
  /sentinel/events             std_msgs/String JSON, sightings (cooldown per source and species):
                               {type: "sighting", source, species, count, e, n, conf, alert, snapshot, t}
  /sentinel/<source>/image/compressed   annotated JPEG per tower (t1, t2, t3)
  /detections/debug_image/compressed    the drone's annotated feed (the dashboard's AI feed)
Day/night follows /resq/sensor_mode, like person_detector.py.

    python3 ardu_ws/sentinel_detector.py     (the dashboard starts it for the forest world)
"""
import json
import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
import torch
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from gz.transport13 import SubscribeOptions
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MISSION_FILE = os.environ.get('RESQ_MISSION_FILE', os.path.join(ROOT, 'resq_mavlink', 'mission_forest.json'))
SNAPSHOT_DIR = os.path.join(ROOT, 'logs', 'sentinel')
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

MODEL = os.path.join(HERE, 'yolov8m.pt')
USE_FP16 = torch.cuda.is_available()
# COCO animals: 15 cat, 16 dog, 17 horse, 18 sheep, 19 cow, 20 elephant, 21 bear, 22 zebra, 23 giraffe
ANIMAL_CLASSES = [15, 16, 17, 18, 19, 20, 21, 22, 23]  # 15 "cat": COCO has no leopard, it scores as cat
CONFIDENCE = 0.35
# Analyses per second per source. 3 towers x 2 + drone 4 = 10/s at FP16 on
# the laptop GPU (~25 ms each at 1280 px), leaving room for Gazebo.
TOWER_HZ = 2.0
DRONE_HZ = 4.0

# Drone camera pair (s500_quad/model.sdf gimbal_link): forward, 45 deg down.
DRONE_MODEL = 's500_quad'
DRONE_CAM_OFFSET = (0.16, 0.0, -0.06)
DRONE_CAM_PITCH = 0.785
DRONE_HFOV = 1.2
TOWER_CAM_FORWARD = 0.25   # sensor sits 0.25 m out on the tower's head
TOWER_HEAD_UP = 0.15

# Size rules (metres on the ground). An adult Asian elephant stands
# 2.5-3 m and is 3.5-4 m long; a calf ~1.3 m; cattle 1.3-1.5 m tall,
# 2-2.5 m long; a village dog ~0.6 m.
ELEPHANT_MIN_HEIGHT_M = 1.9
ELEPHANT_MIN_HEIGHT_DAY_M = 1.7  # cattle measure 1.40-1.63 m from the towers
ELEPHANT_MIN_AREA_M2 = 3.5  # measured: elephants 4.4 (head-on) to 8.0 m2; one cow 1.0-2.1 m2
CATTLE_MIN_EXTENT_M = 1.1
CALF_NEAR_M = 10.0          # a smaller warm body this close to an elephant is part of the herd (night)
HERD_MEMORY_S = 8.0         # ... an elephant seen by the same camera this recently
DUPLICATE_M = 2.0           # two boxes this close on the ground are one animal
ALERT_STREAK = 2            # analyses in a row with an elephant before an alert goes out
UNWATCHED_FPS = 1.0
ALERT_SPECIES = ('elephant', 'leopard', 'bear')
# Leopard by size: COCO has no leopard and calls one "dog" at 18-28 m and
# "bear"/"cat" close up (measured on the sim's leopard). Leopard-sized =
# 0.45-1.15 m tall and >= 1.1 m long, seen from the side (towers look across
# paths); a village dog is under ~0.9 m long.
# A sloth bear is similar in size: a real deployment needs a camera-trap
# model trained on Indian species.
LEOPARD_HEIGHT_M = (0.45, 1.15)  # day: cattle measure 1.40-1.63 m from the towers
LEOPARD_NIGHT_MAX_HEIGHT_M = 1.0  # night: cattle measured 0.95-1.46 m...
LEOPARD_NIGHT_MAX_AREA_M2 = 1.1   # ...but 1.0-2.1 m2 of warm area
LEOPARD_MIN_LENGTH_M = 1.1
# Warm regions further away are watched but not named: at 53-67 m T2 sized
# the herd right (2.7-2.9 m tall); beyond ~80 m, half hidden in the trees,
# an elephant measured cattle-sized.
THERMAL_CLASSIFY_M = 75.0
ELEPHANT_LENGTH_M = 4.0     # per elephant, to count a merged warm region
THERMAL_DELTA_K = 7.0       # warm = this far above the ground's median
THERMAL_MIN_PX = 25
THERMAL_UNITS_PER_K = 100.0
THERMAL_WINDOW_K = (283.0, 313.0)
MAX_RANGE_M = 120.0

EVENT_COOLDOWN_S = {'elephant': 20.0, 'other': 60.0}
COLORS = {'elephant': (40, 40, 235), 'bear': (40, 120, 235), 'leopard': (40, 120, 235), 'cattle': (60, 190, 60), 'dog': (60, 190, 60),
          'animal': (200, 200, 60)}


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def quat_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class CameraPose:
    """Where a camera is and which way it looks, for pixel -> ground. Gazebo
    camera frame: x forward, y left, z up; image right = -y, image down = -z."""

    def __init__(self, position, rotation, hfov, width, height):
        self.p = np.asarray(position, float)
        self.R = rotation
        self.f = (width / 2.0) / math.tan(hfov / 2.0)
        self.cx, self.cy = width / 2.0, height / 2.0
        self.scale = 1.0  # set when the frame size differs from the nominal one

    def ray(self, u, v):
        return self.R @ np.array([1.0, -(u - self.cx) / self.f, -(v - self.cy) / self.f])

    def ground(self, u, v):
        """(east, north, slant range) where pixel (u, v) meets flat ground, or None."""
        d = self.ray(u, v)
        if d[2] >= -1e-6:
            return None
        t = -self.p[2] / d[2]
        g = self.p + t * d
        rng = float(t * np.linalg.norm(d))
        # Plain floats: numpy scalars leak np.bool_ into the JSON (a calf
        # flag broke every drone detection message once).
        return (float(g[0]), float(g[1]), rng) if rng < MAX_RANGE_M else None

    def measure(self, box):
        """Ground point under a box's bottom centre and the object's size:
        (e, n, height_m, length_m, range_m) or None."""
        x1, y1, x2, y2 = box
        g = self.ground((x1 + x2) / 2.0, y2)
        if g is None:
            return None
        e, n, rng = g
        return e, n, (y2 - y1) * rng / self.f, (x2 - x1) * rng / self.f, rng


def leopard_sized(height_m, length_m, area_m2=None):
    """Day: from the box. Night (area given): lower and smaller, since a
    warm region of a cow can measure short but never that small."""
    if area_m2 is not None:
        return (LEOPARD_HEIGHT_M[0] <= height_m <= LEOPARD_NIGHT_MAX_HEIGHT_M and length_m >= LEOPARD_MIN_LENGTH_M
                and area_m2 <= LEOPARD_NIGHT_MAX_AREA_M2)
    return LEOPARD_HEIGHT_M[0] <= height_m <= LEOPARD_HEIGHT_M[1] and length_m >= LEOPARD_MIN_LENGTH_M


def classify(coco_label, height_m, length_m):
    """Species from the class name and the measured size."""
    if coco_label == 'elephant' and height_m >= ELEPHANT_MIN_HEIGHT_DAY_M:
        # Size too: one frame of YOLO calling the cows at T1 "elephant" sent
        # the drone once. Calves (1.3 m) are named by herd memory instead.
        return 'elephant'
    if coco_label in ('cat', 'dog', 'bear') and leopard_sized(height_m, length_m):
        return 'leopard'
    if coco_label == 'bear':
        return 'bear'
    if coco_label == 'cat':
        return 'dog'  # a small cat-like animal: no alert
    return 'cattle' if max(height_m, length_m) >= CATTLE_MIN_EXTENT_M else 'dog'


def thermal_species(height_m, length_m, area_m2):
    """Night: species from the warm region's size. Tall AND big: two cattle
    side by side merge into one long region, and one behind the other into
    a tall one (both gave false elephants at T1 in testing), but neither
    reaches an elephant's warm area (4.4 m2 head-on to 8 m2 side-on)."""
    if height_m >= ELEPHANT_MIN_HEIGHT_M and area_m2 >= ELEPHANT_MIN_AREA_M2:
        return 'elephant'
    if leopard_sized(height_m, length_m, area_m2):
        return 'leopard'
    return 'cattle' if max(height_m, length_m) >= CATTLE_MIN_EXTENT_M else 'dog'


class Source:
    def __init__(self, sid, label, hz, rgb_topic, thermal_topic, out_topic):
        self.id, self.label, self.hz = sid, label, hz
        self.rgb_topic, self.thermal_topic, self.out_topic = rgb_topic, thermal_topic, out_topic
        self.rgb = None        # latest gz Image
        self.thermal = None
        self.pose = None       # CameraPose (fixed for towers)
        self.last_run = 0.0
        self.boxes = []        # (box, species, conf, alert) on the last analysed frame
        self.boxes_t = 0.0
        self.last_event = {}   # species -> wall time
        self.history = []      # (t, e, n) elephant centroids, for heading and speed
        self.recent_elephants = []  # (t, e, n) of every elephant seen lately
        self.streaks = {}      # species -> analyses in a row with it
        self.last_frame = 0.0  # last annotated frame published


class SentinelDetector(Node):
    def __init__(self):
        super().__init__('sentinel_detector')
        with open(MISSION_FILE) as f:
            mission = json.load(f)
        self.world = os.environ.get('RESQ_WORLD', 'forest')
        self.model = YOLO(MODEL)
        self.names = self.model.names
        self.mode = 'day'
        self.lock = threading.Lock()
        os.makedirs(SNAPSHOT_DIR, exist_ok=True)

        self.sources = []
        for t in mission['towers']:
            s = Source(t['id'], f"{t['id']} {t['label']}", TOWER_HZ, t['rgb']['topic'], t['thermal']['topic'],
                       f"/sentinel/{t['id'].lower()}/image/compressed")
            R = rot_z(t['yaw_rad']) @ rot_y(t['pitch_rad'])
            head = np.array([t['east_m'], t['north_m'], t['height_m'] + TOWER_HEAD_UP])
            s.pose = CameraPose(head + R @ np.array([TOWER_CAM_FORWARD, 0, 0]), R, t['hfov_rad'],
                                t['rgb']['width'], t['rgb']['height'])
            s.thermal_size = (t['thermal']['width'], t['thermal']['height'])
            s.hfov = t['hfov_rad']
            self.sources.append(s)
        self.drone = Source('UAV', 'Drone (forward 45 deg)', DRONE_HZ, '/s500/gimbal/image', '/s500/gimbal/thermal',
                            '/detections/debug_image/compressed')
        self.drone.hfov = DRONE_HFOV
        self.sources.append(self.drone)
        self.drone_pose = None  # (position, quaternion wxyz)

        self.det_pub = self.create_publisher(String, '/sentinel/detections', 10)
        self.event_pub = self.create_publisher(String, '/sentinel/events', 10)
        self.img_pubs = {s.id: self.create_publisher(CompressedImage, s.out_topic, 10) for s in self.sources}
        self.create_subscription(String, '/resq/sensor_mode', self.mode_cb, LATCHED)
        # Feeds someone is watching (the command center publishes it): full
        # rate for those, UNWATCHED_FPS for the rest (thumbnails, snapshots).
        self.viewing = None  # None = no list yet: publish everything
        self.create_subscription(String, '/sentinel/viewing', self.viewing_cb, LATCHED)

        self.gz = GzNode()
        self.subscribed = None
        self.subscribe_mode('day')
        opts = SubscribeOptions()
        opts.msgs_per_sec = 30  # dynamic_pose/info comes every physics step
        self.gz.subscribe(Pose_V, f'/world/{self.world}/dynamic_pose/info', self.pose_cb, opts)
        threading.Thread(target=self.loop, daemon=True).start()
        self.get_logger().info(f"Sentinel watching {', '.join(s.id for s in self.sources)} "
                               f"(model {os.path.basename(MODEL)}, FP16 {USE_FP16})")

    # ---------------------------------------------------------------- inputs

    def mode_cb(self, msg):
        mode = msg.data.strip().lower()
        if mode in ('day', 'night') and mode != self.mode:
            self.mode = mode
            self.subscribe_mode(mode)
            self.get_logger().info(f'sensor mode -> {mode}')

    def viewing_cb(self, msg):
        try:
            self.viewing = set(json.loads(msg.data))
        except ValueError:
            pass

    def subscribe_mode(self, mode):
        """Subscribe to the cameras this mode uses, and only those. Gazebo
        renders a camera only while something subscribes: listening to both
        the RGB and the thermal camera of all four sources doubled the
        rendering and the frames Python had to take in, for nothing."""
        kind = 'rgb' if mode == 'day' else 'thermal'
        if kind == self.subscribed:
            return
        for src in self.sources:
            if self.subscribed is not None:
                self.gz.unsubscribe(src.rgb_topic if self.subscribed == 'rgb' else src.thermal_topic)
            topic = src.rgb_topic if kind == 'rgb' else src.thermal_topic
            self.gz.subscribe(GzImage, topic, lambda m, s=src, k=kind: self.image_cb(s, k, m))
        self.subscribed = kind

    def image_cb(self, source, kind, msg):
        with self.lock:
            setattr(source, kind, (msg, time.time()))
        # Live feed at camera rate with the last analysed boxes on it, if
        # someone watches it; else UNWATCHED_FPS.
        if (kind == 'rgb') == (self.mode == 'day'):
            now = time.time()
            if self.viewing is not None and source.id.lower() not in self.viewing \
                    and now - source.last_frame < 1.0 / UNWATCHED_FPS:
                return
            source.last_frame = now
            try:
                self.publish_frame(source, msg, kind)
            except Exception as exc:  # noqa: BLE001 - never kill the gz callback thread
                self.get_logger().error(f'{source.id} frame: {exc}')

    def pose_cb(self, msg):
        for p in msg.pose:
            if p.name == DRONE_MODEL:
                self.drone_pose = ((p.position.x, p.position.y, p.position.z),
                                   (p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z))
                return

    def drone_camera(self, width, height):
        if self.drone_pose is None:
            return None
        pos, q = self.drone_pose
        Rb = quat_matrix(q)
        return CameraPose(np.array(pos) + Rb @ np.array(DRONE_CAM_OFFSET), Rb @ rot_y(DRONE_CAM_PITCH),
                          DRONE_HFOV, width, height)

    # -------------------------------------------------------------- analysis

    def loop(self):
        while rclpy.ok():
            now = time.time()
            due = [s for s in self.sources if now - s.last_run >= 1.0 / s.hz]
            if not due:
                time.sleep(0.02)
                continue
            s = min(due, key=lambda x: x.last_run)
            s.last_run = now
            try:
                self.analyse(s)
            except Exception as exc:  # noqa: BLE001 - keep watching on a bad frame
                self.get_logger().error(f'{s.id}: {exc}')

    def analyse(self, s):
        # One mode for the whole analysis: a day/night switch mid-way had the
        # snapshot decode an RGB frame as thermal.
        mode = self.mode
        with self.lock:
            frame = s.rgb if mode == 'day' else s.thermal
        if frame is None or time.time() - frame[1] > 2.0:
            return
        msg, captured = frame
        if s is self.drone:
            pose = self.drone_camera(msg.width, msg.height)
            if pose is None or pose.p[2] < 3.0:
                return  # on the pad: nothing to measure against
        else:
            pose = s.pose if mode == 'day' else CameraPose(s.pose.p, s.pose.R, s.hfov, msg.width, msg.height)
        objects = self.detect_rgb(msg, pose) if mode == 'day' else self.detect_thermal(msg, pose, s)
        now = time.time()
        # Herd memory across cameras: a smaller animal right where any camera
        # saw an elephant moments ago is part of the herd (the drone, looking
        # down through the trees, called a half-hidden elephant "cattle").
        # YOLO sometimes boxes one animal twice ("elephant" and "cow", or two
        # "elephant"s): a box on the spot of an elephant in the same frame is
        # that elephant, not another animal (best-scored box kept).
        ele_here = []
        kept = []
        for o in sorted(objects, key=lambda o: (o['label'] != 'elephant', -o['conf'])):
            dup = any(math.hypot(o['e'] - e, o['n'] - n) < DUPLICATE_M for e, n in ele_here)
            if o['label'] == 'elephant' and not dup:
                ele_here.append((o['e'], o['n']))
            if not dup:
                kept.append(o)
        objects = kept
        herd = [(e, n) for src in self.sources for (t, e, n) in src.recent_elephants if now - t < HERD_MEMORY_S]
        for o in objects:
            if o['label'] in ('cattle', 'dog', 'animal') and any(math.hypot(o['e'] - e, o['n'] - n) < CALF_NEAR_M for e, n in herd):
                o['calf'] = bool(o['height_m'] < ELEPHANT_MIN_HEIGHT_M)
                o['label'], o['alert'] = 'elephant', True
        s.recent_elephants = [r for r in s.recent_elephants if now - r[0] < HERD_MEMORY_S] + \
            [(now, o['e'], o['n']) for o in objects if o['label'] == 'elephant']
        for sp in ALERT_SPECIES:
            s.streaks[sp] = s.streaks.get(sp, 0) + 1 if any(o['label'] == sp for o in objects) else 0
        s.boxes, s.boxes_t = [(o['box'], o['label'], o['conf'], o['alert']) for o in objects], time.time()
        out = {'source': s.id, 't': round(captured, 3), 'mode': mode,
               'objects': [{k: v for k, v in o.items() if k != 'box'} for o in objects]}
        if s is self.drone:
            out['camera'] = [round(pose.p[0], 2), round(pose.p[1], 2), round(pose.p[2], 2)]
        self.det_pub.publish(String(data=json.dumps(out)))
        self.sightings(s, objects, msg, captured, mode)

    def detect_rgb(self, msg, pose):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)[..., ::-1]
        r = self.model.predict(np.ascontiguousarray(img), imgsz=msg.width, conf=CONFIDENCE, classes=ANIMAL_CLASSES,
                               verbose=False, quantize=16 if USE_FP16 else None)[0]
        objects = []
        for b in r.boxes:
            box = [float(v) for v in b.xyxy[0]]
            m = pose.measure(box)
            if m is None:
                continue
            e, n, h, ln, rng = m
            label = classify(self.names[int(b.cls)], h, ln)
            objects.append({'box': box, 'label': label, 'coco': self.names[int(b.cls)], 'conf': round(float(b.conf), 3),
                            'e': round(e, 2), 'n': round(n, 2), 'height_m': round(h, 2), 'length_m': round(ln, 2),
                            'range_m': round(rng, 1), 'alert': label in ALERT_SPECIES})
        return objects

    def detect_thermal(self, msg, pose, s):
        k = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).astype(np.float32) / THERMAL_UNITS_PER_K
        ground = k[msg.height // 2:, :]
        base = float(np.median(ground[ground > 270])) if np.any(ground > 270) else 288.0
        warm = (k > base + THERMAL_DELTA_K).astype(np.uint8)
        warm = cv2.morphologyEx(warm, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        count, _, stats, _ = cv2.connectedComponentsWithStats(warm)
        objects = []
        for i in range(1, count):
            x, y, w, h, area = stats[i]
            if area < THERMAL_MIN_PX:
                continue
            box = [float(x), float(y), float(x + w), float(y + h)]
            m = pose.measure(box)
            if m is None:
                continue
            e, n, hm, lm, rng = m
            # Too far to size reliably (a herd deep in the trees showed as a
            # cattle-sized patch): watched, but not named and no event.
            area_m2 = float(area) * (rng / pose.f) ** 2  # warm pixels -> square metres facing the camera
            label = thermal_species(hm, lm, area_m2) if rng <= THERMAL_CLASSIFY_M else 'animal'
            objects.append({'box': box, 'label': label, 'coco': None,
                            'conf': round(min(0.99, 0.5 + area / 4000.0), 3), 'e': round(e, 2), 'n': round(n, 2),
                            'height_m': round(hm, 2), 'length_m': round(lm, 2), 'range_m': round(rng, 1)})
        # A smaller warm body beside an elephant (now, or seen by this camera
        # a moment ago) is part of the herd: a calf, or an adult half hidden
        # by trees, of which only a small warm patch shows.
        herd = [(o['e'], o['n']) for o in objects if o['label'] == 'elephant'] + \
            [(e, n) for _, e, n in s.recent_elephants]
        for o in objects:
            if o['label'] != 'elephant' and any(math.hypot(o['e'] - e, o['n'] - n) < CALF_NEAR_M for e, n in herd):
                o['label'] = 'elephant'
                o['calf'] = bool(o['height_m'] < ELEPHANT_MIN_HEIGHT_M)
        for o in objects:
            o['alert'] = o['label'] in ALERT_SPECIES
            if o['alert']:
                # A herd walking close together is one warm region in thermal.
                o['count'] = max(1, round(o['length_m'] / ELEPHANT_LENGTH_M))
        return objects

    # ---------------------------------------------------------------- events

    def sightings(self, s, objects, msg, captured, mode):
        now = time.time()
        by_species = {}
        for o in objects:
            by_species.setdefault(o['label'], []).append(o)
        ele = by_species.get('elephant', [])
        if ele:
            ce = sum(o['e'] for o in ele) / len(ele)
            cn = sum(o['n'] for o in ele) / len(ele)
            s.history = [h for h in s.history if now - h[0] < 30.0] + [(now, ce, cn)]
        for species, objs in by_species.items():
            if species == 'animal':
                continue  # unclassified (too far): no event
            key = 'elephant' if species == 'elephant' else 'other'
            if now - s.last_event.get(species, 0.0) < EVENT_COOLDOWN_S[key]:
                continue
            if species in ALERT_SPECIES and s.streaks.get(species, 0) < ALERT_STREAK:
                continue  # one frame is not a sighting
            s.last_event[species] = now
            ce = sum(o['e'] for o in objs) / len(objs)
            cn = sum(o['n'] for o in objs) / len(objs)
            snap = os.path.join(SNAPSHOT_DIR, f'{s.id.lower()}_{species}_{int(now)}.jpg')
            cv2.imwrite(snap, self.annotate(s, msg, 'rgb' if mode == 'day' else 'thermal'))
            event = {'type': 'sighting', 'source': s.id, 'source_label': s.label, 'species': species,
                     'count': sum(o.get('count', 1) for o in objs), 'e': round(ce, 1), 'n': round(cn, 1),
                     'conf': max(o['conf'] for o in objs), 'alert': objs[0]['alert'], 'mode': mode,
                     'snapshot': snap, 't': round(captured, 3)}
            motion = self.motion(s.history) if species == 'elephant' else None
            if motion:
                event['heading_deg'], event['speed_ms'] = motion
            self.event_pub.publish(String(data=json.dumps(event)))
            what = 'ALERT' if event['alert'] else 'no alert'
            self.get_logger().info(f"{s.id}: {len(objs)} {species} at ({ce:.0f}, {cn:.0f}) - {what}")

    @staticmethod
    def motion(history):
        """(heading deg from north, clockwise; speed m/s) over the recent track, or None."""
        if len(history) < 3 or history[-1][0] - history[0][0] < 3.0:
            return None
        t = np.array([h[0] for h in history])
        t -= t[0]
        ve = np.polyfit(t, [h[1] for h in history], 1)[0]
        vn = np.polyfit(t, [h[2] for h in history], 1)[0]
        speed = math.hypot(ve, vn)
        if speed < 0.15:
            return None
        return round(math.degrees(math.atan2(ve, vn)) % 360, 0), round(speed, 2)

    # --------------------------------------------------------------- display

    def to_bgr(self, msg, kind, width=None):
        """(BGR image, scale from the camera's pixels). With width, shrink
        FIRST: converting a full 1280x720 frame and shrinking it afterwards
        cost most of the per-frame CPU of the live feeds."""
        if kind == 'rgb':
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
            scale = 1.0
            if width and msg.width > width:
                scale = width / msg.width
                img = cv2.resize(img, (width, round(msg.height * scale)), interpolation=cv2.INTER_AREA)
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR), scale
        k = np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).astype(np.float32) / THERMAL_UNITS_PER_K
        lo, hi = THERMAL_WINDOW_K
        grey = np.clip((k - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)  # colder than lo (sky) -> black
        return cv2.applyColorMap(grey, cv2.COLORMAP_INFERNO), 1.0

    def annotate(self, s, msg, kind, width=None):
        img, k = self.to_bgr(msg, kind, width)
        fs = 0.55 * img.shape[1] / 960  # text scale: same look on 960 px and 640 px frames
        bar = max(18, round(26 * img.shape[1] / 960))
        fresh = time.time() - s.boxes_t < 1.5
        alert = None
        for box, label, conf, is_alert in (s.boxes if fresh else []):
            x1, y1, x2, y2 = (round(v * k) for v in box)
            c = COLORS.get(label, COLORS['animal'])
            cv2.rectangle(img, (x1, y1), (x2, y2), c, 2)
            text = f"{label.upper()} {conf:.2f}" + ('' if is_alert else '  no alert')
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, fs, 1)
            cv2.rectangle(img, (x1, max(y1 - th - 8, 0)), (x1 + tw + 6, max(y1, th + 8)), c, -1)
            cv2.putText(img, text, (x1 + 3, max(y1 - 5, th + 3)), cv2.FONT_HERSHEY_SIMPLEX, fs, (255, 255, 255), 1,
                        cv2.LINE_AA)
            if is_alert and not alert:
                alert = label
        banner = f"{s.label} | {'RGB' if kind == 'rgb' else 'THERMAL'} | {time.strftime('%H:%M:%S')}"
        if alert:
            banner += f' | {alert.upper()}'
        cv2.rectangle(img, (0, 0), (img.shape[1], bar), (0, 0, 0), -1)
        cv2.putText(img, banner, (8, round(bar * 0.7)), cv2.FONT_HERSHEY_SIMPLEX, fs, (80, 80, 255) if alert else (230, 230, 230), 1,
                    cv2.LINE_AA)
        return img

    def publish_frame(self, s, msg, kind):
        # Dashboard tiles: 960 px wide is plenty, and cheaper to send.
        img = self.annotate(s, msg, kind, width=960)
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            out = CompressedImage(format='jpeg', data=jpg.tobytes())
            out.header.stamp = self.get_clock().now().to_msg()
            self.img_pubs[s.id].publish(out)


def main():
    rclpy.init()
    node = SentinelDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
