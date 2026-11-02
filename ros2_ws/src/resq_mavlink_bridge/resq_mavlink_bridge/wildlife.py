"""Human-wildlife conflict logic for the wildlife mission (PS000183).

HerdTracker - one elephant herd's position, count, heading and speed from
    every source that sees it (camera towers, the drone, villagers'
    reports), its predicted path, the time until it reaches the village and
    an alert level.
RiskMap - conflict risk per cell: a static layer from the map (where
    elephants leave the forest x what a visit would cost people; illustrative
    weights, see worldgen/generate_forest.py) plus a live layer, sightings
    that fade with time.
standoff_point - where the drone holds while it watches: behind the herd
    (or on the drone's side if it isn't moving), never above it.

Pure Python + numpy, no ROS: mavlink_bridge.py drives it, and it can be
tested on its own. Local frame: metres east (e) / north (n) of the launch
point, wall-clock times.
"""
import math
from collections import deque

import numpy as np

# Herd estimate
FIT_WINDOW_S = 20.0       # velocity from the sightings of the last 20 s
LOST_AFTER_S = 600.0      # no sighting for 10 min: no current herd
MIN_SPEED_MS = 0.15       # slower than this the herd is standing (feeding)
COUNT_WINDOW_S = 90.0     # herd size: the best look this recent
SAME_HERD_S = 20.0        # while the herd was seen this recently...
SAME_HERD_M = 60.0        # ...an elephant further than this from it is another group
SOURCE_WEIGHT = {'UAV': 3.0, 'community': 0.3, 'ingest': 0.3}  # towers 1.0

# Alert levels, by how soon the herd can reach the village
CRITICAL_ETA_S = 120.0
CRITICAL_DIST_M = 40.0
WARNING_ETA_S = 600.0
LEVELS = ('none', 'watch', 'warning', 'critical')
LEVEL_DOWN_S = 60.0
PREDICT_HORIZON_S = 900.0
PREDICT_STEP_S = 5.0

# Live risk layer
SIGHTING_SIGMA_M = 20.0
SIGHTING_HALF_LIFE_S = 1800.0


def point_in_polygon(x, y, poly):
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi:
            inside = not inside
        j = i
    return inside


def distance_to_polygon(x, y, poly):
    """0 inside, else the distance to the nearest edge."""
    if point_in_polygon(x, y, poly):
        return 0.0
    best = float('inf')
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        dx, dy = x1 - x0, y1 - y0
        f = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / (dx * dx + dy * dy + 1e-12)))
        best = min(best, math.hypot(x - x0 - f * dx, y - y0 - f * dy))
    return best


class HerdTracker:
    def __init__(self, village, fields=(), forest_edge_n=None, critical_eta_s=CRITICAL_ETA_S,
                 warning_eta_s=WARNING_ETA_S, critical_dist_m=CRITICAL_DIST_M):
        """forest_edge_n: north of this the herd is still in the forest (level
        'watch' at most). The thresholds come from the mission's 'escalation'
        block: the 200 m demo world needs tighter ones than real distances."""
        self.village = [tuple(p) for p in village]
        self.fields = [[tuple(p) for p in f] for f in fields]
        self.forest_edge_n = forest_edge_n
        self.critical_eta_s, self.warning_eta_s, self.critical_dist_m = critical_eta_s, warning_eta_s, critical_dist_m
        self.obs = deque(maxlen=600)          # (t, e, n, source, count)
        self.track = deque(maxlen=400)        # [e, n] for the map, one per 2 m moved
        self.sources = {}                     # source -> last time it saw the herd
        self.counts = deque(maxlen=60)        # (t, count, source)
        self.level = 'none'
        self.calm_since = None
        self.calf = False

    # -- input

    def observe(self, t, elephants, source):
        """Elephant objects from one analysed frame: [{'e', 'n', 'count'?, 'calf'?}]."""
        if not elephants:
            return
        e = sum(o['e'] for o in elephants) / len(elephants)
        n = sum(o['n'] for o in elephants) / len(elephants)
        count = sum(int(o.get('count', 1)) for o in elephants)
        if self.add(t, e, n, source, count):
            self.calf = self.calf or any(o.get('calf') for o in elephants)

    def add(self, t, e, n, source, count=1):
        # One herd at a time: while it is being followed, an elephant far
        # from it (the tusker at another tower, an imported alert elsewhere)
        # is a separate group. It still alerts and marks the risk map, but
        # must not drag this herd's position across the map.
        if self.obs and t - self.obs[-1][0] < SAME_HERD_S and math.hypot(e - self.obs[-1][1], n - self.obs[-1][2]) > SAME_HERD_M:
            return False
        self.obs.append((t, e, n, source, count))
        self.counts.append((t, count, source))
        self.sources[source] = t
        if not self.track or math.hypot(e - self.track[-1][0], n - self.track[-1][1]) >= 2.0:
            self.track.append([round(e, 1), round(n, 1)])
        return True

    # -- estimate

    def last_seen(self):
        return self.obs[-1][0] if self.obs else None

    def active(self, now):
        return bool(self.obs) and now - self.obs[-1][0] < LOST_AFTER_S

    def estimate(self, now):
        """{'e', 'n', 've', 'vn', 'speed', 'heading_deg', 'count', 'age_s', 'source'} or None.
        Position: the latest sightings (weighted by source); velocity: a
        weighted straight-line fit over FIT_WINDOW_S."""
        if not self.active(now):
            return None
        t_last = self.obs[-1][0]
        recent = [o for o in self.obs if t_last - o[0] <= FIT_WINDOW_S]
        w = np.array([SOURCE_WEIGHT.get(o[3], 1.0) for o in recent])
        t = np.array([o[0] for o in recent]) - t_last
        es = np.array([o[1] for o in recent])
        ns = np.array([o[2] for o in recent])
        ve = vn = 0.0
        if len(recent) >= 4 and t.max() - t.min() >= 4.0:
            ve = float(np.polyfit(t, es, 1, w=np.sqrt(w))[0])
            vn = float(np.polyfit(t, ns, 1, w=np.sqrt(w))[0])
        latest = [o for o in recent if t_last - o[0] <= 2.0]
        lw = np.array([SOURCE_WEIGHT.get(o[3], 1.0) for o in latest])
        e = float(np.average([o[1] for o in latest], weights=lw))
        n = float(np.average([o[2] for o in latest], weights=lw))
        speed = math.hypot(ve, vn)
        if speed < MIN_SPEED_MS:
            ve = vn = speed = 0.0
        # Count: the best recent look from any camera (a single view often
        # shows only part of the herd: the drone first saw 1 of the 3 that
        # T2 had counted a minute before).
        recent_counts = sorted(c for (tc, c, _s) in self.counts if t_last - tc <= COUNT_WINDOW_S)
        # A high look, not the single highest (one frame with a doubled box
        # made a herd of 3 read 6).
        count = recent_counts[int(0.8 * (len(recent_counts) - 1))]
        return {'e': round(e, 1), 'n': round(n, 1), 've': round(ve, 2), 'vn': round(vn, 2),
                'speed': round(speed, 2), 'heading_deg': round(math.degrees(math.atan2(ve, vn)) % 360) if speed else None,
                'count': int(count), 'calf': self.calf, 'age_s': round(now - t_last, 1), 'source': self.obs[-1][3]}

    def predict(self, est, dt):
        """Dead reckoning dt seconds ahead of the last sighting."""
        t = dt + est['age_s']
        return est['e'] + est['ve'] * t, est['n'] + est['vn'] * t

    def predicted_path(self, est, horizon=180.0, step=10.0):
        if not est or not est['speed']:
            return []
        return [[round(c, 1) for c in self.predict(est, k * step)] for k in range(int(horizon / step) + 1)]

    def forest_exit(self, est, horizon=300.0):
        """Where the herd's path crosses the forest edge (going out), or None."""
        if self.forest_edge_n is None or not est['speed'] or est['vn'] >= 0:
            return None
        k = 0.0
        while k <= horizon:
            e, n = self.predict(est, k)
            if n <= self.forest_edge_n:
                return e, n
            k += 2.0
        return None

    def in_fields(self, e, n):
        return any(point_in_polygon(e, n, f) for f in self.fields)

    def village_eta(self, est):
        """Seconds until the herd reaches the village on its current course
        (0 inside it), or None if it isn't heading there."""
        if distance_to_polygon(est['e'], est['n'], self.village) == 0.0:
            return 0.0
        if not est['speed']:
            return None
        k = PREDICT_STEP_S
        while k <= PREDICT_HORIZON_S:
            if point_in_polygon(*self.predict(est, k), self.village):
                return k - est['age_s']
            k += PREDICT_STEP_S
        return None

    def assess(self, now):
        """The whole picture for the map and the alerts."""
        est = self.estimate(now)
        if est is None:
            self.level = 'none'
            return None
        dist = distance_to_polygon(est['e'], est['n'], self.village)
        eta = self.village_eta(est)
        crops = self.in_fields(est['e'], est['n'])
        in_forest = self.forest_edge_n is not None and est['n'] > self.forest_edge_n
        if in_forest:
            level = 'watch'
        elif dist <= self.critical_dist_m or (eta is not None and eta <= self.critical_eta_s):
            level = 'critical'
        elif crops or dist <= 2 * self.critical_dist_m or (eta is not None and eta <= self.warning_eta_s):
            level = 'warning'
        else:
            level = 'watch'
        # Up at once, down only after LEVEL_DOWN_S of calm: a herd feeding in
        # the open has no speed, so no ETA, and the level flapped
        # watch <-> warning every few seconds in testing.
        rank = LEVELS.index
        if rank(level) >= rank(self.level) or self.level == 'none':
            self.level, self.calm_since = level, None
        elif self.calm_since is None:
            self.calm_since = now
        elif now - self.calm_since >= LEVEL_DOWN_S:
            self.level, self.calm_since = level, None
        level = self.level
        return dict(est, village_dist_m=round(dist, 1), eta_s=None if eta is None else round(max(eta, 0.0)),
                    in_crops=crops, in_forest=in_forest, level=level, predicted=self.predicted_path(est),
                    sources={s: round(now - t, 1) for s, t in self.sources.items()})


def standoff_point(herd, drone_e, drone_n, distance):
    """Where to watch from: `distance` behind a moving herd (it walks away
    from the drone, not into it), or on the drone's side of a standing one."""
    if herd.get('speed'):
        ue, un = herd['ve'] / herd['speed'], herd['vn'] / herd['speed']
    else:
        de, dn = drone_e - herd['e'], drone_n - herd['n']
        d = math.hypot(de, dn) or 1.0
        ue, un = -de / d, -dn / d
    return herd['e'] - ue * distance, herd['n'] - un * distance


def oblique_footprint(e, n, alt, yaw, pitch_down, hfov, width, height, max_range=150.0):
    """Ground quadrilateral seen by a forward camera pitched pitch_down below
    the horizon (drone assumed level). yaw: MAVLink (0 = north, clockwise).
    Far corners are clipped to max_range."""
    f = (width / 2.0) / math.tan(hfov / 2.0)
    out = []
    for u, v in ((0, height), (width, height), (width, 0), (0, 0)):
        # camera frame (forward, right, down), tilted down by pitch_down
        x, y, z = 1.0, (u - width / 2.0) / f, (v - height / 2.0) / f
        c, s = math.cos(pitch_down), math.sin(pitch_down)
        fwd, down = c * x - s * z, s * x + c * z
        down = max(down, 1e-3)
        t = min(alt / down, max_range / math.hypot(fwd, y))
        fwd, right = fwd * t, y * t
        out.append([round(e + fwd * math.sin(yaw) + right * math.cos(yaw), 1),
                    round(n + fwd * math.cos(yaw) - right * math.sin(yaw), 1)])
    return out


def bearing_to(e0, n0, e1, n1):
    """Yaw (rad, 0 = north, clockwise) from (e0, n0) toward (e1, n1), as MAVLink wants it."""
    return math.atan2(e1 - e0, n1 - n0)


class RiskMap:
    """Static conflict risk (worldgen's risk_forest.npy) + live sightings."""

    def __init__(self, grid, e0, n0, cell):
        self.static = np.asarray(grid, float)
        self.e0, self.n0, self.cell = e0, n0, cell
        ny, nx = self.static.shape
        c_e = e0 + (np.arange(nx) + 0.5) * cell
        c_n = n0 + (np.arange(ny) + 0.5) * cell
        self.ee, self.nn = np.meshgrid(c_e, c_n)
        self.sightings = deque(maxlen=200)  # (t, e, n, weight)

    def add(self, t, e, n, weight=1.0):
        # One bump per place and minute is enough (towers report every few seconds).
        for (t0, e0, n0, _) in self.sightings:
            if abs(t - t0) < 60 and math.hypot(e - e0, n - n0) < 10:
                return
        self.sightings.append((t, e, n, weight))

    def grid(self, now):
        live = np.zeros_like(self.static)
        for (t, e, n, w) in self.sightings:
            decay = 0.5 ** ((now - t) / SIGHTING_HALF_LIFE_S)
            live += w * decay * np.exp(-((self.ee - e) ** 2 + (self.nn - n) ** 2) / (2 * SIGHTING_SIGMA_M ** 2))
        return self.static + 0.6 * live

    def to_dict(self, now, factor=2):
        """Downsampled 0-100 grid for the dashboard (row 0 = south)."""
        g = self.grid(now)
        ny, nx = g.shape
        g = g[:ny - ny % factor, :nx - nx % factor].reshape(ny // factor, factor, nx // factor, factor).mean(axis=(1, 3))
        top = max(float(g.max()), 1e-6)
        return {'e0': self.e0, 'n0': self.n0, 'cell': self.cell * factor, 'nx': g.shape[1], 'ny': g.shape[0],
                'max': round(top, 3), 'v': [int(v) for v in np.round(g / top * 100).ravel()]}


# ---------------------------------------------------------------------------
# Paths: chains of cameras toward the village
# ---------------------------------------------------------------------------

VISIT_GAP_S = 20.0        # sightings at one camera this close together are one visit
PATH_MEMORY_S = 900.0     # forget a path's story after 15 min of nothing
DEFAULT_SPEED_MS = {'elephant': 1.0, 'leopard': 1.2, 'bear': 0.8}
OVERDUE_FACTOR = 2.0      # the next camera is overdue after 2x the expected walk...
OVERDUE_MIN_S = 45.0      # ...and at least this long
BACKTRACK_GAP_S = 10.0
MIN_VISIT_HITS = 2
# A sighting counts for a camera within this of the path spot it watches.
# Under half the closest spacing (C2-T2, 19 m): at 25 m T2 claimed the herd
# while it was still at C2 and the path "confirmed" a camera early. Every
# route passes within ~3 m of its cameras' spots.
ON_PATH_M = 10.0


class PathTracker:
    """What each camera chain says about an animal's path.

    Feed it every (time, camera, species) sighting. For each corridor and
    species it keeps the visits in time order (a visit = one camera seeing
    the animal, gaps under VISIT_GAP_S merged) and reads them as:

      spotted      seen at one camera only (one look is not a path)
      approaching  seen at cameras in order toward the village
      turned_back  seen again at a camera further from the village than the
                   furthest one it reached
      stopped      the next camera hasn't seen it OVERDUE_FACTOR times longer
                   than the walk should take: stopped, or left the path
      at_village   reached the last camera, at the village edge

    dispatch: approaching (or at_village) and reached the corridor's
    dispatch_at camera, after being seen further up the chain or first seen
    at it: the drone should go. A herd seen at C1 and C2 that turns back
    never sets it.
    """

    def __init__(self, corridors, cameras, village):
        """corridors: [{'id', 'name', 'cameras': [ids forest -> village], 'dispatch_at'}];
        cameras: {id: (e, n)} of where each camera looks."""
        self.corridors = []
        for c in corridors:
            pts = [cameras[cid] for cid in c['cameras']]
            along = [0.0]
            for (e0, n0), (e1, n1) in zip(pts, pts[1:]):
                along.append(along[-1] + math.hypot(e1 - e0, n1 - n0))
            self.corridors.append(dict(c, pts=pts, along=along,
                                       village_m=distance_to_polygon(pts[-1][0], pts[-1][1], village),
                                       dispatch_idx=c['cameras'].index(c['dispatch_at'])))
        self.visits = {}   # (corridor id, species) -> [[cam idx, t_first, t_last]]

    def observe(self, t, camera, species, e=None, n=None):
        """A sighting by a camera; with (e, n), only if the animal is on the
        stretch of path that camera watches (T5, looking west across the
        fields, saw the herd 80 m beyond its own path)."""
        for c in self.corridors:
            if camera not in c['cameras']:
                continue
            idx = c['cameras'].index(camera)
            if e is not None and math.hypot(e - c['pts'][idx][0], n - c['pts'][idx][1]) > ON_PATH_M:
                continue
            v = self.visits.setdefault((c['id'], species), [])
            # Merge with this camera's own last visit even if a neighbour saw
            # the animal in between (overlapping views alternate otherwise).
            mine = next((x for x in reversed(v) if x[0] == idx), None)
            if mine is not None and t - mine[2] < VISIT_GAP_S:
                mine[2] = max(mine[2], t)
                mine[3] += 1
            else:
                v.append([idx, t, t, 1])
                v.sort(key=lambda x: x[1])

    def on_path(self, camera):
        return any(camera in c['cameras'] for c in self.corridors)

    def assess(self, now):
        out = []
        for c in self.corridors:
            for (cid, species), v in list(self.visits.items()):
                if cid != c['id']:
                    continue
                if not v or now - v[-1][2] > PATH_MEMORY_S:
                    del self.visits[(cid, species)]
                    continue
                r = self._read(c, species, v, now)
                if r is not None:
                    out.append(r)
        return out

    def _read(self, c, species, v, now):
        cams, along = c['cameras'], c['along']
        # A visit counts once two analyses agree (one frame of YOLO calling
        # cattle "elephant" at the launch camera sent the drone once).
        v = [x for x in v if x[3] >= MIN_VISIT_HITS]
        if not v:
            return None
        furthest = max(x[0] for x in v)
        reach = max((x for x in v if x[0] == furthest), key=lambda x: x[2])  # last visit at the furthest camera
        last = v[-1]
        # Speed between the first sightings at successive cameras toward the
        # village (first to first: views nearly touch, so "left one view" to
        # "entered the next" made a 1.1 m/s herd look like 3 m/s).
        speed = None
        for a, b in zip(v, v[1:]):
            if b[0] > a[0] and b[1] > a[1]:
                speed = (along[b[0]] - along[a[0]]) / max(b[1] - a[1], 1.0)
                break
        sp = speed or DEFAULT_SPEED_MS.get(species, 1.0)
        # The cameras of this pass: from the last visit at the furthest-back
        # camera on (a tusker that came and went earlier on the same path
        # made the herd's message read C1 -> C2 -> C1 -> C2 -> T2).
        lowest = min(x[0] for x in v)
        start = max(i for i, x in enumerate(v) if x[0] == lowest)
        if any(x[0] < furthest and x[1] > reach[1] for x in v[start:]):
            start = 0  # turning back: show the whole way there and back
        chain = []
        for x in v[start:]:
            if not chain or chain[-1] != cams[x[0]]:
                chain.append(cams[x[0]])
        nxt = cams[furthest + 1] if furthest + 1 < len(cams) else None
        expected = None
        if nxt is not None:
            walk = (along[furthest + 1] - along[furthest]) / sp
            expected = reach[2] + walk
        # Seen again further up the path, a while after last being seen at
        # the furthest camera (not just overlapping views as it walks on).
        backtracked = any(x[0] < furthest and x[1] > reach[1] and x[2] > reach[2] + BACKTRACK_GAP_S for x in v)
        moved_in = any(b[0] > a[0] for a, b in zip(v, v[1:]))
        # Once the path is confirmed the drone is out there watching the real
        # animals: a sighting further back is another group, not a reason to
        # tell the village "turned back" (a tusker walking behind the herd
        # did exactly that in testing).
        if backtracked and furthest < c['dispatch_idx']:
            state = 'turned_back'
        elif nxt is None:
            state = 'at_village'
        elif expected is not None and now - reach[2] > max(OVERDUE_MIN_S, OVERDUE_FACTOR * (expected - reach[2])):
            state = 'stopped'
        elif moved_in or furthest >= c['dispatch_idx']:
            state = 'approaching'
        else:
            state = 'spotted'
        remaining = along[-1] - along[furthest] + c['village_m']
        eta = max(0.0, remaining / sp - (now - reach[2])) if state in ('approaching', 'at_village') else None
        dispatch = state in ('approaching', 'at_village') and furthest >= c['dispatch_idx']
        return {
            'corridor': c['id'], 'name': c['name'], 'species': species, 'state': state, 'chain': chain,
            'cameras': [{'id': cid, 'seen': any(x[0] == i for x in v),
                         'last_s': round(now - max(x[2] for x in v if x[0] == i), 1) if any(x[0] == i for x in v) else None}
                        for i, cid in enumerate(cams)],
            'furthest': cams[furthest], 'last_seen_at': cams[last[0]], 'last_seen_s': round(now - last[2], 1),
            'next': nxt if state == 'approaching' else None,
            'next_in_s': round(max(0.0, expected - now)) if state == 'approaching' and expected else None,
            'speed_ms': round(speed, 2) if speed else None, 'eta_village_s': round(eta) if eta is not None else None,
            'dispatch': dispatch, 'dispatch_at': c['dispatch_at'],
            'look': list(c['pts'][furthest]),
        }
