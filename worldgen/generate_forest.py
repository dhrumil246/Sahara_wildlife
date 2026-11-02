#!/usr/bin/env python3
"""Generate the 200 x 200 m "forest" world: forest edge, elephant corridor,
crop fields and a village, watched by three camera towers (PS000183).

    python3 worldgen/generate_forest.py              # scenery (if missing) + world
    python3 worldgen/generate_forest.py --scenery    # force-rebuild the scenery too

Layout (x = east, y = north; the drone launches from the forest station at
the origin):
  north   y > 32       dense forest; a dirt elephant trail comes down it from
                       the north-west and leaves the trees at (-46, 30)
          y 14..32     forest edge: sparse trees, a waterhole in the west
          y 8..12      forest track (east-west)
  towers  y ~4         T1 (east, cattle grazing), T2 (corridor exit),
                       T3 (west, waterhole)
          y -15..-50   crop fields
  south   y < -58      village (the conflict zone), a lane and huts

Same construction as generate_region.py, whose helpers it reuses: one static
scenery model of merged meshes, no collision except the ground, generated
textures. The animals are separate models (CC BY, see
models/ANIMALS_ATTRIBUTION.md) moved by worldgen/animal_mover.py along the
scripted routes written here. The scenario is scripted, not seeded: the
same herd walks the same corridor every run, so the demo is repeatable.

Outputs:
  ardu_ws/src/ardupilot_gazebo/models/forest_scenery/   static scenery model
  ardu_ws/src/ardupilot_gz/ardupilot_gz_gazebo/worlds/forest.sdf (+ install copy)
  resq_mavlink/mission_forest.json    intel: towers, village, fields, water (no animal positions)
  resq_mavlink/risk_forest.npy        static conflict-risk layer (illustrative weights)
  resq_mavlink/forest_animals.json    animal routes, for animal_mover.py
  resq_mavlink/forest_truth.json      the same routes, for scoring test runs only
  command_center/frontend/basemaps/forest.png
"""
import argparse
import json
import math
import os
import random
import shutil

import cv2
import numpy as np

from generate_region import (HOME_LAT, HOME_LON, MODELS, ROOT, WORLDS_INSTALL, WORLDS_SRC, Mesh, in_poly,
                             make_textures, write_mtl)

SCENERY = os.path.join(MODELS, 'forest_scenery')
MISSION = os.path.join(ROOT, 'resq_mavlink/mission_forest.json')
RISK = os.path.join(ROOT, 'resq_mavlink/risk_forest.npy')
ANIMALS = os.path.join(ROOT, 'resq_mavlink/forest_animals.json')
TRUTH = os.path.join(ROOT, 'resq_mavlink/forest_truth.json')
BASEMAP = os.path.join(ROOT, 'command_center/frontend/basemaps/forest.png')

HALF = 100.0
SCENERY_SEED = 11
FOREST_N = 32.0       # dense forest north of this
EDGE_N = 14.0         # sparse forest-edge band EDGE_N..FOREST_N

# The elephant trail's centreline, north-west to the corridor exit.
TRAIL = [(-72, 100), (-66, 80), (-56, 58), (-48, 40), (-46, 30)]
WATERHOLE = (-80.0, 22.0, 9.0, 6.0)  # centre, radii
VILLAGE = [(-75, -58), (45, -58), (45, -99), (-75, -99)]

# Camera towers: position, the ground point the camera centre looks at, mast height.
# Cameras. "tower": 6 m mast, 1280x720 at 10 Hz (the long views over open
# ground). "trail": a camera trap on a 3 m post beside a path under the
# canopy, 960x540 at 5 Hz (it looks 12 m across the path, and there are
# five of them to render). Cameras sit in chains along the animals' paths
# (CORRIDORS), so Sentinel can tell coming from going.
TOWERS = [
    {'id': 'T1', 'label': 'East edge (grazing land)', 'x': 55.0, 'y': 4.0, 'look': (55.0, 30.0), 'h': 6.0, 'kind': 'tower'},
    {'id': 'T2', 'label': 'Corridor exit', 'x': -44.0, 'y': 4.0, 'look': (-46.0, 30.0), 'h': 6.0, 'kind': 'tower'},
    {'id': 'T3', 'label': 'West waterhole', 'x': -66.0, 'y': 4.0, 'look': (-80.0, 22.0), 'h': 6.0, 'kind': 'tower'},
    {'id': 'C1', 'label': 'Trail, deep forest', 'x': -50.0, 'y': 72.0, 'look': (-62.0, 70.0), 'h': 3.0, 'kind': 'trail'},
    {'id': 'C2', 'label': 'Trail, mid forest', 'x': -40.0, 'y': 50.0, 'look': (-51.5, 48.0), 'h': 3.0, 'kind': 'trail'},
    {'id': 'T4', 'label': 'Village edge (west)', 'x': -10.0, 'y': -50.0, 'look': (-24.0, -46.0), 'h': 4.0, 'kind': 'trail'},
    {'id': 'L1', 'label': 'East nullah, forest', 'x': 60.0, 'y': 66.0, 'look': (72.0, 64.0), 'h': 3.0, 'kind': 'trail'},
    {'id': 'T5', 'label': 'Village edge (east)', 'x': 58.0, 'y': -48.0, 'look': (42.0, -45.0), 'h': 4.0, 'kind': 'trail'},
]
# Paths animals take toward the village, as chains of cameras (forest ->
# village). The drone launches only when the animals reach dispatch_at after
# being seen further up the chain (or are first seen there or later): one
# sighting deep in the forest is not yet a visit to the village.
CORRIDORS = [
    {'id': 'trail', 'name': 'Elephant trail (north-west)', 'cameras': ['C1', 'C2', 'T2', 'T4'], 'dispatch_at': 'T2'},
    {'id': 'east', 'name': 'East nullah', 'cameras': ['L1', 'T1', 'T5'], 'dispatch_at': 'T1'},
]
CAMERA_KIND = {'tower': {'rgb': (1280, 720), 'hz': 10}, 'trail': {'rgb': (960, 540), 'hz': 5}}
TOWER_HFOV = 1.2           # rad, as the drone camera
TOWER_THERMAL = (640, 480)


def tower_aim(t):
    """Yaw (from +x) and downward pitch that put the camera centre on t['look']."""
    dx, dy = t['look'][0] - t['x'], t['look'][1] - t['y']
    return math.atan2(dy, dx), math.atan2(t['h'], math.hypot(dx, dy))


def seg_dist(px, py, pts):
    """Distance from (px, py) to a polyline."""
    best = 1e9
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        dx, dy = x1 - x0, y1 - y0
        f = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(px - x0 - f * dx, py - y0 - f * dy))
    return best


def waterhole_poly(n=20):
    cx, cy, rx, ry = WATERHOLE
    return [(cx + rx * math.cos(2 * math.pi * k / n), cy + ry * math.sin(2 * math.pi * k / n)) for k in range(n)]


# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------

def build_layout(rng):
    L = {'roads': [], 'huts': [], 'trees': [], 'fields': [], 'hero': [], 'water': [waterhole_poly()]}
    # Dirt roads: the forest track, the road from the station to the village, a village lane.
    L['roads'] += [(-100, 8, -12, 12), (12, 8, 100, 12), (-3, -100, 3, -12), (-75, -72, 45, -68)]
    # Crop fields between the forest edge and the village.
    for (x0, x1) in ((-92, -50), (-45, -8), (8, 46), (52, 92)):
        L['fields'].append((x0, -50, x1, -15))

    # Keep clear: the trail, the waterhole, each tower's near view and line of
    # sight to its aim point, the grazing land in front of T1, the station.
    def clear(px, py):
        if seg_dist(px, py, TRAIL) < 5.5 or math.hypot(px + 46, py - 26) < 10:
            return True
        for t in TOWERS:
            near, sight = (18, 7) if t['kind'] == 'tower' else (5, 3.5)
            if math.hypot(px - t['x'], py - t['y']) < near or seg_dist(px, py, [(t['x'], t['y']), t['look']]) < sight:
                return True
        cx, cy, rx, ry = WATERHOLE
        if ((px - cx) / (rx + 4)) ** 2 + ((py - cy) / (ry + 4)) ** 2 < 1:
            return True
        if 44 < px < 68 and py < 40:
            return True
        return abs(px) < 16 and abs(py) < 16
    y = EDGE_N
    while y < 98:
        x = -98
        while x < 98:
            px, py = x + rng.uniform(-2.5, 2.5), y + rng.uniform(-2.5, 2.5)
            dense = py > FOREST_N
            if not clear(px, py) and (dense or rng.random() < 0.22):
                kind = 'broad' if rng.random() < 0.7 else 'pine'  # mostly broadleaf (dry deciduous)
                L['trees'].append((px, py, rng.uniform(7.0, 13.0), kind))
            x += rng.uniform(5.5, 7.5)
        y += rng.uniform(5.5, 7.5)
    # A few shade trees along the field bunds.
    for _ in range(10):
        px, py = rng.uniform(-90, 90), rng.choice([-13.0, -52.0])
        if abs(px) > 6:
            L['trees'].append((px, py, rng.uniform(6.0, 9.0), 'broad'))

    # Village: three Indian houses and huts around the lane.
    for i, (hx, hy, yaw) in enumerate([(-40, -80, 0.3), (10, -84, -0.2), (-62, -88, 1.2)]):
        L['hero'].append({'name': f'indian_house_{i}', 'uri': 'hisuki/Indian House', 'x': hx, 'y': hy, 'yaw': yaw})
    for _ in range(22):
        for _ in range(40):
            hx, hy = rng.uniform(-70, 40), rng.uniform(-96, -60)
            ok = all(math.hypot(hx - h[0], hy - h[1]) > 8 for h in L['huts']) and \
                all(math.hypot(hx - h['x'], hy - h['y']) > 10 for h in L['hero']) and \
                abs(hy + 70) > 5 and abs(hx) > 6
            if ok:
                L['huts'].append((hx, hy, rng.uniform(4, 6), rng.uniform(3.5, 5), rng.uniform(0, math.pi), False))
                break
    # The forest station: office and the forest department jeep.
    L['hero'].append({'name': 'forest_office', 'uri': 'hisuki/House', 'x': 13.0, 'y': -6.0, 'yaw': 3.14})
    return L


# ---------------------------------------------------------------------------
# Scenery model
# ---------------------------------------------------------------------------

def build_scenery(L, rng):
    mesh_dir = os.path.join(SCENERY, 'meshes')
    if os.path.isdir(SCENERY):
        shutil.rmtree(SCENERY)
    os.makedirs(mesh_dir)
    make_textures(mesh_dir, rng)

    g = Mesh()
    G = 300.0
    g.poly('grass', [(-G, -G, 0), (G, -G, 0), (G, G, 0), (-G, G, 0)],
           [(-G / 6, -G / 6), (G / 6, -G / 6), (G / 6, G / 6), (-G / 6, G / 6)])
    # Forest floor: leaf litter (dirt texture) under the dense forest.
    g.poly('litter', [(-G, FOREST_N - 2, 0.01), (G, FOREST_N - 2, 0.01), (G, G, 0.01), (-G, G, 0.01)],
           [(-G / 6, 0), (G / 6, 0), (G / 6, G / 6), (-G / 6, G / 6)])
    for (x0, y0, x1, y1) in L['roads']:
        g.poly('dirt', [(x0, y0, 0.03), (x1, y0, 0.03), (x1, y1, 0.03), (x0, y1, 0.03)],
               [(x0 / 6, y0 / 6), (x1 / 6, y0 / 6), (x1 / 6, y1 / 6), (x0 / 6, y1 / 6)])
    # The elephant trail: a dirt strip along the centreline.
    for (x0, y0), (x1, y1) in zip(TRAIL, TRAIL[1:]):
        nx, ny = -(y1 - y0), x1 - x0
        k = 2.0 / math.hypot(nx, ny)
        nx, ny = nx * k, ny * k
        g.poly('dirt', [(x0 - nx, y0 - ny, 0.025), (x1 - nx, y1 - ny, 0.025), (x1 + nx, y1 + ny, 0.025), (x0 + nx, y0 + ny, 0.025)],
               [(0, 0), (1, 0), (1, 6), (0, 6)], outward=(x0, y0, -1))
    for (x0, y0, x1, y1) in L['fields']:
        g.poly('farmland', [(x0, y0, 0.02), (x1, y0, 0.02), (x1, y1, 0.02), (x0, y1, 0.02)],
               [(0, 0), ((x1 - x0) / 12, 0), ((x1 - x0) / 12, (y1 - y0) / 12), (0, (y1 - y0) / 12)])
    g.poly('concrete', [(-12, -12, 0.035), (12, -12, 0.035), (12, 12, 0.035), (-12, 12, 0.035)],
           [(0, 0), (2, 0), (2, 2), (0, 2)])
    g.write(os.path.join(mesh_dir, 'ground.obj'), 'ground.mtl')
    write_mtl(os.path.join(mesh_dir, 'ground.mtl'), {
        'grass': (1, 1, 1, 'grass.png'), 'litter': (0.75, 0.7, 0.6, 'dirt.png'), 'dirt': (1, 1, 1, 'dirt.png'),
        'farmland': (1, 1, 1, 'farmland.png'), 'concrete': (1, 1, 1, 'concrete.png')})

    b = Mesh()
    for (hx, hy, sx, sy, yaw, _) in L['huts']:
        b.box(hx, hy, 0, sx, sy, 2.6, yaw, 'mud', 'thatch', uv_m=(2.0, 2.0), top_uv=2.0)
        b.gable_roof(hx, hy, 2.6, sx, sy, 1.6, yaw, 'thatch')
    b.write(os.path.join(mesh_dir, 'buildings.obj'), 'buildings.mtl')
    write_mtl(os.path.join(mesh_dir, 'buildings.mtl'), {'mud': (1, 1, 1, 'mud.png'), 'thatch': (1, 1, 1, 'thatch.png')})

    t = Mesh()
    for (tx, ty, th, kind) in L['trees']:
        t.cylinder(tx, ty, 0, 0.25, th * 0.4, 'trunk')
        if kind == 'pine':
            t.cone(tx, ty, th * 0.25, th * 0.2, th * 0.75, 'pine')
        else:
            t.blob(tx, ty, th * 0.62, th * 0.3, th * 0.3, rng.choice(['leaf_a', 'leaf_b', 'leaf_c']), rng)
    t.write(os.path.join(mesh_dir, 'trees.obj'), 'trees.mtl')
    write_mtl(os.path.join(mesh_dir, 'trees.mtl'), {
        'trunk': (0.33, 0.24, 0.16), 'pine': (0.13, 0.30, 0.16),
        'leaf_a': (0.20, 0.38, 0.15), 'leaf_b': (0.28, 0.44, 0.18), 'leaf_c': (0.36, 0.42, 0.16)})

    stats = {'ground_tris': g.tri_count(), 'building_tris': b.tri_count(), 'tree_tris': t.tri_count(),
             'trees': len(L['trees']), 'huts': len(L['huts'])}
    visuals = [f'''      <visual name="{name}">
        <geometry><mesh><uri>model://forest_scenery/meshes/{name}.obj</uri></mesh></geometry>
      </visual>''' for name in ('ground', 'buildings', 'trees')]
    # Waterhole: thermally cold (281 K, as the region's river).
    w = Mesh()
    poly = L['water'][0]
    cx, cy = WATERHOLE[0], WATERHOLE[1]
    for k in range(len(poly)):
        p0, p1 = poly[k], poly[(k + 1) % len(poly)]
        w.poly('water', [(cx, cy, 0.05), (p0[0], p0[1], 0.05), (p1[0], p1[1], 0.05)], [(0, 0), (1, 0), (0, 1)],
               outward=(cx, cy, -1))
    w.write(os.path.join(mesh_dir, 'waterhole.obj'), 'water.mtl')
    write_mtl(os.path.join(mesh_dir, 'water.mtl'), {'water': (0.16, 0.20, 0.19)})
    visuals.append('''      <visual name="waterhole">
        <geometry><mesh><uri>model://forest_scenery/meshes/waterhole.obj</uri></mesh></geometry>
        <material><ambient>0.10 0.13 0.13 1</ambient><diffuse>0.16 0.20 0.19 1</diffuse><specular>0.7 0.7 0.7 1</specular>
          <pbr><metal><roughness>0.08</roughness><metalness>0.0</metalness></metal></pbr></material>
        <cast_shadows>false</cast_shadows>
        <plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal"><temperature>281</temperature></plugin>
      </visual>''')
    with open(os.path.join(SCENERY, 'model.sdf'), 'w') as f:
        f.write(f'''<?xml version="1.0"?>
<!-- Generated by worldgen/generate_forest.py - edit the generator, not this file.
     Static landscape of the forest world as a handful of merged meshes
     ({stats}). No collision except the ground plane. -->
<sdf version="1.9">
  <model name="forest_scenery">
    <static>true</static>
    <link name="link">
      <collision name="ground_collision">
        <geometry><plane><normal>0 0 1</normal><size>600 600</size></plane></geometry>
      </collision>
{chr(10).join(visuals)}
    </link>
  </model>
</sdf>
''')
    with open(os.path.join(SCENERY, 'model.config'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<model><name>forest_scenery</name><version>1.0</version>'
                '<sdf version="1.9">model.sdf</sdf><description>Generated landscape for the forest world.</description></model>\n')
    return stats


def build_basemap(L):
    """Top-down map for the dashboard (1 px = 0.25 m)."""
    px = 4
    S = int(2 * HALF * px)
    img = np.full((S, S, 3), (96, 132, 108), np.uint8)

    def P(x, y):
        return int((x + HALF) * px), int((HALF - y) * px)
    cv2.rectangle(img, P(-HALF, HALF), P(HALF, FOREST_N - 2), (62, 100, 70), -1)
    for (x0, y0, x1, y1) in L['fields']:
        cv2.rectangle(img, P(x0, y1), P(x1, y0), (92, 160, 170), -1)
    for (x0, y0, x1, y1) in L['roads']:
        cv2.rectangle(img, P(x0, y1), P(x1, y0), (100, 135, 160), -1)
    cv2.polylines(img, [np.array([P(*p) for p in TRAIL], np.int32)], False, (100, 135, 160), 4 * px)
    cv2.fillPoly(img, [np.array([P(*p) for p in L['water'][0]], np.int32)], (170, 120, 60))
    for (tx, ty, th, kind) in L['trees']:
        cv2.circle(img, P(tx, ty), int((2.0 if kind == 'pine' else 3.0) * px), (45, 92, 48), -1)
    cv2.rectangle(img, P(-12, 12), P(12, -12), (150, 150, 150), -1)
    for (hx, hy, sx, sy, yaw, _) in L['huts']:
        box = cv2.boxPoints((P(hx, hy), (sx * px, sy * px), -math.degrees(yaw)))
        cv2.fillPoly(img, [box.astype(np.int32)], (110, 150, 185))
    for h in L['hero']:
        cv2.circle(img, P(h['x'], h['y']), 5 * px, (190, 186, 180), -1)
    os.makedirs(os.path.dirname(BASEMAP), exist_ok=True)
    cv2.imwrite(BASEMAP, img)


RISK_CELL = 2.0
RISK_HALF = 96.0


def build_risk(L):
    """Static human-elephant conflict risk per 2 m cell, from the map only.

    risk = presence x stake. Presence: elephants leave the forest most near
    the corridor exit and the waterhole, and less with distance from the
    forest edge. Stake: what a visit costs people (crops 1.0, village 1.5,
    tracks 0.5, open ground 0.3). Weights are ILLUSTRATIVE (sample data);
    a real deployment fits them to the division's incident records. The
    live layer (sightings with time decay) is added on top at runtime.
    """
    n = int(2 * RISK_HALF / RISK_CELL)
    c = -RISK_HALF + (np.arange(n) + 0.5) * RISK_CELL
    ee, nn = np.meshgrid(c, c)  # row = north
    out = np.clip(FOREST_N - nn, 0, None)
    presence = np.exp(-out / 45.0)
    for (hx, hy, r, w) in ((-46, 30, 25, 1.5), (WATERHOLE[0], WATERHOLE[1], 20, 0.8)):
        presence += w * np.exp(-((ee - hx) ** 2 + (nn - hy) ** 2) / (2 * r * r))
    stake = np.full(ee.shape, 0.3)
    for (x0, y0, x1, y1) in L['roads']:
        stake[(ee >= x0) & (ee <= x1) & (nn >= y0) & (nn <= y1)] = 0.5
    for (x0, y0, x1, y1) in L['fields']:
        stake[(ee >= x0) & (ee <= x1) & (nn >= y0) & (nn <= y1)] = 1.0
    village = np.array([in_poly(x, y, VILLAGE) for x, y in zip(ee.ravel(), nn.ravel())]).reshape(ee.shape)
    stake[village] = 1.5
    risk = presence * stake
    risk[nn > FOREST_N] = 0.0  # inside the forest is the elephants' home, not a conflict
    risk /= risk.max()
    np.save(RISK, risk.astype(np.float32))
    return risk


def write_mission(L):
    towers = []
    for t in TOWERS:
        yaw, pitch = tower_aim(t)
        rgb = CAMERA_KIND[t['kind']]['rgb']
        towers.append({'id': t['id'], 'label': t['label'], 'kind': t['kind'], 'east_m': t['x'], 'north_m': t['y'], 'height_m': t['h'],
                       'look': list(t['look']),
                       'yaw_rad': round(yaw, 4), 'pitch_rad': round(pitch, 4), 'hfov_rad': TOWER_HFOV,
                       'rgb': {'topic': f'/tower/{t["id"].lower()}/image', 'width': rgb[0], 'height': rgb[1]},
                       'thermal': {'topic': f'/tower/{t["id"].lower()}/thermal', 'width': TOWER_THERMAL[0],
                                   'height': TOWER_THERMAL[1]}})
    mission = {
        'mission_id': 'HWC-FOREST',
        '_generated': 'worldgen/generate_forest.py - map intel only, no animal positions',
        'target': {'latitude': HOME_LAT, 'longitude': HOME_LON, 'altitude': 10},
        # The wildlife mission flies high and keeps its distance (observe only).
        'search': {'width_m': 192, 'height_m': 192, 'altitude_m': 30, 'speed_ms': 6.0,
                   'planner': 'bayes', 'on_confirm': 'continue', 'target_pos': 0.85, 'max_search_s': 900,
                   'mission_type': 'wildlife'},
        # The wildlife mission: launch on an elephant alert, intercept where
        # the herd will be, watch from behind at a distance, never descend.
        'wildlife': {'auto_dispatch': True, 'dispatch_rule': 'path', 'standoff_m': 25, 'intercept_speed_ms': 8.0, 'track_speed_ms': 6.0,
                     'max_track_s': 360, 'reacquire_s': 90, 'return_battery_pct': 30},
        # Alert levels. Tighter than real distances (a herd 120 m from the
        # village is 2 min away here); in the forest it is only ever 'watch'.
        'escalation': {'critical_eta_s': 45, 'critical_dist_m': 40, 'warning_eta_s': 600},
        '_local_frame': 'metres east/north of the launch point (forest station)',
        'staging': {'east_m': -8, 'north_m': -9, 'label': 'Forest station / rapid response team'},
        'prior_zones': [],
        'towers': towers,
        'corridors': CORRIDORS,
        'village': [[x, y] for x, y in VILLAGE],
        'fields': [[[x0, y0], [x1, y0], [x1, y1], [x0, y1]] for (x0, y0, x1, y1) in L['fields']],
        'forest_edge_north_m': FOREST_N,
        'corridor': [[x, y] for x, y in TRAIL],
        'known_water': [[[round(p[0], 1), round(p[1], 1)] for p in poly] for poly in L['water']],
        'known_hazards': [],
        # Trees top out at 13 m and the drone flies at 30 m; nothing to map.
        'known_obstacles': [],
        'risk_map': {'file': 'risk_forest.npy', 'e0': -RISK_HALF, 'n0': -RISK_HALF, 'cell_m': RISK_CELL,
                     '_note': 'static layer, illustrative weights (sample data)'},
        'basemap': {'url': 'basemaps/forest.png', 'e0': -HALF, 'n0': -HALF, 'w': 2 * HALF, 'h': 2 * HALF},
    }
    with open(MISSION, 'w') as f:
        json.dump(mission, f, indent=2)


# ---------------------------------------------------------------------------
# Animals (scripted scenario)
# ---------------------------------------------------------------------------

ELEPHANT_SPEED = 1.1   # m/s, an unhurried herd (they walk at ~4-6 km/h)
COW_SPEED = 0.35

# Route points (x, y, hold s). The herd waits hidden under the canopy until
# the scenario starts, walks the trail out of the forest, crosses ~13 m in
# front of T2, raids the paddy for a while, then heads for the village and
# stops at its edge.
HERD_ROUTE = [(-70, 92, 0), (-66, 80, 0), (-56, 58, 0), (-48, 40, 0), (-46, 30, 0), (-37, 19, 0), (-31, 8, 0),
              (-28, -8, 0), (-28, -24, 45), (-25, -40, 0), (-22, -55, 0)]
# A lone tusker, triggered separately: the "turns back" case. Comes down the
# trail past C1 and C2, stops, then turns round and walks back into the
# forest; it never reaches T2, so the drone must not launch.
TUSKER_ROUTE = [(-80, 99, 0), (-70, 90, 0), (-66, 80, 0), (-58, 62, 0), (-52, 49, 20), (-58, 62, 0), (-66, 80, 0),
                (-80, 99, 0)]
# A leopard, triggered separately: down the east nullah (L1), across the
# grazing land in front of T1 (sideways to the camera: head-on, 35 m from a
# 6 m mast, it was a 0.5 m wide blob nobody could name), across the fields
# to the east edge of the village (T5).
LEOPARD_SPEED = 1.3
LEOPARD_ROUTE = [(84, 98, 0), (76, 70, 0), (72, 46, 0), (66, 34, 0), (44, 30, 4), (38, 10, 0), (40, -30, 0),
                 (40, -55, 0)]
# Cattle graze in front of T1 all the time, slowly, with long pauses (decoys).
CATTLE = [('cow_holstein', [(50, 24, 20), (56, 30, 25), (61, 25, 15), (54, 20, 20)]),
          ('cow_dark', [(60, 33, 25), (52, 29, 15), (57, 36, 20)])]


def timed(route, speed):
    """[(x, y, hold)] -> waypoints [[t, x, y]] at constant speed, holding at each point."""
    out, t = [], 0.0
    for i, (x, y, hold) in enumerate(route):
        if i:
            px, py = route[i - 1][0], route[i - 1][1]
            t += math.hypot(x - px, y - py) / speed
        out.append([round(t, 2), x, y])
        if hold:
            t += hold
            out.append([round(t, 2), x, y])
    return out


def animals():
    herd = timed(HERD_ROUTE, ELEPHANT_SPEED)
    tusker = timed(TUSKER_ROUTE, ELEPHANT_SPEED * 0.9)
    spec = [
        {'name': 'herd_matriarch', 'model': 'elephant_asian', 'species': 'elephant', 'group': 'herd', 'waypoints': herd},
        # Followers walk the leader's path a few seconds behind, offset to the side.
        {'name': 'herd_female', 'model': 'elephant_grey', 'species': 'elephant', 'group': 'herd',
         'follow': 'herd_matriarch', 'lag_s': 5.0, 'side_m': 2.4},
        {'name': 'herd_calf', 'model': 'elephant_calf', 'species': 'elephant', 'group': 'herd',
         'follow': 'herd_matriarch', 'lag_s': 3.0, 'side_m': -1.8},
        {'name': 'tusker', 'model': 'elephant_tusker', 'species': 'elephant', 'group': 'tusker', 'waypoints': tusker},
        {'name': 'leopard', 'model': 'leopard', 'species': 'leopard', 'group': 'leopard',
         'waypoints': timed(LEOPARD_ROUTE, LEOPARD_SPEED)},
    ]
    for i, (model, route) in enumerate(CATTLE):
        wps = timed(route + [route[0]], COW_SPEED)
        spec.append({'name': f'cattle_{i + 1}', 'model': model, 'species': 'cow', 'group': 'cattle',
                     'waypoints': wps, 'loop': True})
    return spec


def animal_include(a, pose):
    x, y, yaw = pose
    return (f'    <include><name>{a["name"]}</name><uri>model://{a["model"]}</uri>'
            f'<pose>{x:.2f} {y:.2f} 0 0 0 {yaw:.3f}</pose></include>')


def start_pose(a, by_name):
    if 'follow' in a:
        lead = by_name[a['follow']]['waypoints']
        (_, x0, y0), (_, x1, y1) = lead[0], lead[1]
        yaw = math.atan2(y1 - y0, x1 - x0)
        # Behind the leader by lag (at the herd's speed) and to the side.
        back = a['lag_s'] * ELEPHANT_SPEED
        return (x0 - back * math.cos(yaw) - a['side_m'] * math.sin(yaw),
                y0 - back * math.sin(yaw) + a['side_m'] * math.cos(yaw), yaw)
    (_, x0, y0), (_, x1, y1) = a['waypoints'][0], next(w for w in a['waypoints'] if w[1:] != a['waypoints'][0][1:])
    return x0, y0, math.atan2(y1 - y0, x1 - x0)


# ---------------------------------------------------------------------------
# World
# ---------------------------------------------------------------------------

def tower_model(t):
    """Mast with a solar panel and a pan-tilt head carrying boresighted RGB
    and thermal cameras (real towers in TN/Karnataka carry the same pair).
    Static; the sensors publish on fixed /tower/<id>/... topics."""
    yaw, pitch = tower_aim(t)
    tid = t['id'].lower()
    h = t['h']
    hz = CAMERA_KIND[t['kind']]['hz']
    cam = lambda name, typ, size, extra, fmt='': f'''        <sensor name="{name}" type="{typ}">
          <pose>0.25 0 0 0 0 0</pose>
          <topic>/tower/{tid}/{'image' if typ == 'camera' else 'thermal'}</topic>
          <camera><horizontal_fov>{TOWER_HFOV}</horizontal_fov>
            <image><width>{size[0]}</width><height>{size[1]}</height>{fmt}</image>
            <clip><near>0.3</near><far>400</far></clip></camera>
          <always_on>1</always_on><update_rate>{hz}</update_rate>{extra}
        </sensor>'''
    thermal_plugin = '''
          <plugin filename="gz-sim-thermal-sensor-system" name="gz::sim::systems::ThermalSensor">
            <min_temp>250</min_temp><max_temp>350</max_temp><resolution>0.01</resolution></plugin>'''
    grey = '<material><ambient>0.45 0.45 0.45 1</ambient><diffuse>0.55 0.55 0.55 1</diffuse></material>'
    return f'''    <!-- Camera tower {t['id']}: {t['label']}. Aimed at ({t['look'][0]}, {t['look'][1]}). -->
    <model name="tower_{tid}"><static>true</static><pose>{t['x']} {t['y']} 0 0 0 {yaw:.4f}</pose>
      <link name="mast">
        <visual name="pole"><pose>0 0 {h / 2} 0 0 0</pose><geometry><cylinder><radius>0.12</radius><length>{h}</length></cylinder></geometry>{grey}</visual>
        <visual name="base"><pose>0 0 0.2 0 0 0</pose><geometry><box><size>0.9 0.9 0.4</size></box></geometry>{grey}</visual>
        <visual name="solar"><pose>-0.3 0 {h - 1.2} 0 -0.5 0</pose><geometry><box><size>0.9 1.4 0.05</size></box></geometry>
          <material><ambient>0.05 0.08 0.2 1</ambient><diffuse>0.08 0.12 0.3 1</diffuse><specular>0.6 0.6 0.6 1</specular></material></visual>
        <visual name="cabinet"><pose>0.18 0 1.4 0 0 0</pose><geometry><box><size>0.25 0.4 0.5</size></box></geometry>{grey}</visual>
      </link>
      <link name="head"><pose>0 0 {h + 0.15} 0 {pitch:.4f} 0</pose>
        <visual name="housing"><pose>0.1 0.09 0 0 0 0</pose><geometry><box><size>0.35 0.12 0.12</size></box></geometry>
          <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material></visual>
        <visual name="housing_ir"><pose>0.1 -0.09 0 0 0 0</pose><geometry><box><size>0.3 0.12 0.12</size></box></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material></visual>
{cam('rgb', 'camera', CAMERA_KIND[t['kind']]['rgb'], '')}
{cam('thermal', 'thermal', TOWER_THERMAL, thermal_plugin, '<format>L16</format>')}
      </link>
    </model>'''


def write_world(L, spec):
    header_src = open(os.path.join(WORLDS_SRC, 'disaster.sdf')).read()
    # disaster.sdf's proven header: physics, systems, thermal atmosphere,
    # lights (sun/fill names used by day/night), sky domes, GPS origin.
    start = header_src.index('<world name="disaster">')
    end = header_src.index('<include>\n      <uri>model://disaster_ground</uri>')
    header = header_src[start:end].replace('<world name="disaster">', '<world name="forest">')
    # Physics stays at 1 kHz (disaster.sdf's 0.001 s step). 500 Hz would halve
    # its CPU, but ArduCopter then refuses to arm: "PreArm: Gyro 0 rate 500Hz
    # < loop rate*1.8 720Hz" (tried 2026-11-01).
    payload = header_src[header_src.index('    <!-- SAR payload package'):header_src.index('  </world>')]
    heroes = '\n'.join(
        f'    <include><name>{h["name"]}</name><uri>https://fuel.gazebosim.org/1.0/{h["uri"].split("/")[0]}/models/{h["uri"].split("/")[1]}</uri>'
        f'<pose>{h["x"]:.2f} {h["y"]:.2f} 0 0 0 {h["yaw"]:.2f}</pose><static>true</static></include>' for h in L['hero'])
    by_name = {a['name']: a for a in spec}
    beasts = '\n'.join(animal_include(a, start_pose(a, by_name)) for a in spec)
    world = f'''<?xml version="1.0" ?>
<!--
  GENERATED by worldgen/generate_forest.py - edit the generator, not this
  file. 200 x 200 m forest edge (PS000183): forest + elephant corridor (N),
  camera towers T1-T3, crop fields, village (S); forest station at the origin.
  Same frame, lights, sky and GPS origin as disaster.sdf.
-->
<sdf version="1.9">
  {header}
    <include><uri>model://forest_scenery</uri></include>

    <!-- Village houses and the forest station office (Fuel, render-checked in Harmonic) -->
{heroes}
    <include><name>forest_jeep</name><uri>model://pickup</uri><pose>-8 -9 0 0 0 1.57</pose><static>true</static></include>

    <!-- Camera towers (Sentinel) -->
{chr(10).join(tower_model(t) for t in TOWERS)}

    <!-- Animals: CC BY models (models/ANIMALS_ATTRIBUTION.md), moved by worldgen/animal_mover.py -->
{beasts}
    <include><name>village_dog</name><uri>model://dog</uri><pose>-72 14 0 0 0 -0.6</pose></include>

{payload}
  </world>
</sdf>
'''
    for d in (WORLDS_SRC, WORLDS_INSTALL):
        if os.path.isdir(d):
            with open(os.path.join(d, 'forest.sdf'), 'w') as f:
                f.write(world)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seed', type=int, default=1, help='accepted for the dashboard; the scenario is scripted')
    ap.add_argument('--scenery', action='store_true', help='rebuild the scenery model, basemap, risk layer and intel')
    args = ap.parse_args()

    L = build_layout(random.Random(SCENERY_SEED))
    if args.scenery or not os.path.isfile(os.path.join(SCENERY, 'model.sdf')):
        stats = build_scenery(L, np.random.default_rng(SCENERY_SEED))
        build_basemap(L)
        build_risk(L)
        write_mission(L)
        print('scenery:', stats)
    spec = animals()
    write_world(L, spec)
    with open(ANIMALS, 'w') as f:
        json.dump({'world': 'forest', 'speed_ms': {'elephant': ELEPHANT_SPEED, 'cow': COW_SPEED},
                   'triggered_groups': ['herd', 'tusker', 'leopard'], 'animals': spec}, f, indent=2)
    with open(TRUTH, 'w') as f:
        json.dump({'scenario': 'scripted', 'animals': spec}, f, indent=2)
    herd = spec[0]['waypoints']
    print(f'world: forest, {len(spec)} animals; herd route {herd[-1][0]:.0f} s after the scenario starts, '
          f'towers {", ".join(t["id"] for t in TOWERS)}')


if __name__ == '__main__':
    main()
