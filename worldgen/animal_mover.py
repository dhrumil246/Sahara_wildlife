#!/usr/bin/env python3
"""Walks the forest world's animals along their scripted routes.

The animals are static visual models (no physics), so this puts each one
where its route has it at the current simulation time, through the world's
set_pose_vector service, like heat_follower.py does for casualties' heat.
The thermal system is on the models' own visuals, so their heat moves with
them.

- Looping groups (the grazing cattle) run from sim time 0.
- Triggered groups (the herd, the lone tusker) wait at their first
  waypoint, hidden under the canopy, until a scenario command arrives on
  the gz topic /wildlife/scenario (gz.msgs.StringMsg):
      "start herd" | "start tusker" | "reset"   (reset puts everyone back)
  e.g. gz topic -t /wildlife/scenario -m gz.msgs.StringMsg -p 'data: "start herd"'
- Followers walk their leader's path lag_s behind and side_m to its left.
- Heading follows the direction of travel (central difference over 1 s,
  so corners turn smoothly); a small bob while walking.
- Publishes the true poses on /wildlife/truth (gz.msgs.Pose_V) for scoring
  test runs. The drone and the detectors never read it.

Part of the simulated world: it reads the generator's
resq_mavlink/forest_animals.json.

    python3 worldgen/animal_mover.py [--rate 15] [--autostart herd]
"""
import argparse
import json
import math
import os
import threading
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node, SubscribeOptions

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ANIMALS = os.path.join(ROOT, 'resq_mavlink', 'forest_animals.json')
BOB_M = 0.04        # vertical gait bob while walking
STRIDE_S = 0.9      # bob period


def route_xy(wps, t, loop):
    """(x, y) on a waypoint list [[t, x, y], ...] at route time t (clamped, or looped)."""
    if loop:
        t %= wps[-1][0]
    if t <= wps[0][0]:
        return wps[0][1], wps[0][2]
    for (t0, x0, y0), (t1, x1, y1) in zip(wps, wps[1:]):
        if t0 <= t <= t1:
            f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
            return x0 + f * (x1 - x0), y0 + f * (y1 - y0)
    return wps[-1][1], wps[-1][2]


class Animal:
    def __init__(self, spec, by_name):
        self.spec = spec
        self.name = spec['name']
        self.loop = spec.get('loop', False)
        self.leader = by_name.get(spec.get('follow'))
        self.yaw = None

    def xy(self, t):
        if self.leader is None:
            return route_xy(self.spec['waypoints'], t, self.loop)
        # On the leader's path lag_s behind, side_m to the left of its heading.
        tl = t - self.spec['lag_s']
        x, y = self.leader.xy(tl)
        yaw = self.leader.heading(tl)
        s = self.spec['side_m']
        return x - s * math.sin(yaw), y + s * math.cos(yaw)

    def heading(self, t):
        """Direction of travel around t (followers: their leader's, lag_s
        earlier); while standing, the heading of the last leg walked."""
        src, tt = (self, t) if self.leader is None else (self.leader, t - self.spec['lag_s'])
        x0, y0 = src.xy(tt - 0.5)
        x1, y1 = src.xy(tt + 0.5)
        if math.hypot(x1 - x0, y1 - y0) > 0.05:
            return math.atan2(y1 - y0, x1 - x0)
        wps = src.spec['waypoints']
        if src.loop:
            tt %= wps[-1][0]
        best = None
        for (t0, ax, ay), (_, bx, by) in zip(wps, wps[1:]):
            if (ax, ay) != (bx, by) and (best is None or t0 <= tt):
                best = math.atan2(by - ay, bx - ax)
        return best

    def pose(self, t):
        x, y = self.xy(t)
        yaw = self.heading(t)
        if self.yaw is not None:  # turn at most 0.08 rad per update (~70 deg/s at 15 Hz)
            d = math.atan2(math.sin(yaw - self.yaw), math.cos(yaw - self.yaw))
            yaw = self.yaw + max(-0.08, min(0.08, d))
        self.yaw = yaw
        xa, ya = self.xy(t - 0.2)
        moving = math.hypot(x - xa, y - ya) > 0.02
        z = BOB_M * abs(math.sin(math.pi * t / STRIDE_S)) if moving else 0.0
        return x, y, z, yaw


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rate', type=float, default=15.0)
    ap.add_argument('--autostart', action='append', default=[], help='start a triggered group at once (repeatable)')
    args = ap.parse_args()
    with open(ANIMALS) as f:
        spec = json.load(f)
    world = spec['world']
    by_name = {}
    animals = []
    for a in spec['animals']:  # leaders come before their followers in the file
        animals.append(Animal(a, by_name))
        by_name[a['name']] = animals[-1]
    triggered = set(spec['triggered_groups'])

    node = Node()
    lock = threading.Lock()
    state = {'t': None, 'start': {}}  # group -> sim time it started

    def on_clock(msg):
        with lock:
            state['t'] = msg.sim.sec + msg.sim.nsec * 1e-9

    def on_scenario(msg):
        words = msg.data.strip().lower().split()
        with lock:
            if words[:1] == ['reset']:
                state['start'].clear()
                print('animal_mover: reset, triggered groups back at their start', flush=True)
            elif len(words) == 2 and words[0] == 'start' and words[1] in triggered and state['t'] is not None:
                state['start'][words[1]] = state['t']
                print(f'animal_mover: {words[1]} started at sim t={state["t"]:.1f}s', flush=True)
            else:
                print(f'animal_mover: ignored scenario command {msg.data!r} '
                      f'(expected "start <{"|".join(sorted(triggered))}>" or "reset")', flush=True)

    # /clock comes every physics step (1 kHz). Unthrottled, those Python
    # callbacks starved the service replies: ~10% of set_pose_vector calls
    # timed out (500 ms stalls, a visible stutter in the walk).
    clock_opts = SubscribeOptions()
    clock_opts.msgs_per_sec = 60
    node.subscribe(Clock, '/clock', on_clock, clock_opts)
    node.subscribe(StringMsg, '/wildlife/scenario', on_scenario)
    truth_pub = node.advertise('/wildlife/truth', Pose_V)
    service = f'/world/{world}/set_pose_vector'
    print(f'animal_mover: {len(animals)} animals in {world} at {args.rate:.0f} Hz; '
          f'waiting for "start <{"|".join(sorted(triggered))}>" on /wildlife/scenario', flush=True)

    failures = 0
    while True:
        time.sleep(1.0 / args.rate)
        with lock:
            t = state['t']
            starts = dict(state['start'])
            for g in args.autostart:
                if t is not None and g not in state['start']:
                    state['start'][g] = starts[g] = t
                    print(f'animal_mover: {g} autostarted at sim t={t:.1f}s', flush=True)
            args.autostart = [g for g in args.autostart if g not in state['start']]
        if t is None:
            continue
        req = Pose_V()
        for a in animals:
            group = a.spec['group']
            if group in triggered:
                rt = t - starts[group] if group in starts else 0.0
            else:
                rt = t
            x, y, z, yaw = a.pose(rt)
            p = req.pose.add()
            p.name = a.name
            p.position.x, p.position.y, p.position.z = x, y, z
            p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        # Short timeout: a late reply doesn't lose the pose (Gazebo has the
        # request already), but waiting for it stalls the walk.
        ok, rep = node.request(service, req, Pose_V, Boolean, 100)
        if not (ok and rep.data):
            failures += 1
            if failures in (1, 10, 100, 1000):
                print(f'animal_mover: set_pose_vector {"refused" if ok else "timed out"} at sim t={t:.1f}s '
                      f'({failures} so far)', flush=True)
        truth_pub.publish(req)


if __name__ == '__main__':
    main()
