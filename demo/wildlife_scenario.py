#!/usr/bin/env python3
"""Run (and score) one take of the wildlife demo through the dashboard API.

The same sequence every time, so recorded takes are repeatable and the
editor has a timeline of when each beat happened:

  launch forest world -> day/night -> Sentinel feed -> auto-dispatch on
  -> start the herd -> (towers see it -> drone launches by itself ->
  intercept -> tracking -> alerts) -> optional villager report -> stop

    python3 demo/wildlife_scenario.py --launch --mode night          # full take from scratch
    python3 demo/wildlife_scenario.py --mode day --report-at 150     # sim already running
    python3 demo/wildlife_scenario.py --score-only --duration 240    # just watch and score

Writes demo/out/wildlife_timeline_<time>.jsonl (one line per beat) and
prints a score against the animals' true positions (/wildlife/truth):
herd position error, time to first alert, to launch, to the drone's own
sighting, alerts sent.
"""
import argparse
import json
import math
import os
import subprocess
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ROS_ENV = ROOT / 'command_center' / 'backend' / 'ros_env.sh'
OUT = ROOT / 'demo' / 'out'


class Api:
    def __init__(self, base):
        self.base = base.rstrip('/')

    def get(self, path):
        with urllib.request.urlopen(self.base + path, timeout=10) as r:
            return json.loads(r.read())

    def post(self, path, body=None):
        req = urllib.request.Request(self.base + path, data=json.dumps(body or {}).encode(), method='POST',
                                     headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=60) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:
            return json.loads(e.read() or b'{}')


def truth():
    """True animal positions {name: (x, y)} from the sim, or {}."""
    try:
        out = subprocess.run([str(ROS_ENV), 'gz', 'topic', '-e', '-n', '1', '-t', '/wildlife/truth'],
                             capture_output=True, text=True, timeout=8).stdout
    except subprocess.TimeoutExpired:
        return {}
    pos, name, x = {}, None, None
    for line in out.splitlines():
        line = line.strip()
        if line.startswith('name:'):
            name = line.split('"')[1]
        elif line.startswith('x:') and name:
            x = float(line.split()[1])
        elif line.startswith('y:') and name and x is not None:
            pos[name] = (x, float(line.split()[1]))
            name, x = None, None
    return pos


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--server', default='http://localhost:5000')
    ap.add_argument('--launch', action='store_true', help='launch the forest world first')
    ap.add_argument('--mode', choices=('day', 'night'), default='night')
    ap.add_argument('--group', default='herd', choices=('herd', 'tusker'))
    ap.add_argument('--report-at', type=float, default=0, help='seconds after the start: a villager reports the herd')
    ap.add_argument('--duration', type=float, default=420, help='give up after this many seconds')
    ap.add_argument('--score-only', action='store_true', help="don't drive anything, just watch and score")
    args = ap.parse_args()
    api = Api(args.server)
    OUT.mkdir(parents=True, exist_ok=True)
    log_path = OUT / f'wildlife_timeline_{time.strftime("%Y%m%d_%H%M%S")}.jsonl'
    log = open(log_path, 'w')
    t0 = time.time()

    def beat(kind, **data):
        rec = dict(t=round(time.time() - t0, 1), kind=kind, **data)
        log.write(json.dumps(rec) + '\n')
        log.flush()
        print(f"[{rec['t']:6.1f}s] {kind}: {json.dumps(data, ensure_ascii=False)[:160]}", flush=True)

    if not args.score_only:
        if args.launch:
            beat('launch', result=api.post('/api/sim/launch', {'world': 'forest'}).get('message'))
            for _ in range(180):
                try:
                    m = api.get('/api/map/latest')
                    if m.get('origin') and m.get('wildlife'):
                        break
                except Exception:  # noqa: BLE001 - not up yet
                    pass
                time.sleep(2)
            time.sleep(10)  # let Gazebo settle (EKF, cameras)
            beat('bridge_ready')
        beat('mode', result=api.post('/api/mode', {'mode': args.mode}).get('message'))
        st = api.post('/api/feed/start')
        beat('feed', result=st.get('message'))
        time.sleep(20)  # model load + first frames
        beat('auto_dispatch', result=api.post('/api/wildlife/config', {'auto_dispatch': True}).get('message'))
        beat('scenario', result=api.post('/api/wildlife/scenario', {'command': f'start {args.group}'}).get('message'))
    t_start = time.time()

    stage = level = None
    seen_events = 0
    first = {}
    errors = []
    reported = False
    while time.time() - t_start < args.duration:
        time.sleep(1.0)
        try:
            m = api.get('/api/map/latest')
        except Exception:  # noqa: BLE001
            continue
        w = m.get('wildlife') or {}
        herd = w.get('herd')
        el = round(time.time() - t_start, 1)
        if m.get('stage') != stage:
            stage = m.get('stage')
            beat('stage', stage=stage, scenario_s=el)
            first.setdefault(stage, el)
        if w.get('level') != level:
            level = w.get('level')
            beat('level', level=level, scenario_s=el, herd=herd and {k: herd.get(k) for k in ('e', 'n', 'eta_s', 'village_dist_m')})
            first.setdefault(f'level_{level}', el)
        total = m.get('events_total', 0)
        events = m.get('events', [])
        for ev in events[max(0, len(events) - (total - seen_events)):] if total >= seen_events else events:
            beat('event', level=ev['level'], text=ev['text'], scenario_s=el)
        seen_events = total
        if any(s.get('alert') and s.get('source', '').startswith('T') for s in w.get('sightings', [])):
            first.setdefault('tower_alert', el)
        if w.get('uav_visual'):
            first.setdefault('drone_visual', el)
        if herd and int(el) % 5 == 0:
            tr = truth()
            herd_truth = [p for k, p in tr.items() if k.startswith(args.group)]
            if herd_truth:
                cx = sum(p[0] for p in herd_truth) / len(herd_truth)
                cy = sum(p[1] for p in herd_truth) / len(herd_truth)
                errors.append(math.hypot(herd['e'] - cx, herd['n'] - cy))
        if args.report_at and not reported and el >= args.report_at and herd:
            reported = True
            r = api.post('/api/community/report', {'species': 'elephant', 'count': herd['count'], 'e': herd['e'] + 3,
                                                   'n': herd['n'] - 2, 'note': 'Elephants near the paddy, south side'})
            beat('villager_report', result=r.get('message'))
        if stage == 'COMPLETE' and 'TRACKING' in first:
            break

    outbox = api.get('/api/wildlife/outbox')
    errors.sort()
    summary = {
        'first_tower_alert_s': first.get('tower_alert'), 'drone_launch_s': first.get('ARMING'),
        'drone_visual_s': first.get('drone_visual'), 'tracking_s': first.get('TRACKING'),
        'level_warning_s': first.get('level_warning'), 'level_critical_s': first.get('level_critical'),
        'herd_error_median_m': round(errors[len(errors) // 2], 1) if errors else None,
        'herd_error_p90_m': round(errors[int(len(errors) * 0.9)], 1) if errors else None,
        'alerts_sent': len(outbox.get('messages', [])), 'telegram_live': outbox.get('live'),
    }
    beat('summary', **summary)
    print(f'\nTimeline: {log_path}')


if __name__ == '__main__':
    main()
