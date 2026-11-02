#!/usr/bin/env python3
"""Import alerts from camera systems already in the field.

AI camera traps in India (TrailGuard AI, deployed in Kanha-Pench, Dudhwa
and Jhargram) already send alerts as e-mail, Telegram or WhatsApp
messages: a camera id, what it saw, when, and often a photo. This reads such
messages (plain text, a saved .eml e-mail, or a JSON export), looks the
camera up in a registry of where each one stands, and posts them to the
command center, where they join the tower sightings: on the map, in the
herd tracker, the risk layer and the alerts.

    python3 tools/ingest_alert.py alerts.txt --cameras tools/cameras.example.csv
    python3 tools/ingest_alert.py message.eml --cameras cams.csv --server http://field-laptop:5000
    cat alert.txt | python3 tools/ingest_alert.py - --cameras cams.csv --dry-run

Message format: free text; a line counts as one alert if it names a known
camera id and an animal, e.g.
    "TrailGuard AI | CAM TG-07 | Elephant detected | 2026-11-01 23:42"
    "ALERT: tiger at camera TG-03 (21:15)"
Camera registry (CSV): id,label,e,n   (metres east/north of the command
center's map origin) or id,label,lat,lon.
"""
import argparse
import csv
import email
import json
import re
import sys
import time
import urllib.request
from datetime import datetime

SPECIES = {
    'elephant': 'elephant', 'tusker': 'elephant', 'jumbo': 'elephant', 'हाथी': 'elephant',
    'leopard': 'leopard', 'panther': 'leopard', 'tiger': 'other', 'bear': 'bear', 'sloth bear': 'bear',
    'boar': 'wild boar', 'pig': 'wild boar', 'human': None, 'person': None, 'vehicle': None,
}
TIME_RE = re.compile(r'(\d{4}-\d{2}-\d{2})?[ T]?(\d{1,2}):(\d{2})(?::(\d{2}))?')
COUNT_RE = re.compile(r'\b(\d{1,2})\s*(?:x\s*)?(?:elephants?|tuskers?|animals?)\b', re.I)


def load_cameras(path):
    cams = {}
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            cam = {'label': row.get('label') or row['id']}
            if row.get('e') not in (None, ''):
                cam.update(e=float(row['e']), n=float(row['n']))
            else:
                cam.update(lat=float(row['lat']), lon=float(row['lon']))
            cams[row['id'].strip().upper()] = cam
    return cams


def read_text(path):
    raw = sys.stdin.read() if path == '-' else open(path, encoding='utf-8', errors='replace').read()
    if path.endswith('.eml') or raw.startswith(('From:', 'Received:', 'Return-Path:')):
        msg = email.message_from_string(raw)
        parts = [p.get_payload(decode=True) for p in msg.walk() if p.get_content_type() == 'text/plain']
        body = b'\n'.join(p for p in parts if p).decode('utf-8', errors='replace')
        return f"{msg.get('Subject', '')}\n{body}"
    return raw


def parse_time(text):
    m = TIME_RE.search(text)
    if not m:
        return time.time()
    day = m.group(1) or datetime.now().strftime('%Y-%m-%d')
    stamp = f'{day} {int(m.group(2)):02d}:{m.group(3)}:{m.group(4) or "00"}'
    try:
        return datetime.strptime(stamp, '%Y-%m-%d %H:%M:%S').timestamp()
    except ValueError:
        return time.time()


def parse(text, cams):
    """One alert per line that names a known camera and an animal."""
    if text.lstrip().startswith(('[', '{')):
        items = json.loads(text)
        return items if isinstance(items, list) else [items]
    out = []
    for line in text.splitlines():
        up = line.upper()
        cam_id = next((c for c in cams if re.search(rf'\b{re.escape(c)}\b', up)), None)
        species = next((s for k, s in SPECIES.items() if re.search(rf'\b{re.escape(k)}', line, re.I)), 'none')
        if cam_id is None or species in ('none', None):
            continue
        cam = cams[cam_id]
        count = COUNT_RE.search(line)
        item = {'species': species, 'count': int(count.group(1)) if count else 1, 't': parse_time(line),
                'source_label': f"{cam['label']} ({cam_id})", 'note': line.strip()[:200]}
        item.update({k: cam[k] for k in ('e', 'n', 'lat', 'lon') if k in cam})
        out.append(item)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('source', help="text / .eml / .json file, or - for stdin")
    ap.add_argument('--cameras', required=True, help='camera registry CSV (id,label,e,n or id,label,lat,lon)')
    ap.add_argument('--server', default='http://localhost:5000')
    ap.add_argument('--dry-run', action='store_true', help='print what would be sent')
    args = ap.parse_args()
    items = parse(read_text(args.source), load_cameras(args.cameras))
    if not items:
        print('no alerts found (a line needs a known camera id and an animal)')
        return 1
    if args.dry_run:
        print(json.dumps(items, indent=2, ensure_ascii=False))
        return 0
    req = urllib.request.Request(f'{args.server}/api/events/ingest', data=json.dumps(items).encode(),
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=10) as res:
        print(json.loads(res.read())['message'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
