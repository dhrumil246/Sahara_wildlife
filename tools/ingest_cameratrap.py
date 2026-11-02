#!/usr/bin/env python3
"""Turn a folder of camera-trap photos into sightings.

Most of India's camera traps (tens of thousands, e.g. 26,838 locations in
the 2018 tiger estimation; Cuddeback units in published studies) are
offline: staff collect the SD cards. Their photos are still the best record
of where elephants come out of the forest. This runs the same detector as
Sentinel (COCO YOLOv8m) over a batch, keeps the elephants (and bears,
leopards are not a COCO class), takes the time from EXIF, and sends one
sighting per camera and hour to the command center, where they build up
the conflict-risk layer. MegaDetector is the stronger first pass for real
camera-trap archives; its "animal" boxes can feed the same step.

    python3 tools/ingest_cameratrap.py /media/sdcard --cameras tools/cameras.example.csv
    python3 tools/ingest_cameratrap.py photos/ --cameras cams.csv --camera CT-12 --dry-run

Which camera took a photo: --camera for the whole folder, else the first
sub-folder name or filename prefix that is a camera id (CT-12/IMG_001.JPG,
CT-12_0001.JPG).
"""
import argparse
import json
import os
import sys
import time
import urllib.request
from datetime import datetime

from PIL import ExifTags, Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ingest_alert import load_cameras  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL = os.path.join(ROOT, 'ardu_ws', 'yolov8m.pt')
KEEP = {'elephant': 'elephant', 'bear': 'bear'}
EXTS = ('.jpg', '.jpeg', '.png')
DATETIME_TAG = next(k for k, v in ExifTags.TAGS.items() if v == 'DateTimeOriginal')


def taken_at(path):
    try:
        exif = Image.open(path)._getexif() or {}
        return datetime.strptime(exif[DATETIME_TAG], '%Y:%m:%d %H:%M:%S').timestamp()
    except (KeyError, ValueError, AttributeError, OSError):
        return os.path.getmtime(path)


def camera_of(path, folder, cams, forced):
    if forced:
        return forced.upper()
    rel = os.path.relpath(path, folder)
    for part in rel.replace('\\', '/').split('/'):
        for cid in cams:
            if part.upper() == cid or part.upper().startswith(cid + '_'):
                return cid
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folder')
    ap.add_argument('--cameras', required=True)
    ap.add_argument('--camera', help='camera id for every photo in the folder')
    ap.add_argument('--model', default=MODEL)
    ap.add_argument('--conf', type=float, default=0.4)
    ap.add_argument('--imgsz', type=int, default=1280, help='inference size; camera-trap photos are large, keep it high')
    ap.add_argument('--server', default='http://localhost:5000')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    from ultralytics import YOLO  # slow import, after argument errors
    cams = load_cameras(args.cameras)
    model = YOLO(args.model)
    photos = sorted(os.path.join(d, f) for d, _, fs in os.walk(args.folder) for f in fs if f.lower().endswith(EXTS))
    hits = {}  # (camera, species, hour) -> sighting
    for i, path in enumerate(photos):
        cid = camera_of(path, args.folder, cams, args.camera)
        if cid is None or cid not in cams:
            print(f'skip {path}: no camera id', file=sys.stderr)
            continue
        r = model.predict(path, conf=args.conf, imgsz=args.imgsz, verbose=False)[0]
        found = {}
        for b in r.boxes:
            name = model.names[int(b.cls)]
            if name in KEEP:
                found[KEEP[name]] = found.get(KEEP[name], 0) + 1
        t = taken_at(path)
        for species, count in found.items():
            key = (cid, species, int(t // 3600))
            best = hits.get(key)
            if best is None or count > best['count']:
                cam = cams[cid]
                hits[key] = dict({k: cam[k] for k in ('e', 'n', 'lat', 'lon') if k in cam}, species=species,
                                 count=count, t=t, source_label=f"{cam['label']} ({cid})",
                                 note=f'camera trap photo {os.path.basename(path)}')
        if (i + 1) % 50 == 0:
            print(f'{i + 1}/{len(photos)} photos, {len(hits)} sightings', file=sys.stderr)
    items = sorted(hits.values(), key=lambda h: h['t'])
    print(f'{len(photos)} photos -> {len(items)} sightings '
          f"({sum(1 for h in items if h['species'] == 'elephant')} elephant)", file=sys.stderr)
    if not items:
        return 1
    if args.dry_run:
        print(json.dumps(items, indent=2))
        return 0
    req = urllib.request.Request(f'{args.server}/api/events/ingest', data=json.dumps(items).encode(),
                                 headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=30) as res:
        print(json.loads(res.read())['message'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
