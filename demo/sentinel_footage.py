#!/usr/bin/env python3
"""Run Sentinel's detector on REAL footage, for the demo video's inset.

The same model and the same box style as the camera towers in the sim
(ardu_ws/sentinel_detector.py), on a real camera-trap / CCTV / phone clip of
elephants, so the video can show that the detector is not only good on
rendered elephants. Only elephants are red alerts; cattle, dogs etc. are
labelled "no alert" (size can't be measured without a calibrated camera, so
here the class name decides).

    python3 demo/sentinel_footage.py clip.mp4 -o demo/out/real_footage.mp4 \
        --label "Camera trap, Kerala" --credit "Footage: <owner>, <licence>"

Use footage you have the right to use, and put its credit on screen
(--credit). Needs ultralytics and imageio-ffmpeg (both already used here).
"""
import argparse
import subprocess
from pathlib import Path

import cv2
import imageio_ffmpeg
from ultralytics import YOLO

MODEL = str(Path(__file__).resolve().parents[1] / 'ardu_ws' / 'yolov8m.pt')
ANIMALS = [16, 17, 18, 19, 20, 21, 22, 23]
RED, GREEN = (40, 40, 235), (60, 190, 60)


def draw(img, boxes, names, banner, credit):
    alert = False
    for b in boxes:
        x1, y1, x2, y2 = (int(v) for v in b.xyxy[0])
        name = names[int(b.cls)]
        is_alert = name in ('elephant', 'bear')
        alert = alert or is_alert
        c = RED if is_alert else GREEN
        label = 'cattle' if name in ('cow', 'sheep', 'horse') else name
        text = f'{label.upper()} {float(b.conf):.2f}' + ('' if is_alert else '  no alert')
        s = max(0.55, img.shape[1] / 1600)
        cv2.rectangle(img, (x1, y1), (x2, y2), c, max(2, int(2 * s)))
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, s, 1)
        cv2.rectangle(img, (x1, max(y1 - th - 10, 0)), (x1 + tw + 8, max(y1, th + 10)), c, -1)
        cv2.putText(img, text, (x1 + 4, max(y1 - 6, th + 4)), cv2.FONT_HERSHEY_SIMPLEX, s, (255, 255, 255), 1, cv2.LINE_AA)
    h = int(28 * max(1, img.shape[1] / 1280))
    cv2.rectangle(img, (0, 0), (img.shape[1], h), (0, 0, 0), -1)
    cv2.putText(img, banner + (' | ELEPHANT' if alert else ''), (10, int(h * 0.7)), cv2.FONT_HERSHEY_SIMPLEX,
                h / 45, (80, 80, 255) if alert else (230, 230, 230), 1, cv2.LINE_AA)
    if credit:
        cv2.putText(img, credit, (10, img.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, h / 60, (230, 230, 230), 1, cv2.LINE_AA)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('video')
    ap.add_argument('-o', '--out', default='demo/out/real_footage.mp4')
    ap.add_argument('--label', default='Real footage | same detector as the towers')
    ap.add_argument('--credit', default='', help='on-screen credit for the footage (required for CC BY)')
    ap.add_argument('--conf', type=float, default=0.35)
    ap.add_argument('--start', type=float, default=0.0, help='seconds to skip')
    ap.add_argument('--duration', type=float, default=0.0, help='seconds to keep (0 = all)')
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.set(cv2.CAP_PROP_POS_MSEC, args.start * 1000)
    model = YOLO(MODEL)
    ff = subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
                           '-s', f'{w}x{h}', '-r', f'{fps}', '-i', '-', '-c:v', 'libx264', '-crf', '16', '-pix_fmt', 'yuv420p',
                           args.out], stdin=subprocess.PIPE)
    n = elephants = 0
    while True:
        ok, frame = cap.read()
        if not ok or (args.duration and n >= args.duration * fps):
            break
        r = model.predict(frame, conf=args.conf, classes=ANIMALS, imgsz=max(640, min(1280, w)), verbose=False)[0]
        elephants += any(model.names[int(b.cls)] == 'elephant' for b in r.boxes)
        ff.stdin.write(draw(frame, r.boxes, model.names, args.label, args.credit).tobytes())
        n += 1
    ff.stdin.close()
    ff.wait()
    print(f'{n} frames -> {args.out}; elephant in {elephants} ({100 * elephants / max(n, 1):.0f}%)')


if __name__ == '__main__':
    main()
