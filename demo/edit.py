"""Edit the raw OBS recordings into the SAHARA demo video.

  python demo/edit.py stills 6 10.5 19 [--cut end]  # QA composites -> demo/out/stills/
  python demo/edit.py build [--workers 10]  # -> demo/out/sahara-demo.mp4 (main cut, ends on
                                            #    the Live Testing card) and sahara-end.mp4 (closing logo)

The edit (segments, crops, captions, wipes) lives in demo/overlay.html; this
script reads it from the page, so there is one source of truth.
  1. ffmpeg cuts each segment from raw-videos/, retimes it and places the
     cropped panels (Gazebo, dashboard, feed) where the layout's cards are.
  2. Chrome renders the overlay (paper, captions, badges, wipes) as
     transparent frames with holes over the cards.
  3. demo/sound_demo.py scores it; ffmpeg composites and muxes.
The recordings' own audio is discarded.
Needs playwright (system Chrome), imageio-ffmpeg, Pillow; numpy/scipy for the score.
"""
import argparse
import functools
import http.server
import json
import socketserver
import subprocess
import sys
import threading
from multiprocessing import Process
from pathlib import Path

import imageio_ffmpeg
from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]
DEMO = ROOT / "demo"
OUT = DEMO / "out"
RAW = {"day": ROOT / "raw-videos" / "Continous-DayMode.mp4", "night": ROOT / "raw-videos" / "NightModeContinous.mp4"}
FF = imageio_ffmpeg.get_ffmpeg_exe()
FPS = 60


def serve():
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(ROOT))
    handler.func.log_message = lambda *a: None
    httpd = socketserver.ThreadingTCPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd.server_address[1]


def open_overlay(pw, port, cut="main"):
    browser = pw.chromium.launch(channel="chrome", args=["--force-color-profile=srgb"])
    page = browser.new_page(viewport={"width": 1920, "height": 1080})
    page.goto(f"http://127.0.0.1:{port}/demo/overlay.html?cut={cut}")
    page.wait_for_function("window.ready !== undefined")
    page.evaluate("window.ready")
    return browser, page


def seg_filter(seg, edit):
    """filter_complex placing the layout's crops of one retimed segment on black."""
    holes = edit["layouts"][seg["layout"]]
    n = len(holes)
    parts = [f"[0:v]setpts=(PTS-STARTPTS)/{seg['speed']:.6f},fps={FPS},split={n}" + "".join(f"[s{i}]" for i in range(n))]
    for i, (src, dst, _tag) in enumerate(holes):
        x, y, w, h = edit["src"][src]
        dx, dy, dw, dh = dst
        parts.append(f"[s{i}]crop={w}:{h}:{x}:{y},scale={dw}:{dh}:flags=lanczos[c{i}]")
    last = "[1:v]"
    for i, (src, dst, _tag) in enumerate(holes):
        tag = f"[o{i}]" if i < n - 1 else "[out]"
        parts.append(f"{last}[c{i}]overlay={dst[0]}:{dst[1]}:shortest=1{tag}")
        last = f"[o{i}]"
    return ";".join(parts)


def render_segments(edit):
    d = OUT / "segments" / edit["cut"]
    d.mkdir(parents=True, exist_ok=True)
    files = []
    enc = ["-c:v", "libx264", "-preset", "fast", "-crf", "14", "-pix_fmt", "yuv420p", "-r", str(FPS)]
    for i, seg in enumerate(edit["segs"]):
        frames = round(seg["dur"] * FPS)
        f = d / f"seg{i:02d}.mp4"
        black = ["-f", "lavfi", "-i", f"color=c=black:s=1920x1080:r={FPS}:d={seg['dur'] + 1}"]
        if seg.get("src"):
            cmd = [FF, "-y", "-loglevel", "error", "-ss", f"{seg['a']:.3f}", "-t", f"{seg['b'] - seg['a'] + 0.5:.3f}",
                   "-i", str(RAW[seg["src"]]), *black, "-filter_complex", seg_filter(seg, edit),
                   "-map", "[out]", "-an", "-frames:v", str(frames), *enc, str(f)]
        else:
            cmd = [FF, "-y", "-loglevel", "error", *black, "-map", "0:v", "-frames:v", str(frames), *enc, str(f)]
        subprocess.run(cmd, check=True)
        files.append(f)
        print(f"segment {i:02d}  {seg.get('src') or 'card'}  {seg['dur']}s")
    lst = d / "list.txt"
    lst.write_text("".join(f"file '{p}'\n" for p in files))
    base = OUT / f"base-{edit['cut']}.mp4"
    subprocess.run([FF, "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", str(lst), "-c", "copy", str(base)], check=True)
    return base


def overlay_worker(port, frames, fdir, cut):
    with sync_playwright() as pw:
        browser, page = open_overlay(pw, port, cut)
        stage = page.locator("#stage")
        for f in frames:
            page.evaluate(f"renderAt({f / FPS})")
            stage.screenshot(path=str(fdir / f"f{f:05d}.png"), omit_background=True)
        browser.close()


def render_overlay(port, n, workers, cut):
    fdir = OUT / "overlay" / cut
    fdir.mkdir(parents=True, exist_ok=True)
    for p in fdir.glob("*.png"):
        p.unlink()
    procs = [Process(target=overlay_worker, args=(port, list(range(i, n, workers)), fdir, cut)) for i in range(workers)]
    for p in procs:
        p.start()
    for p in procs:
        p.join()
    if any(p.exitcode for p in procs):
        sys.exit("overlay worker failed")
    return fdir


def source_time(edit, t):
    for seg in edit["segs"]:
        if seg["t0"] <= t < seg["t1"]:
            return seg, (seg["a"] + (t - seg["t0"]) * seg["speed"]) if seg.get("src") else None
    return edit["segs"][-1], None


def stills(port, times, cut="main"):
    from PIL import Image
    d = OUT / "stills"
    d.mkdir(parents=True, exist_ok=True)
    with sync_playwright() as pw:
        browser, page = open_overlay(pw, port, cut)
        edit = page.evaluate("window.EDIT")
        for t in times:
            page.evaluate(f"renderAt({t})")
            ov = d / "ov.png"
            page.locator("#stage").screenshot(path=str(ov), omit_background=True)
            canvas = Image.new("RGB", (1920, 1080), "black")
            seg, st = source_time(edit, t)
            if st is not None:
                raw = d / "raw.png"
                subprocess.run([FF, "-y", "-loglevel", "error", "-ss", f"{st:.3f}", "-i", str(RAW[seg["src"]]), "-frames:v", "1", str(raw)], check=True)
                src = Image.open(raw).convert("RGB")
                for name, (dx, dy, dw, dh), _ in edit["layouts"][seg["layout"]]:
                    x, y, w, h = edit["src"][name]
                    canvas.paste(src.crop((x, y, x + w, y + h)).resize((dw, dh), Image.LANCZOS), (dx, dy))
            canvas = canvas.convert("RGBA")
            canvas.alpha_composite(Image.open(ov).convert("RGBA"))
            out = d / f"{cut}-t{t:06.2f}.png"
            canvas.convert("RGB").save(out)
            print(out)
        browser.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["stills", "build"])
    ap.add_argument("times", nargs="*", type=float)
    ap.add_argument("--workers", type=int, default=10)
    ap.add_argument("--cut", default="main", choices=["main", "end"])
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    port = serve()
    if args.mode == "stills":
        stills(port, args.times, args.cut)
        return

    for cut, name in (("main", "sahara-demo.mp4"), ("end", "sahara-end.mp4")):
        build_cut(port, cut, name, args.workers)


def build_cut(port, cut, name, workers):
    with sync_playwright() as pw:
        browser, page = open_overlay(pw, port, cut)
        edit = page.evaluate("window.EDIT")
        events = page.evaluate("sfxEvents()")
        browser.close()
    (OUT / f"edit-{cut}.json").write_text(json.dumps({"cut": cut, "duration": edit["duration"], "events": events, "segs": edit["segs"]}, indent=1))
    base = render_segments(edit)
    n = round(edit["duration"] * FPS)
    fdir = render_overlay(port, n, workers, cut)
    print(cut, "overlay frames:", n)
    wav = OUT / f"soundtrack-{cut}.wav"
    subprocess.run(["python3", "-W", "ignore", str(DEMO / "sound_demo.py"), cut], check=True)
    final = OUT / name
    subprocess.run([FF, "-y", "-loglevel", "error", "-i", str(base), "-framerate", str(FPS), "-i", str(fdir / "f%05d.png"),
                    "-i", str(wav),
                    "-filter_complex", "[0:v][1:v]overlay=0:0:format=auto,format=yuv420p[v]",
                    "-map", "[v]", "-map", "2:a", "-c:v", "libx264", "-preset", "slow", "-crf", "16",
                    "-profile:v", "high", "-c:a", "aac", "-b:a", "256k", "-shortest", "-movflags", "+faststart",
                    "-color_primaries", "bt709", "-color_trc", "bt709", "-colorspace", "bt709", str(final)], check=True)
    print(final)


if __name__ == "__main__":
    main()
