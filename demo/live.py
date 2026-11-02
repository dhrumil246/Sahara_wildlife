"""Build live-testing.mp4 from the real-flight clips, then join the full film.

  python demo/live.py stills 3 30 38     # QA composites -> demo/out/stills/live-*.png
  python demo/live.py build [--workers 10]
      -> demo/out/live-testing.mp4
      -> demo/out/sahara-full.mp4  (sahara-demo + live-testing + sahara-end)

The edit lives in demo/live.html (segments, captions, the drop marker). The
clips' own audio is discarded; demo/sound_demo.py scores the cut.
"""
import argparse
import json
import subprocess
import sys
from multiprocessing import Process
from pathlib import Path

from playwright.sync_api import sync_playwright

sys.path.insert(0, str(Path(__file__).resolve().parent))
from edit import FF, FPS, OUT, ROOT, serve  # noqa: E402

RAW = {"ay": ROOT / "raw-videos" / "Ayakashi.mp4",
       "wa": ROOT / "raw-videos" / "WhatsApp Video 2026-09-29 at 16.12.17.mp4"}
DEMO = ROOT / "demo"


def open_page(pw, port):
    browser = pw.chromium.launch(channel="chrome", args=["--force-color-profile=srgb"])
    page = browser.new_page(viewport={"width": 1920, "height": 1080})
    page.goto(f"http://127.0.0.1:{port}/demo/live.html")
    page.wait_for_function("window.ready !== undefined")
    page.evaluate("window.ready")
    return browser, page


def card_filter(seg, card):
    x, y, w, h = card
    zoom = ""
    if seg.get("zoom"):
        zx, zy, zw, zh = seg["zoom"]
        zoom = f"crop={zw}:{zh}:{zx}:{zy},"
    return zoom + f"scale={w}:{h}:flags=lanczos,setsar=1"


def render_segments(edit):
    d = OUT / "segments" / "live"
    d.mkdir(parents=True, exist_ok=True)
    x, y = edit["card"][:2]
    enc = ["-c:v", "libx264", "-preset", "fast", "-crf", "14", "-pix_fmt", "yuv420p", "-r", str(FPS)]
    files = []
    for i, seg in enumerate(edit["segs"]):
        frames = round(seg["dur"] * FPS)
        f = d / f"seg{i:02d}.mp4"
        black = ["-f", "lavfi", "-i", f"color=c=black:s=1920x1080:r={FPS}:d={seg['dur'] + 1}"]
        if seg.get("still") is not None:
            still = d / f"still{i:02d}.png"
            subprocess.run([FF, "-y", "-loglevel", "error", "-ss", f"{seg['still']:.3f}", "-i", str(RAW[seg["src"]]),
                            "-frames:v", "1", str(still)], check=True)
            src = ["-loop", "1", "-framerate", str(FPS), "-i", str(still)]
            chain = f"[0:v]{card_filter(seg, edit['card'])}[c]"
        else:
            src = ["-ss", f"{seg['a']:.3f}", "-t", f"{seg['b'] - seg['a'] + 0.5:.3f}", "-i", str(RAW[seg["src"]])]
            chain = f"[0:v]setpts=(PTS-STARTPTS)/{seg['speed']:.6f},fps={FPS},{card_filter(seg, edit['card'])}[c]"
        fc = f"{chain};[1:v][c]overlay={x}:{y}:shortest=1[out]"
        subprocess.run([FF, "-y", "-loglevel", "error", *src, *black, "-filter_complex", fc,
                        "-map", "[out]", "-an", "-frames:v", str(frames), *enc, str(f)], check=True)
        files.append(f)
        print(f"segment {i:02d}  {seg['src']}  {seg['dur']}s")
    lst = d / "list.txt"
    lst.write_text("".join(f"file '{p}'\n" for p in files))
    base = OUT / "base-live.mp4"
    subprocess.run([FF, "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", str(lst), "-c", "copy", str(base)], check=True)
    return base


def overlay_worker(port, frames, fdir):
    with sync_playwright() as pw:
        browser, page = open_page(pw, port)
        stage = page.locator("#stage")
        for f in frames:
            page.evaluate(f"renderAt({f / FPS})")
            stage.screenshot(path=str(fdir / f"f{f:05d}.png"), omit_background=True)
        browser.close()


def render_overlay(port, n, workers):
    fdir = OUT / "overlay" / "live"
    fdir.mkdir(parents=True, exist_ok=True)
    for p in fdir.glob("*.png"):
        p.unlink()
    procs = [Process(target=overlay_worker, args=(port, list(range(i, n, workers)), fdir)) for i in range(workers)]
    for p in procs:
        p.start()
    for p in procs:
        p.join()
    if any(p.exitcode for p in procs):
        sys.exit("overlay worker failed")
    return fdir


def stills(port, times):
    from PIL import Image
    d = OUT / "stills"
    d.mkdir(parents=True, exist_ok=True)
    with sync_playwright() as pw:
        browser, page = open_page(pw, port)
        edit = page.evaluate("window.EDIT")
        cx, cy, cw, ch = edit["card"]
        for t in times:
            page.evaluate(f"renderAt({t})")
            ov = d / "ov.png"
            page.locator("#stage").screenshot(path=str(ov), omit_background=True)
            seg = next((s for s in edit["segs"] if s["t0"] <= t < s["t1"]), edit["segs"][-1])
            st = seg["still"] if seg.get("still") is not None else seg["a"] + (t - seg["t0"]) * seg["speed"]
            raw = d / "raw.png"
            subprocess.run([FF, "-y", "-loglevel", "error", "-ss", f"{st:.3f}", "-i", str(RAW[seg["src"]]), "-frames:v", "1", str(raw)], check=True)
            src = Image.open(raw).convert("RGB")
            if seg.get("zoom"):
                zx, zy, zw, zh = seg["zoom"]
                src = src.crop((zx, zy, zx + zw, zy + zh))
            canvas = Image.new("RGB", (1920, 1080), "black")
            canvas.paste(src.resize((cw, ch), Image.LANCZOS), (cx, cy))
            canvas = canvas.convert("RGBA")
            canvas.alpha_composite(Image.open(ov).convert("RGBA"))
            out = d / f"live-t{t:06.2f}.png"
            canvas.convert("RGB").save(out)
            print(out)
        browser.close()


def join_full():
    parts = [OUT / "sahara-demo.mp4", OUT / "live-testing.mp4", OUT / "sahara-end.mp4"]
    inputs = sum((["-i", str(p)] for p in parts), [])
    fc = "".join(f"[{i}:v][{i}:a]" for i in range(len(parts))) + f"concat=n={len(parts)}:v=1:a=1[v][a]"
    full = OUT / "sahara-full.mp4"
    subprocess.run([FF, "-y", "-loglevel", "error", *inputs, "-filter_complex", fc, "-map", "[v]", "-map", "[a]",
                    "-c:v", "libx264", "-preset", "slow", "-crf", "16", "-pix_fmt", "yuv420p", "-r", str(FPS),
                    "-c:a", "aac", "-b:a", "256k", "-ar", "48000", "-movflags", "+faststart",
                    "-color_primaries", "bt709", "-color_trc", "bt709", "-colorspace", "bt709", str(full)], check=True)
    print(full)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["stills", "build", "join"])
    ap.add_argument("times", nargs="*", type=float)
    ap.add_argument("--workers", type=int, default=10)
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    if args.mode == "join":
        join_full()
        return
    port = serve()
    if args.mode == "stills":
        stills(port, args.times)
        return
    with sync_playwright() as pw:
        browser, page = open_page(pw, port)
        edit = page.evaluate("window.EDIT")
        events = page.evaluate("sfxEvents()")
        browser.close()
    (OUT / "edit-live.json").write_text(json.dumps({"cut": "live", "duration": edit["duration"], "events": events, "segs": edit["segs"]}, indent=1))
    base = render_segments(edit)
    fdir = render_overlay(port, round(edit["duration"] * FPS), args.workers)
    subprocess.run(["python3", "-W", "ignore", str(DEMO / "sound_demo.py"), "live"], check=True)
    final = OUT / "live-testing.mp4"
    subprocess.run([FF, "-y", "-loglevel", "error", "-i", str(base), "-framerate", str(FPS), "-i", str(fdir / "f%05d.png"),
                    "-i", str(OUT / "soundtrack-live.wav"),
                    "-filter_complex", "[0:v][1:v]overlay=0:0:format=auto,format=yuv420p[v]",
                    "-map", "[v]", "-map", "2:a", "-c:v", "libx264", "-preset", "slow", "-crf", "16",
                    "-profile:v", "high", "-c:a", "aac", "-b:a", "256k", "-shortest", "-movflags", "+faststart",
                    "-color_primaries", "bt709", "-color_trc", "bt709", "-colorspace", "bt709", str(final)], check=True)
    print(final)
    join_full()


if __name__ == "__main__":
    main()
