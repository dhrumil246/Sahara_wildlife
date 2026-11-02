"""Edit the wildlife (PS000183) recordings into the demo film.

  python demo/wildlife_edit.py prepare --timeline demo/out/wildlife_timeline_<t>.jsonl \
      --file night=raw-videos/wildlife/night_take.mp4 --offset night=41.5 \
      --file day=raw-videos/wildlife/day_take.mp4 --offset day=38 \
      --file real=demo/out/real_footage.mp4 --file phone=raw-videos/wildlife/phone.mp4 \
      --file campus=raw-videos/wildlife/s500_campus_night.mp4
  python demo/wildlife_edit.py stills 3 20 60          # QA composites -> demo/out/stills/wl-*.png
  python demo/wildlife_edit.py build [--workers 10]    # -> demo/out/sahara-wildlife.mp4

Same machinery as edit.py (ffmpeg places each recording's crops, Chrome
paints the overlay demo/wildlife.html frame by frame, sound_demo.py scores
it), with two differences:
- Sources and the take's timeline live in demo/wildlife_sources.json.
  "offset" is the recording's second at which the scenario started (the
  frame where the dashboard's Settings button "Start herd" was pressed, or
  wildlife_scenario.py printed "scenario"); simulation beats are then placed
  from the timeline, so a new take needs only new offsets.
- Real sources (real footage, phone, campus flight) start at their offset.
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
ROOT = Path(__file__).resolve().parents[1]
DEMO = ROOT / "demo"
OUT = DEMO / "out"
edit = None  # demo/edit.py, imported only to render (it needs playwright + imageio-ffmpeg)
SOURCES = DEMO / "wildlife_sources.json"
CUTS = {"main": ("wl-main", "sahara-wildlife-body.mp4"), "end": ("wl-end", "sahara-wildlife-end.mp4")}


def load_sources():
    return json.loads(SOURCES.read_text()) if SOURCES.exists() else {"sources": {}, "timeline": {}, "src": {}}


def timeline_from(path):
    """Scenario seconds of each beat key from a wildlife_scenario.py JSONL."""
    beats = [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
    t_scn = next((b["t"] for b in beats if b["kind"] == "scenario"), 0.0)
    tl = {}
    for b in beats:
        if b["kind"] == "stage":
            tl.setdefault(b["stage"], b["scenario_s"])
        elif b["kind"] == "level":
            tl.setdefault(f"level_{b['level']}", b["scenario_s"])
        elif b["kind"] == "villager_report":
            tl.setdefault("villager_report", round(b["t"] - t_scn, 1))
        elif b["kind"] == "summary":
            for k, v in (("tower_alert", b.get("first_tower_alert_s")), ("drone_visual", b.get("drone_visual_s"))):
                if v is not None:
                    tl[k] = v
    return tl


def prepare(args):
    cfg = load_sources()
    if args.timeline:
        cfg["timeline"] = timeline_from(args.timeline)
    for spec, key in ((args.file, "file"), (args.offset, "offset")):
        for item in spec or []:
            name, val = item.split("=", 1)
            cfg["sources"].setdefault(name, {})[key] = float(val) if key == "offset" else val
    SOURCES.write_text(json.dumps(cfg, indent=2))
    print(f"{SOURCES}:\n  timeline {cfg['timeline']}\n  sources {cfg['sources']}")
    missing = [k for k in ("tower_alert", "ARMING", "drone_visual", "level_warning", "level_critical", "villager_report")
               if k not in cfg["timeline"]]
    if missing:
        print(f"  WARNING: timeline lacks {missing}; the beats that use them will fail. Re-run the take "
              "(wildlife_scenario.py --report-at ...) until it reaches them.")


def bind_sources():
    """Import edit.py and point its RAW table at this film's recordings."""
    global edit
    import edit as _edit
    edit = _edit
    cfg = load_sources()
    edit.RAW.clear()
    for name, s in cfg["sources"].items():
        if s.get("file"):
            edit.RAW[name] = ROOT / s["file"]
    # Stills before every recording exists: a segment whose source isn't
    # registered yet is drawn as an empty hole instead of failing.
    source_time = edit.source_time

    def source_time_or_empty(e, t):
        seg, st = source_time(e, t)
        return seg, (st if seg.get("src") in edit.RAW else None)
    edit.source_time = source_time_or_empty
    edit_overlay = edit.open_overlay

    def open_overlay(pw, port, cut="main"):
        browser = pw.chromium.launch(channel="chrome", args=["--force-color-profile=srgb"])
        page = browser.new_page(viewport={"width": 1920, "height": 1080})
        page.goto(f"http://127.0.0.1:{port}/demo/wildlife.html?cut={cut}")
        page.wait_for_function("window.ready !== undefined")
        page.evaluate("window.ready")
        return browser, page
    edit.open_overlay = open_overlay
    return edit_overlay


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["prepare", "stills", "build"])
    ap.add_argument("times", nargs="*", type=float)
    ap.add_argument("--timeline")
    ap.add_argument("--file", action="append", help="name=path (relative to the repo)")
    ap.add_argument("--offset", action="append", help="name=seconds: recording second where the scenario starts")
    ap.add_argument("--workers", type=int, default=10)
    ap.add_argument("--cut", default="main", choices=list(CUTS))
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    if args.mode == "prepare":
        prepare(args)
        return
    bind_sources()
    port = edit.serve()
    if args.mode == "stills":
        edit.stills(port, args.times, args.cut)
        return
    for cut, (tag, name) in CUTS.items():
        # edit.build_cut names its intermediates by cut; give ours their own names.
        edit_cut = tag
        build(port, cut, edit_cut, name, args.workers)
    body, end = OUT / CUTS["main"][1], OUT / CUTS["end"][1]
    lst = OUT / "wildlife_join.txt"
    lst.write_text(f"file '{body}'\nfile '{end}'\n")
    final = OUT / "sahara-wildlife.mp4"
    import subprocess
    import cv2
    FF = edit.FF
    subprocess.run([FF, "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", str(lst), "-c", "copy", str(final)],
                   check=True)
    cap = cv2.VideoCapture(str(final))
    dur = cap.get(cv2.CAP_PROP_FRAME_COUNT) / (cap.get(cv2.CAP_PROP_FPS) or edit.FPS)
    print(f"{final}  {dur // 60:.0f}:{dur % 60:04.1f}" + ("" if dur <= 300 else "  OVER THE 5:00 LIMIT - shorten segments"))


def build(port, cut, tag, name, workers):
    """edit.build_cut with this film's cut tag for intermediate and score files."""
    from playwright.sync_api import sync_playwright
    import subprocess
    FF, FPS = edit.FF, edit.FPS
    with sync_playwright() as pw:
        browser, page = edit.open_overlay(pw, port, cut)
        e = page.evaluate("window.EDIT")
        events = page.evaluate("sfxEvents()")
        browser.close()
    e["cut"] = tag
    (OUT / f"edit-{tag}.json").write_text(json.dumps({"cut": tag, "duration": e["duration"], "events": events, "segs": e["segs"]}, indent=1))
    base = edit.render_segments(e)
    n = round(e["duration"] * FPS)
    fdir = edit.render_overlay(port, n, workers, cut)
    subprocess.run(["python3", "-W", "ignore", str(DEMO / "sound_demo.py"), tag], check=True)
    final = OUT / name
    subprocess.run([FF, "-y", "-loglevel", "error", "-i", str(base), "-framerate", str(FPS), "-i", str(fdir / "f%05d.png"),
                    "-i", str(OUT / f"soundtrack-{tag}.wav"),
                    "-filter_complex", "[0:v][1:v]overlay=0:0:format=auto,format=yuv420p[v]",
                    "-map", "[v]", "-map", "2:a", "-c:v", "libx264", "-preset", "slow", "-crf", "16",
                    "-profile:v", "high", "-c:a", "aac", "-b:a", "256k", "-shortest", "-movflags", "+faststart",
                    "-color_primaries", "bt709", "-color_trc", "bt709", "-colorspace", "bt709", str(final)], check=True)
    print(final, f"{e['duration']:.1f}s")


if __name__ == "__main__":
    main()
