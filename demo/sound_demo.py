"""Score for the demo edit: an upbeat 120 BPM track plus UI sound effects.

Reads demo/out/edit-<cut>.json (written by edit.py) and reuses the synth voices
from promo/sound.py.
  main: intro swell -> groove from the first wipe -> breakdown on "Night falls"
        -> groove -> resolves and fades on the Live Testing card.
  live: the groove under the real-flight clips, breaking down on the drop freeze.
  end:  impact, a ringing A-minor chord, fade out.
Writes demo/out/soundtrack-<cut>.wav.
"""
import importlib.util
import json
from pathlib import Path

import numpy as np
from scipy.signal import fftconvolve

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "demo" / "out"
spec = importlib.util.spec_from_file_location("promo_sound", ROOT / "promo" / "sound.py")
S = importlib.util.module_from_spec(spec)
spec.loader.exec_module(S)
SR = S.SR


def resolve(music, send, t0, d):
    """A-minor pad + low A that rings out and fades over d seconds."""
    pad = S.pad_chord((220.0, 261.6, 329.6), d + 0.4, 1.0)
    S.place(music, pad, t0)
    S.place(send, pad * 0.6, t0)
    S.place(music, S.bass_note(55.0, d, 1.0), t0)


def groove(music, send, drums, side, g0, g1, breaks):
    """120 BPM Am-F-C-G groove from g0 to g1; drums and plucked bass drop out in breaks."""
    beat = 0.5
    chords = [(55.0, (220.0, 261.6, 329.6)), (43.65, (174.6, 220.0, 261.6)),
              (65.41, (196.0, 261.6, 329.6)), (49.0, (196.0, 246.9, 293.7))]
    bar, tt = 0, g0
    while tt < g1 - 0.01:
        root, triad = chords[bar % 4]
        in_break = any(a <= tt < b for a, b in breaks)
        d = min(2.3, g1 - tt + 0.1)
        S.place(music, S.pad_chord(triad, d, 0.8 if in_break else 1.0), tt)
        S.place(send, S.pad_chord(triad, d, 0.5), tt)
        if in_break:
            S.place(music, S.bass_note(root, min(1.9, g1 - tt), 0.8), tt)
        else:
            for k in range(8):
                if (k in (3, 7) and bar % 2) or tt + k * 0.25 >= g1:
                    continue
                S.place(music, S.bass_note(root * (2 if k in (3, 6) else 1), 0.22), tt + k * 0.25)
        tt += 2.0
        bar += 1
    k, tt = 0, g0
    while tt < g1 - 0.01:
        if not any(a <= tt < b for a, b in breaks):
            S.place(drums, S.kick(0.95), tt)
            i = int(tt * SR)
            dip = 1 - 0.55 * S.env(int(0.3 * SR), 0.0, 0.3, 4)
            side[i:i + len(dip)] = np.minimum(side[i:i + len(dip)], dip[: len(side[i:i + len(dip)])])
            if k % 2 == 1:
                S.place(drums, S.clap(1.0), tt)
            S.place(drums, S.hat(0.9), tt + beat / 2)
            S.place(drums, S.hat(0.45), tt + beat / 4)
            S.place(drums, S.hat(0.45), tt + 3 * beat / 4)
        tt += beat
        k += 1


def build(cut):
    cfg = json.loads((OUT / f"edit-{cut}.json").read_text())
    segs = cfg["segs"]
    dur = cfg["duration"]
    n = int((dur + 0.05) * SR)
    drums, music, sfx, send = (np.zeros(n) for _ in range(4))
    side = np.ones(n)
    fns = {"slam": S.slam, "impact": S.impact, "pop": S.pop, "tick": S.tick, "whoosh": S.whoosh, "swish": S.swish,
           "sweep": S.sweep, "ping": S.ping, "counter": S.counter, "snap": S.snap, "buzz": S.buzz,
           "confirm": S.confirm, "switch": S.switch, "thud": S.thud, "stamp": S.stamp}

    if cut == "main":
        night = next(s for s in segs if s.get("id") == "night")
        live = next(s for s in segs if s.get("id") == "live")
        g0, g1 = segs[1]["t0"], live["t0"]   # groove from the first wipe to the Live Testing card
        t = S.t_axis(g0 + 0.2)
        S.place(music, S.pad_chord((110.0, 164.8, 220.0), g0 + 0.2, 0.6) * np.clip(t / (g0 - 0.4), 0, 1) ** 1.4, 0)
        groove(music, send, drums, side, g0, g1, [(night["t0"], night["t1"])])
        S.place(sfx, S.riser(1.3, 0.9), night["t1"] - 1.3)
        S.place(sfx, S.riser(1.3, 0.9), live["t0"] - 1.3)
        resolve(music, send, live["t0"], live["dur"])
    elif cut == "live":
        fz = next(s for s in segs if s.get("id") == "freeze")
        groove(music, send, drums, side, 0.0, dur, [(fz["t0"], fz["t1"])])
        S.place(sfx, S.riser(1.3, 0.9), fz["t1"] - 1.3)
    else:
        resolve(music, send, 0.0, dur)

    for e in cfg["events"]:
        typ, t0, v = e["type"], e["t"], e["v"]
        x = S.riser(1.3, v) if typ == "riser" else fns[typ](v)
        off = -0.375 if typ == "whoosh" else 0.0
        S.place(sfx, x, max(0.0, t0 + off))
        if typ in ("confirm", "impact", "slam", "pop", "thud"):
            S.place(send, x * 0.6, max(0.0, t0 + off))

    music *= side
    verb = fftconvolve(send, S.reverb_ir(), mode="full")[:n] * 0.22
    L = drums + music * 0.92 + sfx + verb
    R = drums + music + sfx + np.roll(verb, int(0.011 * SR))
    mix = S.hp(np.stack([L, R]), 28)
    # fade out over the last stretch so the file ends clean (main: before the live clips)
    fade = {"main": 1.6, "live": 1.4}.get(cut, 2.0)
    tail = np.clip((dur - S.t_axis(dur + 0.05)) / fade, 0, 1)
    mix *= tail[: mix.shape[1]]
    mix = np.tanh(mix * 1.4 / max(1e-6, np.percentile(np.abs(mix), 99.9)))
    mix *= 10 ** (-1 / 20) / np.max(np.abs(mix))
    return mix


if __name__ == "__main__":
    import sys
    cut = sys.argv[1] if len(sys.argv) > 1 else "main"
    mix = build(cut)
    S.write_wav(OUT / f"soundtrack-{cut}.wav", mix)
    print(f"demo/out/soundtrack-{cut}.wav  {mix.shape[1] / SR:.1f}s")
