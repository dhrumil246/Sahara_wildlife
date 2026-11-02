#!/usr/bin/env python3
"""Cut the PS000183 wildlife film from the raw recordings, with the
narration (demo/voiceover.json), subtitles, captions and a quiet music bed.

    python3 demo/voiceover.py     # once (needs internet): demo/out/vo/*.mp3
    python3 demo/film.py          # -> demo/out/sahara-wildlife.mp4
    python3 demo/film.py 7        # re-render only segment 7, then re-join

Everything is ffmpeg (imageio-ffmpeg build: libx264 + libass, no drawtext),
so captions and cards are ASS subtitles (shaping=complex for Devanagari).
Screen recordings lose the browser chrome (top 112 px) and sit a little
smaller, under a caption band; iPhone HDR clips are tone-mapped.
Each segment is rendered to demo/out/film_tmp/ and joined without
re-encoding; the soundtrack is mixed in numpy and muxed at the end.
"""
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import imageio_ffmpeg
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RAW = ROOT / 'raw-videos'
OUT = ROOT / 'demo' / 'out'
TMP = OUT / 'film_tmp'
VO = OUT / 'vo'
FF = imageio_ffmpeg.get_ffmpeg_exe()
FPS = 30
BG = '0E1A14'  # card / letterbox colour (dark forest green)
spec = importlib.util.spec_from_file_location('promo_sound', ROOT / 'promo' / 'sound.py')
S = importlib.util.module_from_spec(spec)
spec.loader.exec_module(S)

KIND = {'watch': '2563EB', 'ok': '15803D', 'warn': 'D97706', 'danger': 'DC2626', 'info': '475569', 'real': '7C3AED'}
VO_TEXT = {l['id']: l['text'] for l in json.loads((ROOT / 'demo' / 'voiceover.json').read_text())['lines']}
BROWSER = 'crop=1920:968:0:112,scale=1786:900,pad=1920:1080:67:104:color=0x' + BG
HDR = ('zscale=t=linear:npl=203,format=gbrpf32le,zscale=p=bt709,tonemap=hable:desat=0,'
       'zscale=t=bt709:m=bt709:r=tv,format=yuv420p,scale=1920:1080')

# A segment is a clip (src, a, b, speed, look) or a card (dur, draw). 'vo' is
# (line id, seconds into the segment); 'cap' are (t0, t1, kind, text) chips;
# 'badge' sits top-right; 'sfx' are (t, sound) one-shots.
SEGS = [
    dict(card='title', dur=4.5, sfx=[(0.2, 'whoosh')]),
    dict(src='A1_ColdOpen.mp4', a=60, b=70, speed=1, look='browser', vo=('01_cold_open', 0.4),
         badge='SIMULATION · NIGHT', cap=[(0.6, 9.8, 'danger', 'C1 trail camera · thermal · ELEPHANT')],
         sfx=[(0.6, 'ping')]),
    dict(card='problem', dur=13.6, vo=('02_problem', 0.6)),
    dict(src='A2_Decoy.mp4', a=8, b=40, speed=2.5, look='browser', vo=('03_cameras', 0.5),
         badge='SIMULATION · 2.5× SPEED',
         cap=[(0.4, 6.2, 'ok', 'Cattle at T1  →  no alert'),
              (6.4, 12.7, 'info', 'Only elephants and leopards raise an alert')]),
    dict(src='../demo/out/real_footage.mp4', a=0, b=6.8, speed=1, look='full', vo=('04_real_footage', 0.6),
         badge='REAL FOOTAGE · LICENSED', cap=[(0.4, 6.6, 'real', 'Same model, unchanged, on real elephants')]),
    dict(card='tusker', dur=11.5, vo=('05_tusker', 0.8)),
    dict(src='Drone_Dashboard_Split.mp4', a=55, b=175, speed=10, look='browser',
         badge='SIMULATION · 10× SPEED',
         cap=[(0.3, 11.9, 'watch', 'Herd seen at C1, then C2  ·  watching, no launch yet')]),
    dict(src='Drone_Dashboard_Split.mp4', a=340, b=440, speed=5, look='browser', vo=('06_herd_launch', 0.8),
         badge='SIMULATION · 5× SPEED',
         cap=[(0.3, 7.5, 'danger', 'T2: C1 → C2 → T2  ·  path confirmed toward the village'),
              (7.7, 19.9, 'ok', 'Drone launches by itself')],
         sfx=[(0.3, 'confirm')]),
    dict(src='Drone_Dashboard_Split.mp4', a=460, b=520, speed=4, look='browser', vo=('07_tracking', 0.4),
         badge='SIMULATION · 4× SPEED',
         cap=[(0.3, 14.9, 'ok', 'Tracking 3 elephants incl. a calf  ·  25 m behind, never descends')]),
    dict(src='Thermal_Tracking_With_Map.mp4', a=24, b=72, speed=4, look='browser',
         badge='SIMULATION · NIGHT · 4× SPEED',
         cap=[(0.3, 11.9, 'info', 'At night the drone follows the herd on its thermal camera')]),
    dict(src='Thermal_Tracking_With_Map.mp4', a=80, b=108, speed=2.8, look='browser', vo=('08_alerts', 1.0),
         badge='SIMULATION',
         cap=[(0.3, 9.9, 'warn', 'Predicted path  ·  time to the village  ·  risk map')]),
    dict(src='Thermal_Tracking_With_Map.mp4', a=132.5, b=140, speed=1, look='browser',
         badge='SIMULATION', cap=[(0.3, 7.4, 'warn', 'Alerts sent to officers and villagers  ·  English + Hindi')]),
    dict(src='Leopard.mp4', a=168, b=195, speed=3, look='browser', vo=('09_leopard', 0.5),
         badge='SIMULATION · 3× SPEED',
         cap=[(0.3, 8.9, 'danger', 'L1 trail camera · LEOPARD (told apart by its size)')]),
    dict(split=('Phone_Recording.MP4', 4, 37, 'Community_Report_Leopard.mp4', 0, 56), dur=14,
         vo=('10_villager', 0.5), badge='PHONE: REAL  ·  DRONE: SIMULATION 4×',
         cap=[(0.3, 6.8, 'real', 'A villager reports a leopard from their phone'),
              (7.0, 13.9, 'ok', 'The drone goes to check')], sfx=[(9.5, 'ping')]),
    dict(src='IMG_2878.MOV', a=1.5, b=8.5, speed=1, look='hdr', vo=('11_real_drone', 1.0),
         badge='REAL FLIGHT · MANUAL', cap=[(0.4, 6.9, 'real', 'Our S500 · real night flight on campus')]),
    dict(src='IMG_2879.MOV', a=45, b=50, speed=1, look='hdr', badge='REAL FLIGHT · MANUAL',
         cap=[(0.0, 4.9, 'real', 'Same airframe and ArduPilot firmware as the simulation')]),
    dict(src='IMG_2878.MOV', a=27, b=31, speed=1, look='hdr', badge='REAL FLIGHT · MANUAL'),
    dict(card='fits', dur=17.5, vo=('12_fits', 0.8)),
    dict(card='end', dur=6.5, vo=('13_close', 0.6), sfx=[(0.3, 'impact')]),
]


def col(hexrgb, alpha=0):
    r, g, b = hexrgb[0:2], hexrgb[2:4], hexrgb[4:6]
    return f'&H{alpha:02X}{b}{g}{r}&'.upper()


def ts(t):
    t = max(0.0, t)
    return f'{int(t // 3600)}:{int(t // 60 % 60):02d}:{t % 60:05.2f}'


HEADER = f"""[Script Info]
ScriptType: v4.00+
PlayResX: 1920
PlayResY: 1080
WrapStyle: 0
ScaledBorderAndShadow: yes

[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Sub,Ubuntu,40,&H00FFFFFF,&H00FFFFFF,&H50000000,&H50000000,0,0,0,0,100,100,0,0,3,12,0,2,240,240,48,1
Style: Chip,Ubuntu,34,&H00FFFFFF,&H00FFFFFF,&H00000000,&H00000000,1,0,0,0,100,100,0,0,3,12,0,7,67,67,30,1
Style: Badge,Ubuntu,24,&H00FFFFFF,&H00FFFFFF,&H60000000,&H60000000,1,0,0,0,100,100,1,0,3,8,0,9,67,67,36,1
Style: Text,Ubuntu,48,&H00FFFFFF,&H00FFFFFF,&H00000000,&H00000000,0,0,0,0,100,100,0,0,1,0,0,5,0,0,0,1
Style: Hindi,Lohit Devanagari,52,&H00FFFFFF,&H00FFFFFF,&H00000000,&H00000000,0,0,0,0,100,100,0,0,1,0,0,5,0,0,0,1
"""
EV = 'Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text\n'


def dlg(t0, t1, style, text, layer=0):
    return f'Dialogue: {layer},{ts(t0)},{ts(t1)},{style},,0,0,0,,{text}\n'


def txt(t0, t1, x, y, text, size=48, color='FFFFFF', bold=False, an=5, fade=(250, 250), font=None, layer=1):
    f = f'\\fn{font}' if font else ''
    return dlg(t0, t1, 'Text', f'{{\\an{an}\\pos({x},{y})\\fs{size}\\1c{col(color)}\\b{int(bold)}{f}\\fad({fade[0]},{fade[1]})}}{text}', layer)


def box(t0, t1, x, y, w, h, color, alpha=0, outline=None, fade=(250, 250), layer=0):
    shape = f'm 0 0 l {w} 0 {w} {h} 0 {h}'
    o = f'\\bord3\\3c{col(outline)}' if outline else '\\bord0'
    return dlg(t0, t1, 'Text', f'{{\\an7\\pos({x},{y})\\1c{col(color)}\\1a&H{alpha:02X}&{o}\\shad0\\fad({fade[0]},{fade[1]})\\p1}}{shape}{{\\p0}}', layer)


def card_events(name, d):
    e = []
    G, A, R, MUTED = '22C55E', 'F59E0B', 'EF4444', '9CA3AF'
    if name == 'title':
        e += [txt(0.2, d, 960, 430, 'SAHARA', 150, bold=True, fade=(500, 400)),
              txt(0.7, d, 960, 560, 'Early warning for human–wildlife conflict', 50, 'D1FAE5', fade=(400, 400)),
              txt(1.2, d, 960, 650, 'सहारा: जंगल भी सुरक्षित, गाँव भी', 44, MUTED, font='Lohit Devanagari', fade=(400, 400)),
              txt(1.6, d, 960, 960, 'PS000183  ·  Simulation + real hardware', 28, MUTED, fade=(400, 400))]
    elif name == 'problem':
        e += [txt(0.3, d, 960, 330, '≈ 500', 180, A, bold=True, fade=(400, 300)),
              txt(0.8, d, 960, 480, 'people killed by elephants in India every year', 52, fade=(400, 300)),
              txt(1.4, d, 960, 620, '…and leopards, bears and other wildlife add many more', 36, 'D1D5DB', fade=(400, 300)),
              txt(1.0, d, 960, 548, 'Source: MoEFCC figures reported to Parliament', 26, MUTED, fade=(400, 300)),
              txt(5.6, d, 960, 740, 'Forests already have cameras.', 50, fade=(400, 300)),
              txt(6.6, d, 960, 816, 'But an animal is lost the moment it walks out of view.', 50, 'FCA5A5', bold=True, fade=(400, 300))]
    elif name == 'tusker':
        e.append(txt(0.2, d, 960, 200, 'Not every sighting needs a drone', 64, bold=True, fade=(300, 300)))
        cams, x0, w, gap, y = ['C1', 'C2', 'T2', 'T4', 'Village'], 330, 200, 70, 380
        for i, c in enumerate(cams):
            x, lit = x0 + i * (w + gap), i < 2
            t = 1.0 + 1.3 * i if lit else 0.6
            e.append(box(t, d, x, y, w, 110, A if lit else BG, outline=None if lit else '6B7280', fade=(200, 300)))
            e.append(txt(t, d, x + w // 2, y + 55, c, 46, '111827' if lit else MUTED, bold=True, fade=(200, 300), layer=2))
            if i < len(cams) - 1:
                e.append(txt(0.6, d, x + w + gap // 2, y + 55, '→', 44, MUTED, fade=(200, 300)))
        e += [txt(4.2, d, 330 + 270 + 100, 560, '←  seen at C1 again: turned back', 44, A, bold=True, an=5, fade=(300, 300)),
              txt(6.2, d, 960, 700, 'No launch  ·  officers informed  ·  village not disturbed', 50, G, bold=True, fade=(300, 300)),
              txt(7.4, d, 960, 790, 'The drone flies only when the path toward the village is confirmed (T2).', 34, MUTED, fade=(300, 300))]
    elif name == 'fits':
        e.append(txt(0.2, d, 960, 150, 'Plugs into what already exists', 62, bold=True, fade=(300, 300)))
        items = [('Forest cameras', 'towers and camera traps'), ('Alert feeds', 'TrailGuard-style, Telegram, e-mail'),
                 ('Villagers', 'reports from any phone')]
        for i, (h, s) in enumerate(items):
            x = 300 + i * 660
            e += [box(0.6 + 0.4 * i, d, x - 270, 240, 540, 170, '17332A', fade=(250, 300)),
                  txt(0.6 + 0.4 * i, d, x, 300, h, 44, bold=True, fade=(250, 300), layer=2),
                  txt(0.6 + 0.4 * i, d, x, 362, s, 30, MUTED, fade=(250, 300), layer=2)]
        e.append(txt(2.4, d, 960, 495, '↓   one event bus  →  camera-path logic  →  drone  →  alerts   ↓', 40, G, bold=True, fade=(300, 300)))
        lim = ['Animal paths are scripted; sped-up clips are marked',
               'The risk layer uses sample data',
               'The real flight was manual; the autonomous mission shown is simulated',
               'A real deployment needs a detector trained on Indian camera-trap images']
        e += [box(7.0, d, 360, 600, 1200, 330, '111827', fade=(300, 300)),
              txt(7.0, d, 960, 650, 'Honest limits', 40, A, bold=True, fade=(300, 300), layer=2)]
        e += [txt(7.4 + 0.3 * i, d, 960, 715 + 52 * i, s, 32, 'E5E7EB', fade=(250, 300), layer=2) for i, s in enumerate(lim)]
    elif name == 'end':
        e += [txt(0.2, d, 960, 440, 'SAHARA', 150, bold=True, fade=(500, 600)),
              txt(0.6, d, 960, 580, 'सहारा: जंगल भी सुरक्षित, गाँव भी', 54, 'D1FAE5', font='Lohit Devanagari', fade=(400, 600)),
              txt(1.0, d, 960, 660, 'Safe forests, safe villages', 40, MUTED, fade=(400, 600)),
              txt(1.4, d, 960, 960, 'SEVA Hackathon  ·  PS000183  ·  AI for Human–Wildlife Conflict', 28, MUTED, fade=(400, 600))]
    return e


def vo_len(line):
    return json.loads((VO / 'durations.json').read_text())[line]


def seg_dur(s):
    if 'dur' in s:
        return s['dur']
    return (s['b'] - s['a']) / s['speed']


def write_ass(i, s, d):
    ev = []
    if 'card' in s:
        ev += card_events(s['card'], d)
    for t0, t1, kind, text in s.get('cap', []):
        ev.append(dlg(t0, min(t1, d - 0.05), 'Chip', f'{{\\3c{col(KIND[kind])}\\fad(200,200)}}{text}', 3))
    if s.get('badge'):
        ev.append(dlg(0, d, 'Badge', s['badge'], 3))
    if s.get('vo'):
        line, off = s['vo']
        ev.append(dlg(off, off + vo_len(line), 'Sub', VO_TEXT[line], 4))
    if 'real_footage' in s.get('src', ''):
        ev.append(dlg(0, d, 'Text', '{\\an3\\pos(1880,1000)\\fs24\\1c&H00C0C0C0&}Licensed stock footage', 3))
    path = TMP / f'seg{i:02d}.ass'
    path.write_text(HEADER + '\n[Events]\n' + EV + ''.join(ev))
    return path


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        sys.exit(f'ffmpeg failed:\n{" ".join(map(str, cmd))}\n{r.stderr[-2000:]}')


def render(i, s):
    d = seg_dur(s)
    frames = round(d * FPS)
    ass = write_ass(i, s, d)
    out = TMP / f'seg{i:02d}.mp4'
    enc = ['-frames:v', str(frames), '-r', str(FPS), '-an', '-c:v', 'libx264', '-preset', 'medium', '-crf', '19',
           '-pix_fmt', 'yuv420p', str(out)]
    sub = f"ass=filename={ass}:shaping=complex"
    if 'card' in s:
        run([FF, '-y', '-f', 'lavfi', '-i', f'color=c=0x{BG}:s=1920x1080:r={FPS}:d={d + 0.5}',
             '-vf', sub] + enc)
    elif 'split' in s:
        pf, pa, pb, cf, ca, cb = s['split']
        ps, cs = (pb - pa) / d, (cb - ca) / d
        graph = (f'color=c=0x{BG}:s=1920x1080:r={FPS}[bg];'
                 f'[0:v]setpts=(PTS-STARTPTS)/{ps},fps={FPS},scale=-2:960[ph];'
                 f'[1:v]setpts=(PTS-STARTPTS)/{cs},fps={FPS},crop=1920:968:0:112,scale=1300:-2[cm];'
                 f'[bg][ph]overlay=110:60:shortest=1[a];[a][cm]overlay=560:250,{sub}')
        run([FF, '-y', '-ss', str(pa), '-t', str(pb - pa), '-i', str(RAW / pf),
             '-ss', str(ca), '-t', str(cb - ca), '-i', str(RAW / cf), '-filter_complex', graph] + enc)
    else:
        look = {'browser': BROWSER, 'hdr': HDR, 'full': 'scale=1920:1080'}[s['look']]
        vf = f"setpts=(PTS-STARTPTS)/{s['speed']},fps={FPS},{look},{sub}"
        run([FF, '-y', '-ss', str(s['a']), '-t', str(s['b'] - s['a'] + 1), '-i', str(RAW / s['src']), '-vf', vf] + enc)
    return frames / FPS


def decode(path):
    raw = subprocess.run([FF, '-v', 'error', '-i', str(path), '-f', 'f32le', '-ac', '1', '-ar', str(S.SR), '-'],
                         capture_output=True).stdout
    return np.frombuffer(raw, np.float32).astype(np.float64)


def soundtrack(starts, total):
    n = int(total * S.SR) + S.SR
    vo, music, sfx = np.zeros(n), np.zeros(n), np.zeros(n)
    for s, t0 in zip(SEGS, starts):
        if s.get('vo'):
            line, off = s['vo']
            S.place(vo, decode(VO / f'{line}.mp3'), t0 + off)
        for t, name in s.get('sfx', []):
            S.place(sfx, getattr(S, name)(), t0 + t)
    # Am - F - C - G pad, 4 s per chord, with a soft root note underneath.
    chords = [(220.0, 261.6, 329.6), (174.6, 220.0, 261.6), (196.0, 261.6, 329.6), (196.0, 246.9, 293.7)]
    roots = [110.0, 87.3, 130.8, 98.0]
    t, k = 0.0, 0
    while t < total:
        S.place(music, S.pad_chord(chords[k % 4], 4.6), t)
        tt = S.t_axis(4.2)
        S.place(music, 0.25 * np.sin(2 * np.pi * roots[k % 4] * tt) * np.clip(tt / 0.3, 0, 1) * np.clip((4.2 - tt) / 0.8, 0, 1), t)
        t, k = t + 4.0, k + 1
    # Duck the music under the voice (0.3 s ramps), fade in/out.
    talk = (np.abs(vo) > 1e-4).astype(float)
    win = int(0.3 * S.SR)
    talk = np.convolve(talk, np.ones(win * 2) / (win * 2), 'same')
    duck = 1.0 - 0.7 * np.clip(talk * 4, 0, 1)
    fade = np.clip(np.arange(n) / (2 * S.SR), 0, 1) * np.clip((total - np.arange(n) / S.SR) / 3.0, 0, 1)
    vo /= max(1e-6, np.abs(vo).max())
    music = music / max(1e-6, np.abs(music).max()) * 0.22 * duck * fade
    mix = vo * 0.9 + music + sfx * 0.6
    mix = mix[: int(total * S.SR)]
    mix *= 0.95 / max(1e-6, np.abs(mix).max())
    path = TMP / 'soundtrack.wav'
    S.write_wav(path, np.vstack([mix, mix]))
    return path


def fit_voice():
    """Stretch a segment that is shorter than its narration line."""
    for s in SEGS:
        if not s.get('vo'):
            continue
        need = s['vo'][1] + vo_len(s['vo'][0]) + 0.6
        if seg_dur(s) < need:
            if 'dur' in s:
                s['dur'] = need
            else:
                s['b'] = s['a'] + need * s['speed']


def main(only):
    TMP.mkdir(parents=True, exist_ok=True)
    fit_voice()
    durs = []
    for i, s in enumerate(SEGS):
        seg = TMP / f'seg{i:02d}.mp4'
        if only and i not in only and seg.exists():
            durs.append(round(seg_dur(s) * FPS) / FPS)
            continue
        print(f'segment {i:2d}: {s.get("card") or s.get("src") or "split"}')
        durs.append(render(i, s))
    starts = list(np.cumsum([0] + durs[:-1]))
    total = float(sum(durs))
    (TMP / 'list.txt').write_text(''.join(f"file 'seg{i:02d}.mp4'\n" for i in range(len(SEGS))))
    run([FF, '-y', '-f', 'concat', '-safe', '0', '-i', str(TMP / 'list.txt'), '-c', 'copy', str(TMP / 'video.mp4')])
    wav = soundtrack(starts, total)
    final = OUT / 'sahara-wildlife.mp4'
    run([FF, '-y', '-i', str(TMP / 'video.mp4'), '-i', str(wav), '-map', '0:v', '-map', '1:a', '-c:v', 'copy',
         '-c:a', 'aac', '-b:a', '192k', '-shortest', '-movflags', '+faststart', str(final)])
    print(f'{final}  {total:.1f} s ({int(total // 60)}:{total % 60:04.1f})' + ('  OVER 5:00!' if total > 300 else ''))


if __name__ == '__main__':
    main({int(a) for a in sys.argv[1:]})
