#!/usr/bin/env python3
"""Narration for the wildlife film, in a free Microsoft neural voice via
edge-tts (voice set in voiceover.json). Needs internet.

    python3 demo/voiceover.py            # all lines -> demo/out/vo/<id>.mp3
    python3 demo/voiceover.py 05_tusker  # just one line (after editing it)

Lines, voice and speed live in demo/voiceover.json. Also writes
demo/out/vo/subtitles.srt (one cue per line laid end to end with a
0.6 s gap) and durations.json, so the edit can place each line on its clip.
"""
import asyncio
import json
import subprocess
import sys
from pathlib import Path

import edge_tts
import imageio_ffmpeg

DEMO = Path(__file__).resolve().parent
CFG = DEMO / 'voiceover.json'
OUT = DEMO / 'out' / 'vo'
GAP_S = 0.6


def duration(path):
    info = subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), '-i', str(path)], capture_output=True, text=True).stderr
    hms = info.split('Duration: ')[1].split(',')[0]
    h, m, s = hms.split(':')
    return int(h) * 3600 + int(m) * 60 + float(s)


def srt_time(t):
    ms = int(round(t * 1000))
    return f'{ms // 3600000:02d}:{ms // 60000 % 60:02d}:{ms // 1000 % 60:02d},{ms % 1000:03d}'


async def speak(cfg, line):
    path = OUT / f"{line['id']}.mp3"
    await edge_tts.Communicate(line['text'], cfg['voice'], rate=cfg['rate'], pitch=cfg['pitch']).save(str(path))
    return path


async def main(only):
    cfg = json.loads(CFG.read_text())
    OUT.mkdir(parents=True, exist_ok=True)
    for line in cfg['lines']:
        if not only or line['id'] in only:
            print('speaking', line['id'])
            await speak(cfg, line)
    t, cues, durations = 0.0, [], {}
    for i, line in enumerate(cfg['lines'], 1):
        path = OUT / f"{line['id']}.mp3"
        if not path.exists():
            continue
        d = durations[line['id']] = round(duration(path), 2)
        cues.append(f"{i}\n{srt_time(t)} --> {srt_time(t + d)}\n{line['text']}\n")
        t += d + GAP_S
    (OUT / 'subtitles.srt').write_text('\n'.join(cues))
    (OUT / 'durations.json').write_text(json.dumps(durations, indent=1))
    print(f'{len(durations)} lines, {t:.1f} s of narration -> {OUT}')


if __name__ == '__main__':
    asyncio.run(main(set(sys.argv[1:])))
