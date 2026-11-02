"""Alert outbox for the wildlife mission (PS000183): who gets told what, when.

Messages go to two channels: forest officers and the village. Each one is
in English and a local language (Hindi here; the templates are the place to
add Kannada, Tamil, Odia... - have a native speaker check them). With
TELEGRAM_BOT_TOKEN and a chat id set, they are really sent through the
Telegram Bot API (photo + caption for sightings); without, they are kept
as "simulated" so the dashboard's phone panel still shows exactly what
would have gone out.

    TELEGRAM_BOT_TOKEN=...  TELEGRAM_CHAT_ID=...             one chat for both channels
    TELEGRAM_CHAT_OFFICERS=...  TELEGRAM_CHAT_VILLAGE=...     or one per channel

When (from the tower sightings and the mission bridge's map):
  - first elephant sighting of an episode (no alert for EPISODE_GAP_S) -> both
  - herd level rises to warning / critical                           -> both
  - the drone gets the herd in view                                   -> officers
"""
import json
import math
import os
import threading
import time
import urllib.request
import uuid
from collections import deque

EPISODE_GAP_S = 300.0
OTHER_DANGER = {'leopard': ('🐆', 'तेंदुए'), 'bear': ('🐻', 'भालू')}
# (icon, English subject, Hindi subject) for path messages.
PATH_WHO = {'elephant': ('🐘', 'Elephants are', 'हाथी'), 'leopard': ('🐆', 'A leopard is', 'तेंदुआ'),
            'bear': ('🐻', 'A sloth bear is', 'भालू')}
EARTH_RADIUS_M = 6371000.0

TEXT = {
    'sighting': {
        'en': '🐘 ELEPHANT ALERT: {count} elephant{s} seen by the {place} camera at {time}. Keep away from the forest '
              'edge and stay indoors. Location: {maps}',
        'hi': '🐘 हाथी चेतावनी: {time} बजे {place} कैमरे पर {count} हाथी देखे गए। जंगल के किनारे से दूर रहें, '
              'घर के अंदर रहें।',
    },
    'warning': {
        'en': '⚠️ WARNING: the herd ({count}) has left the forest{crops}. {eta} '
              'Bring children and livestock inside. Location: {maps}',
        'hi': '⚠️ सावधान: हाथियों का झुंड ({count}) जंगल से बाहर है{crops_hi}। {eta_hi} '
              'बच्चों और पशुओं को अंदर रखें।',
    },
    'critical': {
        'en': '🚨 DANGER: elephants {dist} m from the village. Stay indoors, do not approach, do not use crackers or '
              'torches on them. Forest team is on the way. Location: {maps}',
        'hi': '🚨 खतरा: हाथी गाँव से {dist} मीटर दूर हैं। घर के अंदर रहें, पास न जाएँ, पटाखे या मशाल न चलाएँ। '
              'वन विभाग की टीम रास्ते में है।',
    },
    'report': {
        'en': '{icon} {species_cap} reported near {place} ({who}, {time}). Keep children and livestock inside, '
              'avoid the area after dark. {drone} Location: {maps}',
        'hi': '{icon} {time} बजे {place} के पास {species_hi} की सूचना ({who_hi})। बच्चों और पशुओं को अंदर रखें, '
              'अंधेरे में उस ओर न जाएँ।',
    },
    'path': {
        'en': '{icon} {who_en} coming toward the village along the {name}: seen at cameras {chain}. {eta_en} '
              'Stay indoors and keep children and livestock inside. The drone is on its way. Location: {maps}',
        'hi': '{icon} {who_hi} गाँव की ओर बढ़ रहे हैं (कैमरे {chain})। {eta_hi} घर के अंदर रहें, बच्चों और पशुओं को अंदर रखें।',
    },
    'turned_back': {
        'en': '✅ {who_en} turned back into the forest (cameras {chain}). No danger for now; the cameras keep watching.',
        'hi': '✅ {who_hi} जंगल की ओर लौट गए। अभी खतरा नहीं; कैमरे नज़र रख रहे हैं।',
    },
    'manual': {
        'en': '📢 Forest department: {note} Location: {maps}',
        'hi': '📢 वन विभाग: {note_hi}',
    },
    'tracking': {
        'en': '🛰️ Drone tracking the herd: {count} elephant{s}{calf}, {heading}. '
              'Village {dist} m away{eta_en}. Live position: {maps}',
    },
}


def eta_text(eta_s):
    """(English, Hindi) sentence on when the herd can reach the village."""
    if eta_s is None:
        return 'It is not heading for the village right now.', 'अभी वह गाँव की ओर नहीं बढ़ रहा।'
    m = max(1, round(eta_s / 60))
    return f'It can reach the village in about {m} min.', f'गाँव तक लगभग {m} मिनट में पहुँच सकता है।'


def compass(deg):
    if deg is None:
        return 'standing'
    return ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'][int((deg + 22.5) % 360 // 45)]


class Notifier:
    def __init__(self, log):
        self.log = log
        self.token = os.environ.get('TELEGRAM_BOT_TOKEN', '').strip()
        default = os.environ.get('TELEGRAM_CHAT_ID', '').strip()
        self.chats = {'officers': os.environ.get('TELEGRAM_CHAT_OFFICERS', default).strip(),
                      'village': os.environ.get('TELEGRAM_CHAT_VILLAGE', default).strip()}
        self.outbox = deque(maxlen=60)
        self.lock = threading.Lock()
        self.last_alert = 0.0
        self.level = 'none'
        self.stage = None
        self.origin = None
        self.seq = 0
        self.path_cams = set()     # cameras on a path: first sightings there go to officers only
        self.path_sent = set()     # (corridor, species) paths the village was told about
        self.path_states = {}

    @property
    def live(self):
        return bool(self.token and any(self.chats.values()))

    # -------------------------------------------------------------- inputs

    def on_event(self, ev):
        """A sentinel event (tower, community report, ingest)."""
        on_path = ev.get('source') in self.path_cams
        if ev.get('species') in OTHER_DANGER and ev.get('source') != 'UAV':
            sp = ev['species']
            fields = dict(icon=OTHER_DANGER[sp][0], species_cap=sp.capitalize(), species_hi=OTHER_DANGER[sp][1],
                          place=f"({ev.get('e', 0):.0f}, {ev.get('n', 0):.0f})" if ev.get('e') is not None else 'the village',
                          who='villager report' if ev.get('source') == 'community' else (ev.get('source_label') or ev.get('source')),
                          who_hi='ग्रामीण की सूचना' if ev.get('source') == 'community' else 'कैमरा',
                          time=time.strftime('%H:%M'), drone=self.drone_note(), maps=self.maps(ev.get('e'), ev.get('n')))
            # On a path, villagers hear when the path is confirmed (on_map).
            self.send(['officers'] if on_path else ['officers', 'village'], 'report', fields)
            return
        if ev.get('species') != 'elephant' or not ev.get('alert') or ev.get('source') == 'UAV':
            return
        now = time.time()
        first = now - self.last_alert > EPISODE_GAP_S
        self.last_alert = now
        if not first:
            return
        count = int(ev.get('count') or 1)
        place = ev.get('source_label') or ev.get('source')
        fields = dict(count=count, s='s' if count != 1 else '', place=place, time=time.strftime('%H:%M'),
                      maps=self.maps(ev.get('e'), ev.get('n')))
        # One sighting on a path camera is not yet a visit: officers only.
        self.send(['officers'] if on_path else ['officers', 'village'], 'sighting', fields, photo=ev.get('snapshot'))

    def on_map(self, data):
        """The mission bridge's map (2 Hz): level changes and the drone getting visual."""
        self.origin = data.get('origin') or self.origin
        w = data.get('wildlife') or {}
        self.path_cams = {cam['id'] for c in w.get('corridors') or [] for cam in c['cameras']}
        for r in w.get('paths') or []:
            key = (r['corridor'], r['species'])
            icon, who_en, who_hi = PATH_WHO.get(r['species'], ('⚠️', 'An animal is', 'जानवर'))
            chain = ' → '.join(r['chain'])
            if r['dispatch'] and key not in self.path_sent:
                self.path_sent.add(key)
                eta = r.get('eta_village_s')
                m = max(1, round(eta / 60)) if eta is not None else None
                self.send(['officers', 'village'], 'path', dict(
                    icon=icon, who_en=who_en, who_hi=who_hi, name=r['name'], chain=chain,
                    eta_en=f'They could reach it in about {m} min.' if m else '', eta_hi=f'लगभग {m} मिनट में पहुँच सकते हैं।' if m else '',
                    maps=self.maps(*r['look'])))
            if r['state'] == 'turned_back' and self.path_states.get(key) != 'turned_back':
                who = ['officers', 'village'] if key in self.path_sent else ['officers']
                self.send(who, 'turned_back', dict(icon=icon, who_en=who_en.replace(' is', '').replace(' are', ''),
                                                   who_hi=who_hi, chain=chain))
            self.path_states[key] = r['state']
        herd = w.get('herd')
        level = w.get('level', 'none')
        rank = ('none', 'watch', 'warning', 'critical')
        if herd and level in ('warning', 'critical') and rank.index(level) > rank.index(self.level):
            eta_en, eta_hi = eta_text(herd.get('eta_s'))
            crops = herd.get('in_crops')
            fields = dict(count=herd['count'], dist=round(herd['village_dist_m']), eta=eta_en, eta_hi=eta_hi,
                          crops=' and is in the crop fields' if crops else '', crops_hi=' और खेतों में है' if crops else '',
                          maps=self.maps(herd['e'], herd['n']))
            self.last_alert = time.time()
            self.send(['officers', 'village'], level, fields)
        self.level = level
        stage = data.get('stage')
        if stage == 'TRACKING' and self.stage != 'TRACKING' and herd:
            eta = herd.get('eta_s')
            fields = dict(count=herd['count'], s='s' if herd['count'] != 1 else '',
                          calf=' incl. a calf' if herd.get('calf') else '',
                          heading=(f"heading {compass(herd['heading_deg'])} at {herd['speed'] * 3.6:.1f} km/h"
                                   if herd.get('speed') and herd.get('heading_deg') is not None else 'standing still'),
                          dist=round(herd['village_dist_m']), eta_en=f', ETA ~{max(1, round(eta / 60))} min' if eta is not None else '',
                          maps=self.maps(herd['e'], herd['n']))
            self.send(['officers'], 'tracking', fields)
        self.stage = stage

    def manual(self, note, herd):
        """The operator's 'Warn village' button."""
        if herd:
            default = (f"{herd['count']} elephant{'s' if herd['count'] != 1 else ''} about {round(herd['village_dist_m'])} m "
                       'from the village. Stay indoors and keep children and livestock inside.')
            default_hi = f"हाथी गाँव से लगभग {round(herd['village_dist_m'])} मीटर दूर हैं। घर के अंदर रहें।"
        else:
            default = 'Wild animals reported near the village. Stay alert and stay indoors after dark.'
            default_hi = 'गाँव के पास जंगली जानवर। सावधान रहें, अंधेरे में घर के अंदर रहें।'
        self.send(['village'], 'manual', dict(note=note or default, note_hi=default_hi,
                                              maps=self.maps(herd['e'], herd['n']) if herd else '(see the forest team)'))

    def drone_note(self):
        busy = self.stage in ('INTERCEPT', 'TRACKING', 'REACQUIRE') and self.level in ('warning', 'critical')
        return ('The drone is tracking elephants near the village; it checks this next. Forest team informed.' if busy
                else 'The drone is going to check. Forest team informed.')

    # ------------------------------------------------------------- sending

    def maps(self, e, n):
        if self.origin is None or e is None:
            return '(position on the dashboard)'
        lat = self.origin[0] + math.degrees(n / EARTH_RADIUS_M)
        lon = self.origin[1] + math.degrees(e / (EARTH_RADIUS_M * math.cos(math.radians(self.origin[0]))))
        return f'https://maps.google.com/?q={lat:.6f},{lon:.6f}'

    def send(self, channels, kind, fields, photo=None):
        en = TEXT[kind]['en'].format(**fields)
        local = TEXT[kind]['hi'].format(**fields) if 'hi' in TEXT[kind] else None
        text = en + (f'\n\n{local}' if local else '')
        for ch in channels:
            with self.lock:
                self.seq += 1
                entry = {'id': self.seq, 't': time.time(), 'channel': ch, 'kind': kind, 'text_en': en,
                         'text_local': local, 'photo': os.path.basename(photo) if photo else None,
                         'delivery': 'sending' if self.live and self.chats.get(ch) else 'simulated'}
                self.outbox.append(entry)
            self.log('alerts', f"[{ch}] {kind}: {en}")
            if entry['delivery'] == 'sending':
                threading.Thread(target=self._telegram, args=(entry, self.chats[ch], text, photo), daemon=True).start()

    def _telegram(self, entry, chat, text, photo):
        try:
            if photo and os.path.isfile(photo):
                self._post_photo(chat, text[:1024], photo)
            else:
                body = json.dumps({'chat_id': chat, 'text': text, 'disable_web_page_preview': True}).encode()
                req = urllib.request.Request(f'https://api.telegram.org/bot{self.token}/sendMessage', data=body,
                                             headers={'Content-Type': 'application/json'})
                urllib.request.urlopen(req, timeout=10).read()
            result = 'telegram'
        except Exception as exc:  # noqa: BLE001 - report any delivery failure on the dashboard
            result = f'failed: {str(exc)[:80]}'
            self.log('alerts', f"Telegram delivery failed: {exc}")
        with self.lock:
            entry['delivery'] = result

    def _post_photo(self, chat, caption, path):
        boundary = uuid.uuid4().hex
        parts = []
        for name, value in (('chat_id', chat), ('caption', caption)):
            parts.append(f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"\r\n\r\n{value}\r\n'.encode())
        with open(path, 'rb') as f:
            img = f.read()
        parts.append(f'--{boundary}\r\nContent-Disposition: form-data; name="photo"; filename="sighting.jpg"\r\n'
                     f'Content-Type: image/jpeg\r\n\r\n'.encode() + img + b'\r\n')
        parts.append(f'--{boundary}--\r\n'.encode())
        req = urllib.request.Request(f'https://api.telegram.org/bot{self.token}/sendPhoto', data=b''.join(parts),
                                     headers={'Content-Type': f'multipart/form-data; boundary={boundary}'})
        urllib.request.urlopen(req, timeout=20).read()

    def snapshot(self):
        with self.lock:
            return {'live': self.live, 'messages': [dict(m) for m in self.outbox]}
