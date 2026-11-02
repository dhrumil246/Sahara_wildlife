# SIH 2026 — Autonomous SAR Drone: Project Context

The single source of truth for this project: what we are building, how every part works, what is already done (and measured), and what is still planned. For run instructions and deeper technical notes per feature, see `README.md` and `command_center/README.md`.

**Status legend**
- ✅ Done and tested (numbers below are measured, not estimated)
- 🟡 Done, but not yet tested on the real drone, or only partly done
- 🔲 Planned, not started

Last updated: 27 Sep 2026. Git branch: `feature/intelligent-search`. **Nothing on this branch is committed yet.**

---

## 1. Problem statement and how we answer it

> *A deployable AI-powered autonomous drone that aids search-and-rescue operations by detecting people and hazards, improving responder safety and reducing victim discovery time.*

| PS requirement | How we do it | Status |
|---|---|---|
| **Autonomous navigation** (GPS, obstacle avoidance) | ArduPilot GUIDED mode driven by our mission bridge. Bayesian search planner. Forward lidar + downward rangefinder obstacle guard. Mapped building heights (pre-disaster intel). Guarded return home. | ✅ in sim · 🔲 real hardware |
| GPS-denied navigation / SLAM | Not built | 🔲 |
| **On-device AI inference** | YOLOv8m (RGB) + YOLOv8n (thermal, trained on HIT-UAV) + a thermal-signature detector, all running locally. No cloud. | ✅ on laptop GPU · 🔲 on the drone (Pi 5 + Hailo planned) |
| **Multi-sensor fusion** (RGB, thermal, IMU, GPS) | Every RGB "person" is checked against the boresighted thermal frame: body temperature, size, skin temperature, no fire pixels. Detections are geo-projected with the drone's GPS + attitude at the frame's capture time. Flood = cold in thermal **and** flat in RGB. | ✅ in sim |
| **Hazard classification** (fire, smoke, flood, debris, unstable structures) | Fire / hot spots and flood water from thermal (+ RGB). Collapsed structures from mission intel. River vs flood from the known-water map. | ✅ fire, flood, structures · 🔲 smoke, landslides, chemical leaks, electrical lines |
| **Geo-tagged mapping** | Live situational map: probability heatmap, flight trail, casualties, hazard zones with safety buffers, A* safe ground routes, pre-disaster basemap | ✅ |
| **Emergency alerting + prioritised rescue** | Alert feed with severity and chime. Casualties ranked by nearby threats (fire/flood/structure), movement and route time. Recommended action per casualty. | ✅ |
| **Situational reports** | Printable SITREP page (save as PDF), generated from the drone's own data | ✅ |
| **Offline resilience** | Everything runs locally (no internet, no cloud, no external fonts/CDNs). The dashboard only needs the local link. | ✅ design · 🔲 comms relay / store-and-forward |
| **Command-center dashboard** | Web dashboard: live AI feed, thermal + split view, live map, drone status, forward-lidar profile, rescue priorities, alerts, logs, SITREP | ✅ |

---

## 2. System architecture

### 2.1 Simulation (working now)

```
 ┌──────────────── Gazebo Harmonic (world: runway / disaster / region) ───────────────┐
 │  S500 quad model: RGB cam (640x480, 10 Hz), thermal cam (L16 radiometric, 10 Hz),  │
 │  forward 3D lidar (48x16, 40 m), downward rangefinder, IMU, GPS, payload box       │
 │  Actors (casualties, 5 poses) + heat bodies · dogs · fires · flood · buildings     │
 └───────────┬──────────────────────────────────────┬──────────────────────────────────┘
             │ JSON sensor/servo (lockstep)          │ ros_gz_bridge (images, lidar, clock)
     ArduPilot SITL (ArduCopter 4.8)                 ▼
             │ MAVLink (MAVProxy → UDP 14550)   ROS 2 Humble topics
             ▼                                        │
   mavlink_bridge (ROS 2 node) ◄──── /detections/* ── person_detector.py (YOLO + fusion + hazards)
   mission state machine, planner,                    ▲
   obstacle guard, hazard map, routes                 │ /camera/image, /thermal/image
             │ /resq/map (JSON 2 Hz), services        
             ▼
   command_center/backend/server.py (Flask + rclpy) ──► browser dashboard (http://localhost:5000)
```

### 2.2 Real drone (target)

```
 Pixhawk 2.4.8 (ArduCopter, Pixhawk1-1M build)
   ├─ RC receiver ─── your RC transmitter (always the override)
   ├─ GPS/compass, power module (6200 mAh), 4x A2212 + 8" props on S500 frame
   ├─ TELEM1 ── SiK telemetry radio ~~~ radio on laptop USB ── MAVProxy ── bridge / Mission Planner / preflight
   └─ TELEM2 ── (phase 2) Raspberry Pi 5 ── RGB camera (+ thermal later) ── detector (Hailo-8L)
                                              └─ WiFi ── laptop dashboard
```

- **Phase 1 (now):** everything runs on the laptop, with MAVLink over the telemetry radio. First test: the **hop test**.
- **Phase 2:** bridge + detector on the Pi 5 (on-device AI). The laptop only shows the dashboard.

---

## 3. Components in detail

### 3.1 Simulation stack ✅

| Piece | What / where | Notes |
|---|---|---|
| Gazebo Harmonic | `ardu_ws/src/ardupilot_gz/` (launch, worlds), `ardu_ws/src/ardupilot_gazebo/` (models, ArduPilot plugin) | Launched by `s500_quad_runway.launch.py world:=<name>`. RViz off by default (`RESQ_RVIZ=1` to open it). |
| ArduPilot SITL | ArduCopter 4.8.0, JSON interface, lockstep | Same firmware family as the real Pixhawk |
| ROS 2 Humble | `ros2_ws/src/resq_mavlink_bridge` | Two workspaces; `command_center/backend/ros_env.sh` sources both |
| S500 model | `ardu_ws/src/ardupilot_gazebo/models/s500_quad/model.sdf` | Nadir RGB + thermal (boresighted, HFOV 1.2 rad, 640×480, **10 Hz**; 20 Hz made the Gazebo window stall). Forward lidar + downward rangefinder. Detachable payload. |
| GPU rendering | `ros_env.sh` forces NVIDIA EGL/GLX | Headless rendering had fallen back to the CPU (`dri2` warning) |
| ROS bridge config | `ardupilot_gz_bringup/config/s500_bridge.yaml` | Images, lidar points, rangefinder, clock, IMU, GPS, battery |

**Performance (measured, full mission):**
- Small world: 0.92× real time with the Gazebo window. GPU 36% mean / 63% peak.
- Region world: 1.00× headless (16% GPU); ~0.96× with the Gazebo window (46% GPU, a fixed cost of the window itself).

### 3.2 Worlds

| World | Size | Content | Status |
|---|---|---|---|
| `runway` | flat | 1 casualty, simple test | ✅ |
| `disaster` | 40×40 m search area | Earthquake aftermath. Collapsed buildings ring the area; rubble, cars, poles inside. **2 casualties**, 2 dogs (thermal distractors), smouldering debris (fire), flood pool, smoke plumes, staging area. | ✅ |
| `region` | **200×200 m** | Generated. City (NE), forest (NW), village (SW), river + flood (SE), launch pad at the centre. 6 seeded casualties in 5 poses. | ✅ |
| `forest` | 200×200 m | **PS000183 (human-wildlife conflict).** Generated by `worldgen/generate_forest.py`. Dense forest + elephant trail (N), forest edge with waterhole, camera towers T1–T3 (RGB 1280×720 + thermal, on `/tower/t<n>/image` and `/tower/t<n>/thermal`), crop fields, village (S), forest station at the origin. Scripted animals (CC BY models, `models/ANIMALS_ATTRIBUTION.md`) walked by `worldgen/animal_mover.py`: a herd of 3 and a lone tusker wait in the forest until `POST /api/wildlife/scenario {"command": "start herd"}`; decoy cattle graze at T1. COCO YOLOv8m on T2's frames: elephants 0.60–0.96. | 🟡 world done; Sentinel detector + wildlife mission next |

**Region world generator:** `worldgen/generate_region.py`
- The whole landscape is **one static model of merged meshes**: 167 trees in one OBJ, all procedural buildings in another, ground/roads/fields in a third. About 6,500 triangles total. Generated tileable textures.
- No collision except the ground (keeps physics cheap).
- Fuel "hero" models, render-checked in Harmonic: apartment, office, post office, 3× Indian house, 2× collapsed house, bus, cars, truss bridge. Fuel House 1/2/3 render black and are not used.
- **Casualties are seeded**: `./start.sh region <seed>`. They are placed only on open ground the camera can see (rubble, street, lane, field, clearing, riverbank). Poses come from a separate random stream (lying 35%, crawling 15%, sitting 20%, waving 15%, walking 15%). Movers pace a clear path.
- Heat bodies per pose (skin 309 K, clothing 305 K). `worldgen/heat_follower.py` moves the heat of walking/crawling casualties with their actors, from the sim clock, at 10 Hz.
- It also writes the mission intel `resq_mavlink/mission_region.json`, containing only pre-disaster information:
  - building footprints + heights (`known_obstacles`)
  - the river outline (`known_water`)
  - a land-use prior raster (`prior_region.npy`)
  - the basemap PNG
  - the staging point

  Ground truth goes to `region_truth.json` (scoring only; the drone never reads it). Mover routes go to `region_actors.json`.
- **Demo seed: 3.** All 6 casualties are clearly visible to the detector (seeds 2–7 had 4–5 of 6).

### 3.3 Perception — `ardu_ws/person_detector.py` ✅ (sim)

**Person detection**
- **Day:** COCO YOLOv8m on RGB, run on the frame + 90°/180°/270° rotations (people lying from above face any way). Confidence ≥ 0.6.
- **Night:** YOLOv8n trained on HIT-UAV (drone thermal) on white-hot 283–313 K frames. Confidence ≥ 0.45.
- **Day also runs the thermal model and the thermal-signature detector**, merged with RGB.
- **Thermal-signature detector:** a person-sized warm region with a skin-temperature peak (≥ 308 K) and no fire pixels. It is the only route to **sitting** people.

**Fusion checks on every box, using the thermal frame:**
1. Body temperature: 97th percentile 303–316 K.
2. Fire rejection: the warm region must not reach 323 K (smouldering ash is 305 K but touches 330–360 K embers).
3. Size / skin: a warm region ≥ 1.1 m = lying or crawling. A compact one (0.35–1.1 m) counts only if its peak is ≥ 308 K (exposed skin). The dog's fur is at most 307 K and rejects as an animal. *This is a 2 K margin in simulation; it needs validating with real thermal footage.*

**Detection by pose, from above at 8 m:**

| Pose | Found by |
|---|---|
| Lying | RGB 0.92 + thermal model + signature |
| Crawling | RGB 0.91 + thermal model + signature |
| Walking | thermal model + signature |
| Waving | thermal model + signature |
| Sitting | signature only |
| Dog | rejected |

**Hazards (1 Hz scan of every thermal frame, day and night)**
- Fire / hot spot: > 323 K.
- Flood: colder than 283.5 K and larger than 1.5 m², plus RGB "flat and textureless" by day.
- Published on `/detections/hazards` (JSON with image offsets).

**Performance**
- GPU, capped at **5 analyses/s in FP16** (`quantize=16`): 0.05 s per analysis, which leaves the GPU for Gazebo.
- Live frames show the latest boxes (no freeze-frame when inference is fast).
- Output is JPEG `/detections/debug_image/compressed`. Raw ROS images cost ~93 ms per frame in Python, which had capped the feed at 3.4 fps.

### 3.4 Mission bridge — `ros2_ws/src/resq_mavlink_bridge/resq_mavlink_bridge/mavlink_bridge.py` ✅ (sim)

**Mission state machine**

`STARTING → ARMING → TAKEOFF → CLIMBING → SEARCHING ⇄ VERIFYING → MARKING → … → RETURNING → LANDING → COMPLETE`

Plus `HOVERING` (hop test), `ABORTED` (crash / arming failure) and `PILOT` (override).

**Mission types**
- `search` (default): full SAR mission.
- `hop` (hop test): take off to 20 m (configurable, 3–40 m), hover 10 s, then LAND straight down. **No horizontal command is ever sent.** Horizontal drift in sim: ≤ 1 cm.

**Search planners**
- `bayes` (default): the probability-map planner (§3.5). Flies to the most probability per second of flight. Replans every 1 s with 25% hysteresis.
- **Carrot:** it aims 2 s of flight past the target so ArduPilot doesn't brake at every hop. Average speed went from 1.47 → ~3 m/s at a commanded 5 m/s.
- `grid`: lawnmower, for comparison.
- **Livelock guard:** 30 s at one target with no progress writes that area off.

**Verification**
- Stop, fly over the candidate, hover.
- Confirm when ≥ 8 frames are analysed with a person in ≥ 20% of them (fusion has already removed animals and shadows).
- Fast-reject when < 8% of 25+ frames.
- Tracks the latest 3 sightings (for movers). Duplicate check at confirmation.
- Drone-shadow rejection by sun geometry (day).

**On confirm**
- `continue`: drop the aid kit 2.5 m beside the first casualty, mark later ones, keep searching.
- `land`: land beside the casualty.

**Triage:** a casualty whose sightings drift > 2.5 m is flagged **moving (responsive)** and ranked below still ones. *Only walkers are detectable; crawlers are too slow to separate from ~1 m jitter.*

**Hazard map:** hazard sightings are projected with the pose at capture time and merged (fire 8 m, flood 5 m). A zone is confirmed after 2 sightings. Flood detections inside the known river are ignored.

**Obstacle safety (lessons from 2 crashes in the region city)**
- **Obstacle guard** (`obstacle_guard.py`): levels the lidar points with roll/pitch and checks a 5 m corridor ahead (look-ahead 10–35 m, growing with speed).
  - Climbs to top + 4 m.
  - **Blocked** only if the needed climb can't be done in the time to reach the obstacle (2 m/s climb).
  - Tall walls beyond the lidar's view block within stopping distance.
  - Holds altitude 8 s. Won't descend onto roofs (rangefinder).
- **Mapped obstacles:** flies at building height + 4 m along the next 25 m of path, and climbs in place first when close.
- **Caution:** max 2 m/s and no carrot while an obstacle still needs climbing.
- **Brake-back hold:** the hold point is 1 s of travel *behind* the drone.
- **Crash / in-flight disarm detection:** critical alert, mission `ABORTED`.
- **Guarded return home** (instead of RTL at a fixed height).

**Pilot override** ✅ tested in sim: if the flight mode changes away from what the bridge set (e.g. the RC switch to LOITER), the bridge stops commanding immediately, shows **PILOT OVERRIDE**, and never switches back.

**Other safety:** arming timeout 20 s. The Pixhawk's STATUSTEXT (PreArm, failsafe, EKF…) is forwarded to the dashboard alerts.

**Aid drop:** Gazebo detach in sim. On the real drone, a servo via `RESQ_PAYLOAD_SERVO="<output>:<pwm>"` (e.g. `9:1900` = AUX1). There is also a `/resq/payload/drop` service.

**Configuration:**
- `mission.json` / `mission_region.json`: area, altitude, speed, planner, on-confirm, POS target, time budget, priors, hazards, obstacles, water, basemap.
- `RESQ_MISSION_FILE` overrides the mission file.
- Dashboard overrides via the latched `/resq/mission/config`.

### 3.5 Planning library — `search_map.py` ✅

- **ProbabilityMap** (Bayesian search theory): 2 m cells.
  - The prior comes from Gaussian zones or a raster (the region's land-use prior).
  - Each analysed frame multiplies the probability of the cells in the central 80% of the footprint by (1 − POD). POD is 0.7 RGB / 0.6 thermal, scaled by (8/alt)² above 8 m, and a look counts only once per second.
  - Gains use a small convolution (scales to 200 m; a dense matrix would need ~680 MB).
  - Probability of success = prior mass searched.
- **HazardMap:** merge distances per type; a zone is confirmed at 2 sightings; intel zones are pre-confirmed.
- **safe_route:** A* around hazard buffers (fire 4 m, flood 1.5 m, structure 3 m). Grid 1 m (coarser for big areas).
- **assess_victims:** threat score (fire < 15 m, flood < 10 m, structure < 8 m), moving −10, then route ETA at 1 m/s on foot → priority P1…Pn + recommended action.

### 3.6 Command center ✅

- **Backend** `command_center/backend/server.py` (Flask + rclpy):
  - Process manager (sim, bridge, feed, heat follower, drone link).
  - REST API + SSE: status, logs, map.
  - MJPEG streams: AI feed, raw thermal.
  - World generation per seed.
  - Day/night lighting via Gazebo services.
  - Real-drone mode (`RESQ_REAL=1`).
  - `RESQ_PORT` for a second instance.
- **Dashboard** `command_center/frontend/` (plain HTML/JS/CSS, no build step, works offline):
  - **Home:** AI feed with AI / THERMAL / SPLIT (draggable diagonal) views, HUD (alt/speed/heading/stage), compass, LIVE badge, alert toasts with chime, full screen, reticle. Mini map with zoom/follow. Drone status (altitude, speed, battery, satellites). Forward-lidar side profile. Environment and camera pickers. Mission strip (stage, probability of success, casualty and hazard chips). Action bar.
  - **Map:** full map (basemap, heatmap, trail, footprint, target, candidates, rejected, casualties with priority badges, hazards with buffers, safe routes, staging), rescue-priority cards, hazard list, alert log.
  - **Telemetry:** every value + process status + console.
  - **Settings:** world, casualty seed, mission type (search / hop test), hop altitude, planner, on-confirm.
  - **Log drawer** (Logs button or `L`): Events + Console with source filters and search.
  - **SITREP** (`/report`): KPIs, map, priorities table with coordinates, hazards, mission log; print to PDF.
  - Real-drone mode: *Connect Drone*, sim-only settings hidden.
- **`start.sh`:**
  - `./start.sh [disaster|runway|region [seed]|real [device] [baud]]`
  - Stops any previous run first (other `start.sh`, leftover processes, port 5000).
  - Real mode does not start the detector unless `RESQ_CAMERA=1`.

### 3.7 Tools ✅

| Tool | Purpose |
|---|---|
| `resq_mavlink/search_benchmark.py` | Monte Carlo: probability planner vs lawnmower, 40 m area |
| `resq_mavlink/region_benchmark.py` | Same over the region world, using the generator's real seeded casualties |
| `resq_mavlink/preflight.py` | Real-drone pre-flight check, props off (§4.4) |
| `worldgen/generate_region.py`, `worldgen/heat_follower.py` | Region world generation, moving heat |

---

## 4. Real drone integration

### 4.1 Hardware (as known)

| Item | Status |
|---|---|
| Frame | S500 quad, A2212 motors, 8" props, 6200 mAh battery |
| Flight controller | Pixhawk 2.4.8 (clone): flash the **ArduCopter "Pixhawk1-1M"** build (1 MB flash limit) |
| RC | Transmitter + receiver: **always the override** (keep LOITER + STABILIZE on the mode switch) |
| Telemetry | SiK radio pair (USB on the laptop) |
| Companion computer | **Raspberry Pi 5, running Pi OS** |
| Cameras | RGB camera. **No thermal camera yet.** |
| Lidar / rangefinder | None yet (optional) |

### 4.2 Phases

| Phase | Goal | Status |
|---|---|---|
| 0 | Real-drone software path (link mode, hop test, pilot override, preflight, servo drop) | ✅ built and tested against the simulated Pixhawk through the real-mode path |
| 1a | Bench, props OFF: `./start.sh real`, `preflight.py` all PASS, dashboard telemetry, arming, servo | 🔲 needs the drone |
| 1b | **Hop test**, open field, you on the RC: 20 m up, hover, land in place | 🔲 |
| 1c | Camera on the laptop: stream RGB from the Pi over WiFi; detector on the laptop; detection check by carrying the camera at ~8 m over a person lying on grass | 🔲 |
| 2 | On-device AI: bridge + detector on the Pi 5. ROS 2 via Docker (Pi OS has no native Humble) or Ubuntu 24.04. Detector: YOLOv8n (CPU) or the **Raspberry Pi AI Kit (Hailo-8L)** | 🔲 |
| 3 | Small autonomous search (20×20 m, volunteer / mannequin, empty field, obstacle guard off) | 🔲 |
| 4 | Thermal camera (radiometric: InfiRay P2 Pro / Topdon TC001 / FLIR Lepton 3.5 + PureThermal): enables night mode and the fusion checks | 🔲 |
| 5 | Optional forward lidar / rangefinder (TFmini-class downward for AGL first) | 🔲 |

### 4.3 How real mode works (Phase 0 ✅)

1. `./start.sh real` (defaults `/dev/ttyUSB0 57600`), or `./start.sh real /dev/ttyACM0 57600`.
2. **Connect Drone** starts MAVProxy on the radio, split to UDP **14550** (bridge, same as sim), **14551** (Mission Planner / QGroundControl as a safety monitor; in QGC add a UDP link on 14551), **14552** (preflight check).
3. Settings → **Mission type: Hop test**, altitude 20 → **Start Hop Test**.
4. Flip the RC mode switch at any time: the bridge stops (PILOT OVERRIDE).

### 4.4 Pre-flight checklist (`python3 resq_mavlink/preflight.py`)

It checks the link, ArduPilot firmware, disarmed state, GPS (3D fix, ≥ 8 satellites, HDOP < 1.5), sensor health, EKF, battery (V/cell), RC receiver, geofence, RTL altitude, RC-loss failsafe, battery failsafe, arming checks, a takeover mode on the switch, and PreArm messages.

**Parameters to set in Mission Planner before flying (the simulated Pixhawk's defaults failed these):**
- `FENCE_ENABLE=1`, `FENCE_TYPE=3`, `FENCE_ALT_MAX=30`, `FENCE_RADIUS=50`, `FENCE_ACTION=1`
- `BATT_MONITOR=4`, `BATT_LOW_VOLT≈3.6×cells`, `BATT_FS_LOW_ACT=2`
- `FS_THR_ENABLE=1`
- RTL altitude 10–30 m

### 4.5 Still to build for the real drone 🔲

- A shared **camera config file** (HFOV, resolution, mount) used by both the bridge and the detector. Today these are constants matching the sim camera.
- A Pi camera publisher (`/camera/image`) and ROS 2 on the Pi (Docker).
- The detector on the Pi (YOLOv8n export / Hailo) and a re-measurement of accuracy.
- A time sync (chrony) between Pi and laptop if frames are stamped on the Pi.
- A thermal camera adapter (each vendor's radiometric format → Kelvin).
- **Legal (India, Drone Rules 2021):** ~1.5 kg = small category. Register on Digital Sky (UIN), fly in green zones below 400 ft. Check current rules.

---

## 5. Measured results

| What | Result |
|---|---|
| Disaster world, full mission (GPU) | Both casualties found (T+19 s, T+34 s, ≤ 0.5 m error); fire and flood mapped; no false positives after fusion; mission 153 s search + landing |
| Dogs vs people | Before fusion: 2 dogs confirmed as casualties. After: 0 (120 RGB detections rejected as "likely an animal") |
| Planner benchmark, 40 m area | Probability map vs lawnmower median time-to-find: **51% faster** (intel-driven placement), 19% faster (random placement) |
| Region benchmark, 16 seeds × 6 casualties | Land-use map planner at 5 m/s found 43/67/81% by 5/10/15 min; lawnmower 31/57/62% |
| **Region, demo seed 3, full 25 min mission** | **All 6 found** (sitting, sitting, crawling, crawling in rubble, waving in a street, lying in a forest clearing); 5 of 6 in 7 min; errors 0.2–1.4 m; no false positives, no crash |
| Feed | 3.4 → ~6 fps AI feed at 10 Hz cameras (16 fps at 20 Hz headless); detector 0.05 s/analysis |
| Sim speed | Small world 0.92× with the Gazebo window; region 1.00× headless, ~0.96× with the window |
| Hop test (sim, and real-mode path) | 20 m in ~12 s, 10 s hover, LAND, disarmed; ≤ 1 cm drift |
| Pilot override (sim) | Detected ~2.5 s after the mode switch; bridge stopped; mode never forced back |

---

## 6. Known limitations (be honest with judges)

- **Nadir-only camera:** upright people are small from above. Sitting people are found only by thermal signature. A tilted gimbal would help.
- **Skin-temperature rule** (person vs dog) relies on a 2 K sim margin; unvalidated on real footage.
- **The "moving" triage flag** works for walkers only (crawlers are within projection jitter).
- **City streets between tall blocks** are searched from above the buildings (lower detection per look); the price of not crashing.
- **The forward-only lidar** can't see sideways or backwards. Mapped building heights cover known buildings; unmapped tall obstacles are the residual risk.
- **Benchmarks are optimistic:** they assume constant speed; real flights brake, climb and verify (region: benchmark ~80% by 15 min vs 2–5 of 6 in real sim runs before the latest fixes).
- **No GPS-denied navigation / SLAM, no comms relay** yet.
- **Sim-only pieces** (world generator, heat follower, day/night lighting) don't exist on the real drone.

---

## 7. Backlog — ideas discussed, not started 🔲

Roughly in priority order:
1. **Real drone phases 1–3** (§4.2): bench → hop test → camera → on-device AI → small search.
2. **Thermal camera** on the real drone (unlocks night mode and fusion).
3. **Video for the SIH submission:** cold open on a thermal lock, real drone → digital twin match cut, split-screen mission, "it says no" rejections (shadow, dog, cone), mid-mission day/night switch, payload drop, SITREP finale. Remotion for titles and captions.
4. **Remote triage** (challenge and response): hover, play a voice prompt (Hindi + local language), classify the response with pose estimation → START tags.
5. **Finding people cameras can't see:** an ESP32 sniffing phone WiFi probe / BLE RSSI; locate buried victims by multilateration; fuse with thermal/RGB.
6. **Comms resilience:** show the comms radius, store-and-forward ("data mule") return, or a second drone as a relay (`multiagent.launch.py` exists).
7. **More seeds / benchmark in the real sim** (headless batch runs), and tuning the region search speed/altitude further.
8. Smoke / landslide / electrical-line hazard classifiers.
9. GPS-denied navigation (visual odometry / SLAM).
10. A gimbal / oblique camera for upright people.

---

## 8. How to run

```bash
./start.sh                  # disaster world (40 m, 2 casualties)
./start.sh region 3         # 200 m region, demo seed 3 (6 casualties, 5 poses)
./start.sh region           # region, random placement
./start.sh forest           # PS000183 forest edge; then POST /api/wildlife/scenario {"command": "start herd"}
./start.sh real             # real drone via telemetry radio (/dev/ttyUSB0 57600)
RESQ_RVIZ=1 ./start.sh      # also open RViz
python3 resq_mavlink/preflight.py                 # with ./start.sh real running, props OFF
python3 resq_mavlink/search_benchmark.py          # planner benchmark (40 m)
python3 resq_mavlink/region_benchmark.py          # planner benchmark (region)
python3 worldgen/generate_region.py --scenery     # rebuild region scenery + intel
```

Dashboard: http://localhost:5000. Press **Start Search & Rescue** (or **Start Hop Test**). Ctrl+C in the terminal stops everything.

GPU note: PyTorch is the CUDA build (`torch 2.11.0+cu128`). Keep **numpy 1.26.4**, because numpy 2 breaks ROS Humble's `cv_bridge`. Never `pip install --force-reinstall` without `"numpy<2"`.

---

## 9. File map

| Path | Role |
|---|---|
| `start.sh` | One-command launcher, cleanup of previous runs, real mode |
| `command_center/backend/server.py`, `ros_env.sh` | Dashboard backend, process manager, ROS env (NVIDIA rendering) |
| `command_center/frontend/index.html`, `app.js`, `style.css` | Dashboard |
| `command_center/frontend/map.js` | Map renderer (dashboard + SITREP) |
| `command_center/frontend/report.html` | SITREP |
| `command_center/frontend/basemaps/region.png` | Pre-disaster basemap |
| `command_center/README.md` | Detailed technical notes and measurements per feature |
| `ros2_ws/src/resq_mavlink_bridge/resq_mavlink_bridge/mavlink_bridge.py` | Mission bridge (state machine, safety, map publishing) |
| `…/search_map.py` | Probability map, hazard map, A*, victim priorities |
| `…/obstacle_guard.py` | Lidar obstacle guard |
| `ardu_ws/person_detector.py` | Detector: YOLO RGB + thermal, fusion, signature, hazards |
| `ardu_ws/yolov8m.pt`, `ardu_ws/thermal_best.pt` | Models (COCO; HIT-UAV thermal) |
| `ardu_ws/src/ardupilot_gazebo/models/s500_quad/model.sdf` | Drone model and sensors |
| `ardu_ws/src/ardupilot_gazebo/models/region_scenery/` | Generated region landscape |
| `ardu_ws/src/ardupilot_gz/ardupilot_gz_gazebo/worlds/` | `runway.sdf`, `disaster.sdf`, `region.sdf` (generated) |
| `ardu_ws/src/ardupilot_gz/ardupilot_gz_bringup/config/s500_bridge.yaml` | Gazebo ↔ ROS topics |
| `worldgen/generate_region.py`, `heat_follower.py` | Region generator, moving heat |
| `resq_mavlink/mission.json` | Disaster-world mission + intel |
| `resq_mavlink/mission_region.json`, `prior_region.npy` | Region mission intel (generated) |
| `resq_mavlink/region_truth.json`, `region_actors.json` | Ground truth (scoring only), mover routes |
| `resq_mavlink/preflight.py` | Real-drone pre-flight check |
| `resq_mavlink/search_benchmark.py`, `region_benchmark.py` | Planner benchmarks |

---

## 10. PS000183 — AI for human-wildlife conflict (forest world)

Same stack, a second problem statement: **camera towers spot an elephant herd, the drone flies out by itself to where the herd will be and follows it past the towers' view, and villagers are warned in time.** Fixed AI cameras already exist in India (Coimbatore and Gudalur thermal towers, TrailGuard AI); their gap is that a fixed camera sees ~150 m and then loses the herd. SAR mode is untouched: everything below switches on with the `forest` world.

### 10.1 Pieces

| Piece | Where | What it does |
|---|---|---|
| Forest world | `worldgen/generate_forest.py` | 200×200 m: forest + elephant trail (N), forest edge + waterhole, towers T1–T3 (RGB 1280×720 + thermal, `/tower/t<n>/{image,thermal}`), crop fields, village (S), forest station = launch pad. Writes `mission_forest.json` (towers, village, fields, wildlife + escalation settings), `risk_forest.npy` (static risk, illustrative weights), `forest_animals.json` (routes). |
| Animals | `models/elephant_*`, `models/cow_*` (CC BY, `models/ANIMALS_ATTRIBUTION.md`); `worldgen/animal_mover.py` | Real-looking Sketchfab models (via Objaverse), real size, thermal 302–304 K. Scripted: herd of 3 (incl. calf) and a lone tusker wait in the forest until `start herd` / `start tusker`; decoy cattle graze at T1. |
| Drone camera | `s500_quad/model.sdf` `gimbal_link` | RGB + thermal looking forward 45° down (`/s500/gimbal/*`). Not in the ROS bridge yaml, so it renders only when Sentinel subscribes. From straight above COCO calls an elephant a "bird"; from 45° it scores 0.76–0.94. |
| Sentinel | `ardu_ws/sentinel_detector.py` | Towers + drone, straight from gz-transport. Day: COCO YOLOv8m. Night: warm regions by size (height and area). Every box projected to the ground and measured in metres: elephant only if the class says so (day) or ≥ 1.9 m tall (night); cattle-sized = cattle; herd memory across cameras for half-hidden members. Only elephants alert. Out: `/sentinel/detections`, `/sentinel/events` (+ snapshot), annotated feeds. |
| Wildlife logic | `resq_mavlink_bridge/wildlife.py` | `HerdTracker` (position, count, heading, speed from all sources; predicted path; village ETA; level watch → warning → critical), `RiskMap` (static + sightings with 30 min half-life), standoff point, forest-exit point, oblique footprint. |
| Wildlife mission | `mavlink_bridge.py` (`mission_type: wildlife`) | **Auto-dispatch** on an elephant alert → take off → above 15 m **INTERCEPT** (aims where the herd will be; if it is still under the canopy, waits at its predicted exit point on the forest edge) → **TRACKING** 25 m behind at 30 m, nose and camera on the herd (MAVLink position + yaw), never descends → **REACQUIRE** (circles the predicted position) → hand-over and return on battery / time. |
| Alerts | `command_center/backend/alerts.py` | Officers + village channels, English + Hindi, on first sighting, warning, critical, drone visual. Telegram Bot API if `TELEGRAM_BOT_TOKEN` (+ `TELEGRAM_CHAT_ID`, or per channel) is set, with the snapshot; otherwise kept as "simulated" and shown in the dashboard's phone panel. |
| Dashboard | `command_center/frontend/` | Wildlife mode for the forest world: Sentinel tower tiles, herd card, threat level, alerts sent; Map view with risk layer, village, towers' view, herd track, predicted path (+1 min ticks), standoff, villager pins, sightings with snapshots, phone panel. Auto-dispatch toggle; scenario buttons in Settings. |
| Villagers' page | `/community` (`community.html`) | Phone-first, English/Hindi: live alert banner, do's and don'ts, report a sighting (tap the map or GPS). Reports join the same event stream. |
| Existing systems | `tools/ingest_alert.py`, `tools/ingest_cameratrap.py` → `POST /api/events/ingest` | TrailGuard-style alert text / e-mail / JSON with a camera registry (`tools/cameras.example.csv`); camera-trap photo folders (YOLO at 1280 px, EXIF time, one sighting per camera and hour). |
| Demo tooling | `demo/wildlife_scenario.py`, `demo/sentinel_footage.py`, `demo/wildlife.html` + `demo/wildlife_edit.py` | Repeatable scripted take with a timeline and a score; real-footage inset with the same detector; the film's overlay and build (captions keyed to the take's timeline). |

### 10.2 Run

```bash
./start.sh forest [night] [herd] [nogui]           # wildlife console at http://localhost:5000
# Auto-dispatch is on by default (mission_forest.json); then in Settings: "Start herd (T2)", or
curl -X POST localhost:5000/api/wildlife/scenario -H 'Content-Type: application/json' -d '{"command":"start herd"}'
python3 demo/wildlife_scenario.py --launch --mode night --report-at 260   # the whole take, scored
TELEGRAM_BOT_TOKEN=... TELEGRAM_CHAT_ID=... ./start.sh forest              # real Telegram alerts
```
Villagers' page: `http://<laptop-ip>:5000/community` on a phone on the same network.

### 10.3 Measured (forest world, sim)

| What | Result |
|---|---|
| Sentinel, day, T1 + T2 (~1,000 detections) | labels 100% right (cattle never "dog"/"elephant"); ground position error median 0.2–0.5 m, p90 ≤ 0.8 m; herd heading 160° / 1.2 m/s vs true 156° / 1.1 m/s |
| Sentinel, night (thermal only) | T3 median 0.4 m; T2 sizes the herd right (2.7–2.9 m) at 53–67 m; beyond 75 m warm regions stay unclassified. Elephant = ≥ 1.9 m tall **and** ≥ 3.5 m² warm area (measured: elephants 4.4 m² head-on to 8 m² side-on, one cow 1.0–2.1 m²); this removed false elephants from two cattle merged side by side or one behind the other |
| Herd tracker (scripted route) | position within ~1 m; levels watch (forest) → warning (leaves the forest, 90 s out) → critical (45 s out / 40 m) |
| **Full night take** (`wildlife_scenario.py --mode night`, take 4) | T2 alert → auto-dispatch at 39 s; drone waits at the predicted forest exit (-45, 31) (true exit (-46, 30)); herd in view at 58 s; warning 84 s; critical 131 s; tracked 6 min, handed over, landed. Herd position vs truth: median 1.6 m, p90 3.2 m. 7 alerts (simulated delivery). No crash, no false elephant. |
| Imported alert (`tools/ingest_alert.py`) | placed at the registered camera, on the map, and (auto-dispatch armed) launched the drone |

### 10.5 Known limitations (wildlife)

- **Canopy:** no camera sees through dense leaves. Towers watch the forest edge; the drone waits at the herd's predicted exit point and works in the open. AOS (airborne optical sectioning) is future work.
- **One herd at a time:** while a herd is being followed, elephants > 60 m from it are other groups: they alert and mark the risk map but are not tracked.
- **Night sizing** uses flat ground and a calibrated camera pose; on slopes or with an uncalibrated real camera the metres are off. Calves and half-hidden adults are named by herd memory (within 10 m of an elephant seen in the last 8 s), which could also pull a cow standing next to a herd into it.
- **Scripted animals**, rigid models (no leg animation). Risk weights are sample data. Alert thresholds are tightened for a 200 m world.
- **Telegram** needs mobile data where the laptop is; SMS/IVR gateways are the field option (paid per message).
- Hindi alert text and the villagers' page need checking by a native speaker; other languages (Kannada, Tamil, Odia…) are template additions.

### 10.3a Console, drone tasks, performance (Nov 2026 update)

**Wildlife console** (`frontend/live.html` + `live.css` + `live.js`, served at `/` when the forest world runs; the old dashboard is at `/classic`). Light theme, one plain-language status line (All clear → Elephants under watch → Warning → Danger), three screens with 2–3 actions each:
- **Live**: one big video (Drone / T1 / T2 / T3 thumbnails switch it), Situation and Drone cards. Actions: *Send drone* / *Bring drone back* (`POST /api/mission/return` → bridge `/resq/mission/return`), *Warn village* (`POST /api/wildlife/broadcast`), small *Emergency: land now*.
- **Map**: map + reports list. Action: *Add a report* (tap the map).
- **Alerts**: messages sent + what happened. Action: *Warn village*.
- Settings (gear): simulation, day/night, auto-send, demo animals, Telegram status, links (`/cams`, `/community`, `/classic`).
Only the big player streams MJPEG; thumbnails are `/snapshot/<cam>` every 1.5 s; the map redraws at 4 fps.

**Drone tasks** (bridge, `CHECK` stage). Leopard/bear reports (villagers, imports, towers; COCO "cat" ≥ 0.5 m = leopard) and elephant reports far from the followed herd become tasks:
- drone idle → it flies out, circles the spot 60 s with the camera on it, reports what it saw, returns;
- drone tracking a herd at **warning/critical** → it stays on the elephants; the report is **queued** (shown on the console), officers and villagers are messaged at once, and the drone checks it **after the hand-over** before going home;
- herd still in the forest (watch) → the drone **diverts** to the report (towers keep watching the herd);
- an elephant alert during a check → the check goes back to the front of the queue and the drone intercepts.

**Performance** (measured 2026-11-01). The laptop CPU sits at **94 °C and throttles to 1.3–1.9 GHz** even on charger + performance profile (EPP stays `balance_performance`), so the sim ran at 0.36–0.47× real time and the camera feeds at ~2–3 fps (sensor rates are in sim time). The GPU had headroom (36–60 %). Changes: Sentinel subscribes only the active mode's cameras (day RGB / night thermal) and shrinks frames before converting; the forest world's ROS bridge (`s500_bridge_forest.yaml`) leaves out the nadir cameras; towers and the gimbal camera at 10 Hz; `./start.sh forest nogui` runs without the Gazebo window (~2 cores); the new console replaces a dashboard that redrew two canvases at 60 fps. Result: sim 0.65× and **5.8 fps on every feed** without the window (was 2–3). Not possible: 500 Hz physics (ArduCopter won't arm below 720 Hz IMU). Remaining limit is heat: a cooling pad / raised rear / vendor fan mode, and closing old dashboard tabs (each launch opened one; they keep streaming).

### 10.3b Camera chains: launch on the path, not on a first sighting

Cameras sit in chains along the paths animals take toward the village (`CORRIDORS` in `generate_forest.py`, `corridors` in `mission_forest.json`):

| Path | Cameras (forest → village) | Drone goes at |
|---|---|---|
| Elephant trail (north-west) | C1 trail cam, deep forest → C2 trail cam, mid forest → T2 forest exit → T4 village edge west | T2 |
| East nullah | L1 trail cam, forest → T1 grazing land → T5 village edge east | T1 |

Trail cams are camera traps on 3 m posts beside the path under the canopy (960×540 at 5 Hz); towers are 6 m masts (1280×720 at 10 Hz). `wildlife.PathTracker` reads each chain per species: **spotted** (one camera: watch, don't launch), **approaching** (cameras in order toward the village; speed from first-sighting to first-sighting, next camera and when it should see them, village ETA), **turned back** (seen again further up the path), **stopped** (the next camera stays quiet for 2× the expected walk: stopped or left the path), **at village** (last camera). The drone launches when the animals reach the `dispatch_at` camera after being seen further up (or first seen there or later); elephants → intercept, leopard/bear → a check halfway to the next camera. Cameras off any path (T3), villagers' reports and imported alerts keep the old rule (send on the report). `dispatch_rule: "first_sight"` in the mission file switches back. Alerts follow: a first sighting on a path camera goes to officers only; the village is told when the path is confirmed ("coming toward the village… seen at C1 → C2 → T2"), and an all-clear if the animals turn back.

Verified live (`./start.sh forest tusker herd nogui`, 2026-11-01): tusker spotted at C1 → approaching C1 → C2 → **turned back, no launch** (officers told "turned back", villagers not disturbed); 5 min later the herd: approaching → **path confirmed at T2 → drone launched**, village told "coming toward the village… C1 → C2 → T2", then the warning; drone found the herd (3 incl. calf) and tracked it. Leopard alone: L1 → T1 confirmed → drone sent to check halfway to T5. Lessons built in: a camera only counts for its own 10 m path spot (T5 once claimed the herd 80 m beyond; T2 claimed it at C2), a visit needs 2 agreeing analyses, daytime elephants must be ≥ 1.7 m tall (one frame of "elephant" on cattle at T1 once launched the drone), no "turned back" after a confirmed path (a tusker behind the herd produced a false all-clear), and two groups on one path can't be told apart by cameras alone (the demo starts the tusker 300 s before the herd).

Scenario (Settings → Demo animals, or `./start.sh forest tusker herd leopard`): the **tusker** walks C1 → C2, stops and goes back (turned back, no launch); the **herd** walks the whole trail (launch at T2); the **leopard** (CC BY model, 0.7 m, `models/leopard`) comes down the east nullah past the cattle at T1 to the east village edge (launch / queued check at T1). Replayed through the tracker: herd C1@13 s → C2@35 s (1.11 m/s measured, 1.1 true) → T2@44 s → T4@165 s; tusker C1 → C2 → C1 = turned back; leopard L1@21 s → T1@42 s → T5@113 s.

**Leopards:** COCO has no leopard class; on the sim's leopard YOLOv8m says "dog" at 18–28 m and "bear"/"cat" up close. Sentinel calls a cat/dog/bear-labelled animal 0.45–0.9 m tall and ≥ 1.15 m long a leopard, and the same size rule applies to warm regions at night (cattle measured 0.95–1.46 m tall in thermal, village dogs < 0.9 m long). A sloth bear is similar in size: a real deployment needs a camera-trap model trained on Indian species.

### 10.4 The demo film (≤ 5 min, target ~4:30)

Storyline: one herd, one night, one continuous story. The edit is `demo/wildlife.html` (segments, captions, cards); `demo/wildlife_edit.py` builds it.

| # | Segment | Source |
|---|---|---|
| — | Intro card | — |
| 01 | Night thermal at T2: the herd leaves the forest, alert | night take (Sentinel tiles) |
| — | The problem: deaths, electrocutions, towers exist but lose the herd (sourced figures) | card |
| 02 | Day: cattle at T1 are measured and get "no alert" | day take |
| 03 | Same detector on real elephant footage | `demo/sentinel_footage.py` output |
| 04 | No one presses a button: auto-dispatch | night take (drone feed) |
| 05 | Intercept: waits at the predicted forest exit | night take (Map view) |
| 06 | Tracking 25 m behind; warning; ETA | night take (drone feed) |
| 07 | Real phone gets the Telegram alert (EN + HI) | phone screen recording |
| 08 | One picture: critical, villager report, risk map | night take (Map view) |
| 09 | The same S500 flies at night (manual, campus) | your night campus clip |
| — | Built on what exists + cost; honest limits; outro | cards |

Recording checklist:
1. `powerprofilesctl set performance`, charger in, close heavy apps (0.38× real time on `balanced`).
2. OBS 1920×1080 @ 60, screen split as the SAR demo: Gazebo window left half, dashboard (Chrome, `http://localhost:5000/?view=…`) right half.
3. Night take: `python3 demo/wildlife_scenario.py --launch --mode night --report-at 230`; note the recording second when it prints `scenario` (the offset). Day take for segment 02 the same way with `--mode day`.
4. Telegram: create a bot (@BotFather), put `TELEGRAM_BOT_TOKEN` / `TELEGRAM_CHAT_ID` in the environment before `./start.sh forest`; screen-record the phone during the night take.
5. Real footage: `python3 demo/sentinel_footage.py clip.mp4 -o demo/out/real_footage.mp4 --credit "..."` (footage you may use, credited).
6. `python3 demo/wildlife_edit.py prepare --timeline demo/out/wildlife_timeline_<t>.jsonl --file night=… --offset night=… …`, then `stills` to check crops (adjust `SRC` in `wildlife.html` if the window layout differs), then `build`. It prints the length and warns over 5:00.
7. Needs `pip install --user playwright imageio-ffmpeg` (with numpy kept at 1.26.4) and system Chrome.
