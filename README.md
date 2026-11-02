# SAHARA Wildlife

**Early warning for human–wildlife conflict.** Built for problem statement PS000183, *AI-Powered Solutions for Mitigating Human-Wildlife Conflict*.

Forests in India already have camera towers and camera traps. Their weakness is that a fixed camera sees one spot: once an elephant herd or a leopard walks out of view, it's lost, and every first sighting either triggers a panic or gets ignored. SAHARA connects those cameras into **paths**, works out whether an animal is actually heading for a village, and only then sends a drone to find it, follow it and warn people in time.

🎬 **Demo film (3:28):** _link coming soon_

## How it works

1. **Cameras along the paths animals use.** Trail cameras inside the forest and towers at the forest edge, in two chains toward the village:
   - Elephant trail: **C1 → C2 → T2 → T4 → village**
   - East nullah (leopard route): **L1 → T1 → T5 → village**
2. **Sentinel detector** (`ardu_ws/sentinel_detector.py`). YOLOv8 by day, thermal cameras at night. It measures each animal's size on the ground, so it can tell an elephant or a leopard from cattle and dogs. Cattle and dogs never raise an alert.
3. **Path logic** (`ros2_ws/src/resq_mavlink_bridge/resq_mavlink_bridge/wildlife.py`). The order in which cameras see an animal gives its path:
   - **Seen once:** watch it, don't launch.
   - **Seen in order toward the village:** approaching, with its speed and when the next camera should see it.
   - **Reaches the launch camera** (T2 or T1): path confirmed, so the **drone launches by itself** and the village is warned.
   - **Seen again further back:** turned back. No launch, and officers get an all-clear.
   - **The next camera stays quiet for too long:** it has stopped or left the path.
4. **Drone mission** (`mavlink_bridge.py`). An S500 running ArduCopter, with a 45° gimbal camera and thermal camera:
   - It flies to where the herd is *going*, finds it, and follows from 25 m behind without descending.
   - It keeps estimating when the herd will reach the village, raising the alert from watch to warning to critical.
   - A leopard report from a villager is queued while the drone is busy with elephants, then checked afterwards.
5. **Alerts.** Officers and villagers get messages in English and Hindi through Telegram, or simulated if no bot token is set. There's also a villagers' page for reporting sightings from a phone.

## Run

Requires Ubuntu 22.04, ROS 2 Humble and Gazebo Harmonic (see setup below).

```bash
./start.sh forest tusker herd nogui     # a tusker turns back (no launch), then a herd comes through (launch)
./start.sh forest leopard nogui         # a leopard on the east path
./start.sh forest night herd            # at night, on thermal cameras, with the Gazebo window
```

`start.sh` prints the links when it's ready:

| Page | URL |
|---|---|
| Console: live cameras, situation, drone, map, alerts | http://localhost:5000 |
| All cameras on one screen | http://localhost:5000/cams |
| Villagers' report page (open it on a phone on the same Wi-Fi) | http://&lt;laptop-ip&gt;:5000/community |

To start animals while the sim is running, use **Settings → Demo animals**, or run:

```bash
curl -X POST localhost:5000/api/wildlife/scenario -H 'Content-Type: application/json' -d '{"command":"start leopard"}'
```

For real Telegram alerts, start with `TELEGRAM_BOT_TOKEN=... TELEGRAM_CHAT_ID=... ./start.sh forest`.

The original search-and-rescue demo still works: `./start.sh` for the post-disaster world, or `./start.sh region 3` (see [command_center/README.md](command_center/README.md)).

## Layout

- `start.sh`: one command that launches the whole stack.
- `worldgen/`
  - `generate_forest.py`: builds the forest world (camera positions, paths, village, fields).
  - `animal_mover.py`: walks the scripted animals.
- `ardu_ws/`: ArduPilot and Gazebo workspace.
  - `sentinel_detector.py`: the camera and drone-gimbal animal detector.
  - `src/ardupilot_gazebo/models/`: the S500 with its gimbal, plus the animal models. Credits are in `ANIMALS_ATTRIBUTION.md`.
  - `src/ardupilot_gz/`: launch files and ROS–Gazebo bridge configs.
  - `deps.repos`: pinned upstream repos that aren't committed here (ArduPilot, ros_gz, …).
- `ros2_ws/src/resq_mavlink_bridge/`: mission logic, with the herd tracker, path tracker and risk map in `wildlife.py`.
- `command_center/`: Flask backend (`server.py`, `alerts.py`) and the plain HTML/JS console (`live.html`, `cams.html`, `community.html`).
- `resq_mavlink/`: mission files, including `mission_forest.json`.
- `tools/`: adapters that feed existing systems into the same event stream:
  - `ingest_alert.py`: TrailGuard-style alert messages.
  - `ingest_cameratrap.py`: camera-trap image folders.
- `demo/`: the film pipeline.
  - `film.py`: cuts the film.
  - `voiceover.py`: narration through edge-tts.
  - `sentinel_footage.py`: runs the detector on real footage.
- `CONTEXT.md`: detailed engineering notes, including what was measured and the known limits.

## Setting up from a fresh clone

```bash
cd ardu_ws/src
vcs import --recursive < ../deps.repos
(cd Micro-XRCE-DDS-Gen && git apply ../../patches/Micro-XRCE-DDS-Gen-gradle.patch)
cd .. && colcon build            # ArduPilot SITL, ros_gz, ardupilot_gz, ...
cd ../ros2_ws && colcon build
pip3 install --user -r ../command_center/backend/requirements.txt
pip3 install --user ultralytics "numpy<2"
```

`command_center/backend/ros_env.sh` sources both workspaces and sets `GZ_SIM_RESOURCE_PATH` for everything the console launches. Keep numpy below 2: `cv_bridge` in ROS 2 Humble needs numpy 1.x.

## Honest limits

- In the simulation, the animals walk scripted paths, and the risk layer uses sample data.
- The detector is COCO-trained YOLOv8, which has no leopard class. Leopards are identified by their measured size, so a sloth bear of similar size could be confused with one. A real deployment needs a model trained on Indian camera-trap images.
- Cameras alone can't tell apart two groups walking the same path at the same time.
- The real S500 flights in the film were flown manually. The autonomous mission shown is simulated.
