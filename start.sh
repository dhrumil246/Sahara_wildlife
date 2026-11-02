#!/usr/bin/env bash
# One command to bring up the whole SAR demo:
#   dashboard server -> Gazebo + ArduPilot SITL + mission bridge
#   -> live feed (person detector) -> dashboard opened in the browser.
#
#   ./start.sh            post-disaster world (default)
#   ./start.sh runway     flat runway world
#   ./start.sh region     200 x 200 m region (city, forest, village, river),
#                         new random casualty placement; ./start.sh region 42
#                         repeats placement 42. Demo seed: ./start.sh region 3
#                         (all 6 casualties clearly visible to the detector)
#   ./start.sh forest     200 x 200 m forest edge (PS000183): elephant corridor,
#                         camera towers T1-T3, crop fields, village. The herd
#                         waits in the forest until POST /api/wildlife/scenario
#                         {"command": "start herd"} (or Settings -> Start herd).
#   ./start.sh forest night herd nogui   ... in night (thermal) mode, start
#                         the herd by itself once the drone is ready (also
#                         "tusker": turns back; "leopard": east path), and no
#                         Gazebo window (smoother video on a hot laptop; the
#                         cameras still work). Every word is optional.
#   ./start.sh real [device] [baud]   REAL DRONE via the telemetry radio
#                         (default /dev/ttyUSB0 57600): no Gazebo; Mission
#                         Planner/QGC can watch on UDP 14551. Settings ->
#                         Mission type -> Hop test for the first flights.
#                         RESQ_PAYLOAD_SERVO=9:1900 for a release servo.
#   RESQ_RVIZ=1 ./start.sh   also open RViz (off by default)
#
# Then press "Start Search & Rescue" in the dashboard. Ctrl+C here stops
# everything (sim, bridge, detector, server). Any previous run still going
# (another start.sh, or processes it left behind) is stopped first.
set -u

WORLD="${1:-disaster}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
URL="http://localhost:5000"

case "$WORLD" in
  disaster|runway|region|forest) ;;
  real)
    # Real drone: telemetry radio on the laptop, no Gazebo.
    export RESQ_REAL=1
    export RESQ_LINK="${2:-/dev/ttyUSB0}"
    export RESQ_BAUD="${3:-57600}"
    if [ ! -e "$RESQ_LINK" ]; then
      echo "No device at $RESQ_LINK. Plug in the telemetry radio (ls /dev/ttyUSB* /dev/ttyACM*)," >&2
      echo "or give its path: ./start.sh real /dev/ttyACM0 57600" >&2
      exit 1
    fi
    ;;
  *) echo "usage: $0 [disaster|runway|region [seed]|forest [night] [herd] [tusker] [leopard] [nogui] | real [device] [baud]]" >&2; exit 1 ;;
esac
SEED="${2:-}"   # region world: casualty placement seed (blank = random)
[ "$WORLD" = real ] && SEED=""
# forest world: optional words "night" and "herd" (in any order)
NIGHT=0; HERD=0; ANIMALS=""
if [ "$WORLD" = forest ]; then
  SEED=""
  for w in "${@:2}"; do
    case "$w" in
      night) NIGHT=1 ;;
      herd|tusker|leopard) ANIMALS="$ANIMALS $w"; HERD=1 ;;
      nogui) export RESQ_GZ_GUI=0 ;;
      *) echo "forest world: unknown option '$w' (expected night, herd, tusker, leopard and/or nogui)" >&2; exit 1 ;;
    esac
  done
fi

# ---- Stop anything left from a previous run -------------------------------
# Same process patterns as server.py's KILL_PATTERNS, plus the dashboard
# server and older start.sh instances. Specific on purpose: a bare "gz" or
# "ardupilot" would also kill e.g. an editor with an ArduPilot file open.
STACK_PATTERNS=(
  "ros2 launch ardupilot_gz_bringup" "gz sim" "arducopter" "rviz2"
  "robot_state_publisher" "parameter_bridge" "micro_ros_agent" "mavproxy.py"
  "resq_mavlink_bridge" "person_detector.py" "heat_follower.py" "animal_mover.py" "command_center/backend/server.py"
)

stack_running() {
  for p in "${STACK_PATTERNS[@]}"; do
    pgrep -f "$p" > /dev/null && return 0
  done
  return 1
}

# Only real start.sh runs (bash executing it), not e.g. an editor with it open.
OLD_STARTS=$(pgrep -f "^(/usr)?(/bin/)?bash (.*/)?start\.sh" | grep -vx "$$" || true)
if [ -n "$OLD_STARTS" ] || stack_running || curl -s -o /dev/null "$URL"; then
  echo "Stopping the previous run..."
  # Politely first: an old start.sh runs its own cleanup on SIGINT, and the
  # dashboard stops the sim's process groups cleanly.
  [ -n "$OLD_STARTS" ] && kill -INT $OLD_STARTS 2> /dev/null
  curl -s -m 10 -X POST "$URL/api/sim/stop" > /dev/null 2>&1
  for _ in $(seq 1 10); do
    stack_running || break
    sleep 1
  done
  # Then force whatever is left.
  [ -n "$OLD_STARTS" ] && kill -9 $OLD_STARTS 2> /dev/null
  for p in "${STACK_PATTERNS[@]}"; do
    pkill -9 -f "$p" 2> /dev/null
  done
  for _ in $(seq 1 10); do
    curl -s -o /dev/null "$URL" || break
    sleep 1
  done
  if curl -s -o /dev/null "$URL"; then
    echo "Port 5000 is still in use by something else; stop it and try again." >&2
    exit 1
  fi
  echo "Previous run stopped."
fi

python3 "$ROOT/command_center/backend/server.py" &
SERVER=$!

cleanup() {
  trap - INT TERM EXIT
  echo
  echo "Stopping simulation, bridge and detector..."
  curl -s -X POST "$URL/api/sim/stop" > /dev/null
  # SIGINT, not SIGTERM: rclpy's handler catches SIGTERM and only stops
  # the server's ROS thread, leaving Flask running.
  kill -INT "$SERVER" 2> /dev/null
  for _ in 1 2 3 4 5; do
    kill -0 "$SERVER" 2> /dev/null || break
    sleep 1
  done
  kill -9 "$SERVER" 2> /dev/null
  wait "$SERVER" 2> /dev/null
  echo "Stopped."
}
trap cleanup INT TERM EXIT

echo "Waiting for the dashboard server..."
until curl -s -o /dev/null "$URL"; do
  if ! kill -0 "$SERVER" 2> /dev/null; then
    echo "Dashboard server exited during startup (see the errors above)." >&2
    exit 1
  fi
  sleep 1
done

echo "Launching the simulation ($WORLD world)..."
curl -s -X POST -H "Content-Type: application/json" -d "{\"world\": \"$WORLD\", \"seed\": \"$SEED\"}" "$URL/api/sim/launch"
echo
if [ "$WORLD" = forest ]; then
  # In the background: the lights can only change once Gazebo has loaded
  # (the server re-applies night as soon as it can), which took ~1 min.
  [ "$NIGHT" = 1 ] && { curl -s -m 180 -X POST -H "Content-Type: application/json" -d '{"mode": "night"}' "$URL/api/mode" > /dev/null & }
  echo "Starting Sentinel (camera towers + drone camera)..."
  curl -s -X POST "$URL/api/feed/start"
  echo
elif [ "$WORLD" != real ] || [ "${RESQ_CAMERA:-0}" = 1 ]; then
  echo "Starting the live feed (person detector)..."
  curl -s -X POST "$URL/api/feed/start"
  echo
fi

xdg-open "$URL" > /dev/null 2>&1 || true
echo
echo "Dashboard: $URL  (Gazebo opens in its own window; allow ~30 s)"
if [ "$WORLD" = forest ]; then
  LAN=$(hostname -I 2> /dev/null | awk '{print $1}')
  echo
  echo "  Camera wall (drone + 3 towers):  $URL/cams"
  echo "  Drone camera only:               $URL/video_feed"
  echo "  Tower T1 / T2 / T3:              $URL/video_feed/tower/t1   .../t2   .../t3"
  echo "  Map view:                        $URL/?view=map"
  echo "  Villagers' page (phone):         $URL/community"
  [ -n "$LAN" ] && echo "  From a phone/laptop on the same Wi-Fi, use http://$LAN:5000 instead of $URL"
  echo
  if [ "$HERD" = 1 ]; then
    # Wait for the drone's GPS fix (the bridge's map has an origin), then
    # give Sentinel's model a moment to load before the herd walks out.
    ( for _ in $(seq 1 180); do
        curl -s "$URL/api/map/latest" | grep -q '"origin"' && break
        sleep 2
      done
      sleep 25
      first=1; GAP=40
      for g in $ANIMALS; do
        [ "$first" = 1 ] || sleep "$GAP"   # one story at a time
        first=0
        # The tusker turns back about 100 s after it starts; anything after
        # it waits until it is gone (two groups on one trail at once can't be
        # told apart by the camera chain).
        # (300 s of wall time: the sim runs slower than real time on a hot laptop.)
        GAP=40; [ "$g" = tusker ] && GAP=300
        curl -s -X POST -H "Content-Type: application/json" -d "{\"command\": \"start $g\"}" "$URL/api/wildlife/scenario" > /dev/null \
          && echo "Started: $g. The drone goes only once its path toward the village is confirmed."
      done ) &
    echo "Starting$ANIMALS by itself once the drone is ready (about 1-2 min)."
  else
    echo "Start animals in Settings (gear) -> Demo animals. Auto-send is on."
  fi
else
  echo "Press Start Search & Rescue there."
fi
echo "Ctrl+C here stops everything."
wait "$SERVER"
