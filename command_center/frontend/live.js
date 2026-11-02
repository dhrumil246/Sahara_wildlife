// Wildlife console. Streams: /api/status (telemetry, processes) and /api/map
// (the bridge's picture: herd, drone, sightings, events). Only the big
// player holds a live MJPEG stream; thumbnails are snapshots every 1.5 s and
// the map redraws 4 times a second, so the browser stays light next to
// Gazebo on the same laptop.
const $ = (id) => document.getElementById(id);
const $$ = (sel) => document.querySelectorAll(sel);
let status = null;
let map = null;
let tab = "live";
let cam = "drone";
let msgsSeen = 0;
let msgCount = 0;

const CAMS = { drone: ["Drone", "/video_feed"], t1: ["T1 · East edge", "/video_feed/tower/t1"],
  t2: ["T2 · Corridor exit", "/video_feed/tower/t2"], t3: ["T3 · Waterhole", "/video_feed/tower/t3"],
  c1: ["C1 · Trail, deep forest", "/video_feed/tower/c1"], c2: ["C2 · Trail, mid forest", "/video_feed/tower/c2"],
  t4: ["T4 · Village edge west", "/video_feed/tower/t4"], l1: ["L1 · East nullah", "/video_feed/tower/l1"],
  t5: ["T5 · Village edge east", "/video_feed/tower/t5"] };
const IDLE = ["IDLE", "COMPLETE", "ABORTED", "PILOT"];
const DRONE_LINE = {
  IDLE: "On the ground, ready.", STARTING: "Taking off…", ARMING: "Taking off…", TAKEOFF: "Taking off…",
  CLIMBING: "Climbing above the trees…", INTERCEPT: "Flying to meet the herd.", TRACKING: "Following the herd from 25 m behind.",
  REACQUIRE: "Looking for the herd again.", CHECK: "Checking a report.", RETURNING: "Coming back to the station.",
  LANDING: "Landing.", COMPLETE: "Landed. Ready.", ABORTED: "Stopped. Check the drone.", PILOT: "The pilot has control.",
};
const COMPASS = ["north", "north-east", "east", "south-east", "south", "south-west", "west", "north-west"];

// ---------------------------------------------------------------- helpers

async function post(path, body) {
  try {
    const res = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body || {}) });
    const r = await res.json();
    toast(r.message || (r.ok ? "Done." : "Failed."));
    return r;
  } catch (e) {
    toast("No connection to the command center.");
    return null;
  }
}
let toastTimer = null;
function toast(text) {
  const t = $("toast");
  t.textContent = text;
  t.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => (t.hidden = true), 3500);
}
const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
const mmss = (s) => `${Math.floor(s / 60)}:${String(Math.round(s % 60)).padStart(2, "0")}`;
function setText(id, text) { const el = $(id); if (el.textContent !== text) el.textContent = text; }
function setHtml(el, html) { if (el.dataset.h !== html) { el.innerHTML = html; el.dataset.h = html; } }

// ------------------------------------------------------------------ tabs

function showTab(name) {
  tab = name;
  $$(".tab").forEach((b) => b.classList.toggle("active", b.dataset.tab === name));
  $$(".screen").forEach((s) => s.classList.toggle("active", s.dataset.screen === name));
  // Only the Live screen holds a video stream.
  const want = name === "live" ? CAMS[cam][1] : "";
  if (($("player").getAttribute("src") || "") !== want) {
    if (want) $("player").setAttribute("src", want); else $("player").removeAttribute("src");
  }
  if (name === "alerts") { msgsSeen = msgCount; setText("alerts-count", ""); }
  try { localStorage.setItem("tab", name); } catch (e) { /* private mode */ }
}
$$(".tab").forEach((b) => (b.onclick = () => showTab(b.dataset.tab)));

function showCam(c) {
  cam = c;
  setText("player-label", CAMS[c][0]);
  $$(".thumb").forEach((t) => t.classList.toggle("active", t.dataset.cam === c));
  if (tab === "live") $("player").setAttribute("src", CAMS[c][1]);
}
$$(".thumb").forEach((t) => (t.onclick = () => showCam(t.dataset.cam)));

// One thumbnail per camera, path cameras first (from the bridge's map).
let thumbsBuilt = false;
function buildThumbs(w) {
  if (thumbsBuilt || !w.towers || !w.towers.length) return;
  thumbsBuilt = true;
  const onPath = (w.corridors || []).flatMap((c) => c.cameras.map((x) => x.id));
  const order = [...w.towers].sort((a, b) => (onPath.includes(a.id) ? onPath.indexOf(a.id) : 99) - (onPath.includes(b.id) ? onPath.indexOf(b.id) : 99));
  for (const t of order) {
    const id = t.id.toLowerCase();
    CAMS[id] = [`${t.id} · ${t.label}`, `/video_feed/tower/${id}`];
    const b = document.createElement("button");
    b.className = "thumb";
    b.dataset.cam = id;
    b.innerHTML = `<img alt=""><span>${esc(t.id)} · ${esc(t.label)}</span>`;
    b.onclick = () => showCam(id);
    $("thumbs").appendChild(b);
  }
}

function refreshThumbs() {
  if (tab === "live" && status && status.feed_running) {
    const now = Date.now();
    $$(".thumb").forEach((t) => (t.querySelector("img").src = `/snapshot/${t.dataset.cam}?t=${now}`));
  }
  setTimeout(refreshThumbs, 1500);
}

// --------------------------------------------------------------- actions

$("btn-drone").onclick = () => {
  const flying = status && !IDLE.includes(status.mission_stage);
  if (flying) post("/api/mission/return");
  else post("/api/mission/start", { mission_type: "wildlife" });
};
async function warnVillage() {
  if (!confirm("Send a warning to the village now?")) return;
  post("/api/wildlife/broadcast", {});
}
$("btn-warn").onclick = warnVillage;
$("btn-warn-2").onclick = warnVillage;
$("btn-land").onclick = () => { if (confirm("Land the drone where it is now?")) post("/api/mission/land"); };

// Map: add a report by tapping
let placing = null;
$("btn-report").onclick = () => {
  $("report-form").hidden = false;
  $("btn-report").hidden = true;
  $("map").classList.add("placing");
  $("map-hint").hidden = false;
  placing = {};
};
function endPlacing() {
  placing = null;
  $("report-form").hidden = true;
  $("btn-report").hidden = false;
  $("map").classList.remove("placing");
  $("map-hint").hidden = true;
  $("r-send").disabled = true;
  setText("r-where", "Tap the map to place it.");
}
$("r-cancel").onclick = endPlacing;
$("map").addEventListener("click", (ev) => {
  const v = $("map")._view;
  if (!placing || !v) return;
  const r = $("map").getBoundingClientRect();
  const [e, n] = v.toEN(ev.clientX - r.left, ev.clientY - r.top);
  placing = { e: +e.toFixed(1), n: +n.toFixed(1) };
  setText("r-where", `Placed at ${Math.round(e)} m E, ${Math.round(n)} m N.`);
  $("r-send").disabled = false;
});
$("report-form").onsubmit = async (ev) => {
  ev.preventDefault();
  if (!placing || placing.e == null) return;
  const r = await post("/api/community/report", { species: $("r-species").value, count: +$("r-count").value || 1,
    note: $("r-note").value, source_label: "Forest officer", ...placing });
  if (r && r.ok) endPlacing();
};

// Settings
$("open-settings").onclick = () => ($("settings").hidden = false);
$("close-settings").onclick = () => ($("settings").hidden = true);
$("settings").onclick = (e) => { if (e.target.id === "settings") $("settings").hidden = true; };
$("s-sim").onclick = () => (status && status.sim_running ? post("/api/sim/stop") : post("/api/sim/launch", { world: "forest" })
  .then(() => post("/api/feed/start")));
$$(".seg button").forEach((b) => (b.onclick = () => post("/api/mode", { mode: b.dataset.mode })));
$("s-auto").onchange = (e) => post("/api/wildlife/config", { auto_dispatch: e.target.checked });
$$("[data-scenario]").forEach((b) => (b.onclick = () => post("/api/wildlife/scenario", { command: b.dataset.scenario })));

// ---------------------------------------------------------------- render

function herdWords(h) {
  const n = `${h.count} elephant${h.count !== 1 ? "s" : ""}${h.calf ? " with a calf" : ""}`;
  const where = h.in_forest ? "in the forest" : h.in_crops ? "in the crop fields" : "in the open";
  return `${n} ${where}`;
}

function renderStatus() {
  const w = (map && map.wildlife) || {};
  const h = w.herd;
  const level = w.level || "none";
  $("status").className = `status ${level}`;
  let head = "All clear";
  let sub = "Cameras are watching the forest edge.";
  if (!status || !status.sim_running) { head = "Offline"; sub = "Start the simulation in Settings."; }
  else if (h && level === "watch") { head = "Elephants under watch"; sub = `${herdWords(h)}, ${Math.round(h.village_dist_m)} m from the village.`; }
  else if (h && level === "warning") { head = "Warning"; sub = `${herdWords(h)}${h.eta_s != null ? ` · could reach the village in about ${mmss(h.eta_s)}` : `, ${Math.round(h.village_dist_m)} m from the village`}.`; }
  else if (h && level === "critical") { head = "Danger"; sub = `Elephants ${Math.round(h.village_dist_m)} m from the village. Villagers have been told to stay indoors.`; }
  setText("status-text", head);
  setText("status-sub", sub);
  document.title = level === "critical" ? "⚠ Danger · SAHARA" : "SAHARA Wildlife";
}

function renderLive() {
  const w = (map && map.wildlife) || {};
  const h = w.herd;
  if (h) {
    setText("herd-line", `${herdWords(h).replace(/^./, (c) => c.toUpperCase())}.`);
    setText("f-heading", h.heading_deg != null ? `${COMPASS[Math.round(h.heading_deg / 45) % 8]} · ${(h.speed * 3.6).toFixed(1)} km/h` : "Standing still");
    setText("f-dist", `${Math.round(h.village_dist_m)} m`);
    setText("f-eta", h.village_dist_m === 0 ? "In the village" : h.eta_s != null ? `about ${mmss(h.eta_s)}` : "Not heading there");
    const fresh = Object.entries(h.sources || {}).filter(([, a]) => a < 15).map(([s]) => (s === "UAV" ? "Drone" : s === "community" ? "Villager" : s === "ingest" ? "Other camera" : s));
    setText("f-src", fresh.length ? fresh.join(", ") : `${mmss(h.age_s)} ago`);
  } else {
    setText("herd-line", "No elephants seen.");
    ["f-heading", "f-dist", "f-eta", "f-src"].forEach((id) => setText(id, "—"));
  }
  // Drone
  const stage = (status && status.mission_stage) || "IDLE";
  let line = DRONE_LINE[stage] || stage;
  const task = w.current_task;
  if (stage === "CHECK" && task) line = `Checking a ${task.species} report from ${task.label}.`;
  setText("drone-line", line);
  const d = map && map.drone;
  setText("drone-sub", status && status.bridge_running && d
    ? `Battery ${d.battery >= 0 ? d.battery + "%" : "—"} · ${d.alt.toFixed(0)} m up · auto-send ${w.auto_dispatch ? "on" : "off"}` : "");
  const q = (w.tasks || []).map((t) => `<li>Next: ${esc(t.species)} report · ${esc(t.label)}</li>`).join("");
  setHtml($("queue"), q);
  // Buttons
  const btn = $("btn-drone");
  const flying = !IDLE.includes(stage);
  btn.textContent = flying ? "Bring drone back" : "Send drone";
  btn.classList.toggle("recall", flying);
  btn.disabled = !(status && status.bridge_running);
  $("btn-land").hidden = !flying;
  // Thumbnails light up on a fresh alert
  const now = Date.now() / 1000;
  $$(".thumb").forEach((t) => {
    const id = t.dataset.cam === "drone" ? "UAV" : t.dataset.cam.toUpperCase();
    t.classList.toggle("alert", (w.sightings || []).some((s) => s.source === id && s.alert && now - s.t < 12) || (id === "UAV" && w.uav_visual));
  });
}

const PATH_TEXT = {
  spotted: (r) => `Seen once at ${r.furthest}. Watching: one sighting does not send the drone.`,
  approaching: (r) => `Coming toward the village${r.speed_ms ? ` at ${(r.speed_ms * 3.6).toFixed(1)} km/h` : ""}.${r.next ? ` ${r.next} should see it in about ${r.next_in_s} s.` : ""}${r.dispatch ? " Drone sent." : ""}`,
  turned_back: (r) => `Turned back (${r.chain.join(" → ")}). No launch needed.`,
  stopped: () => "Not seen at the next camera when expected: stopped or left the path. No launch.",
  at_village: () => "At the village edge.",
};
const SPECIES_ICON = { elephant: "🐘", leopard: "🐆", bear: "🐻" };

function renderPaths() {
  const w = (map && map.wildlife) || {};
  setHtml($("paths"), (w.paths || []).map((r) => {
    const seenIdx = r.cameras.map((c) => c.seen);
    const chips = r.cameras.map((c) => `<span class="cam ${c.seen ? "seen" : ""} ${c.seen && r.dispatch ? "go" : ""} ${c.id === r.next ? "next" : ""}" title="${c.seen ? `seen ${mmss(c.last_s)} ago` : "not seen"}">${esc(c.id)}</span>`);
    return `<div><div class="path-name">${SPECIES_ICON[r.species] || ""} ${esc(r.name)}</div>
      <div class="chain">${chips.join('<span class="arrow">→</span>')}<span class="arrow">→</span><span class="home" title="village">🏠</span></div>
      <div class="path-text ${r.state}">${esc((PATH_TEXT[r.state] || (() => r.state))(r))}</div></div>`;
  }).join(""));
}

function renderSightings() {
  const w = (map && map.wildlife) || {};
  // Latest sighting per camera and species (towers report every few seconds),
  // and every report from people or other systems.
  const seen = new Set();
  const list = [...(w.sightings || [])].reverse().filter((s) => {
    if (s.source === "community" || s.source === "ingest") return true;
    const key = `${s.source}/${s.species}`;
    if (!s.alert || seen.has(key)) return false;
    seen.add(key);
    return true;
  }).slice(0, 12);
  const done = (w.done_tasks || []).slice().reverse().map((t) => `<li class="plain"><b>Checked: ${esc(t.species)} report</b><small>Drone saw ${esc(t.seen)}</small></li>`);
  setHtml($("sightings"), (done.join("") + list.map((s) => {
    const who = s.source === "community" ? (s.source_label || "Villager") : s.source === "ingest" ? (s.source_label || "Other camera") : s.source === "UAV" ? "Drone" : (s.source_label || s.source);
    const img = s.snapshot ? `<img src="/api/wildlife/snapshot/${encodeURIComponent(s.snapshot)}" alt="">` : `<span class="noimg">📍</span>`;
    return `<li>${img}<span><b class="${s.alert ? "alert" : ""}">${s.count} ${esc(s.species)}${s.count !== 1 && !["cattle", "wild boar"].includes(s.species) ? "s" : ""}</b><small>${esc(who)} · ${new Date(s.t * 1000).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" })}${s.note ? ` · ${esc(s.note)}` : ""}</small></span></li>`;
  }).join("")) || '<li class="muted plain">Nothing yet.</li>');
}

function renderLog() {
  const events = ((map && map.events) || []).filter((e) => !e.text.startsWith("Pixhawk:") || e.level !== "info");
  const clock = (e) => (e.at ? new Date(e.at * 1000).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit", second: "2-digit" }) : mmss(e.t));
  setHtml($("log"), events.slice().reverse().slice(0, 40).map((e) => `<li class="${e.level}"><time>${clock(e)}</time>${esc(e.text)}</li>`).join("")
    || '<li class="empty">Nothing yet.</li>');
}

async function pollMessages() {
  try {
    const o = await (await fetch("/api/wildlife/outbox")).json();
    const msgs = o.messages || [];
    msgCount = msgs.length;
    setText("outbox-mode", o.live ? "· sent by Telegram" : "· simulated (no Telegram token set)");
    setText("tg-state", o.live ? "Telegram is connected." : "Not connected: set TELEGRAM_BOT_TOKEN and TELEGRAM_CHAT_ID before starting.");
    if (tab === "alerts") msgsSeen = msgCount;
    setText("alerts-count", msgCount > msgsSeen ? String(msgCount - msgsSeen) : "");
    setHtml($("messages"), msgs.slice().reverse().map((m) => `<li class="${m.kind}">${m.photo ? `<img src="/api/wildlife/snapshot/${encodeURIComponent(m.photo)}" alt="">` : ""}${esc(m.text_en)}${m.text_local ? `<span class="local">${esc(m.text_local)}</span>` : ""}<span class="meta"><span>To ${m.channel === "village" ? "the village" : "forest officers"}</span><span>${new Date(m.t * 1000).toLocaleTimeString()}</span><span>${esc(m.delivery)}</span></span></li>`).join("")
      || '<li class="empty">No messages yet. The first elephant sighting sends one to officers and the village.</li>');
  } catch (e) { /* server restarting */ }
  setTimeout(pollMessages, 2000);
}

function renderSettings() {
  if (!status) return;
  setText("sim-state", status.sim_running ? `Running: ${status.world} world` : "Stopped");
  setText("s-sim", status.sim_running ? "Stop" : "Start");
  $$(".seg button").forEach((b) => b.classList.toggle("on", b.dataset.mode === status.sensor_mode));
  const w = (map && map.wildlife) || {};
  if (document.activeElement !== $("s-auto")) $("s-auto").checked = !!w.auto_dispatch;
}

function drawLoop() {
  if (tab === "map") drawMap($("map"), map, { theme: "light", showRisk: true });
  setTimeout(drawLoop, 250);
}

// --------------------------------------------------------------- streams

function connect() {
  const st = new EventSource("/api/status");
  st.onmessage = (e) => { status = JSON.parse(e.data); renderStatus(); renderLive(); renderSettings(); };
  st.onerror = () => { status = null; renderStatus(); };
  const mp = new EventSource("/api/map");
  mp.onmessage = (e) => { map = JSON.parse(e.data); buildThumbs(map.wildlife || {}); renderStatus(); renderLive(); renderPaths(); renderSightings(); renderLog(); };
}

let start = "live";
try { start = localStorage.getItem("tab") || "live"; } catch (e) { /* private mode */ }
start = new URLSearchParams(location.search).get("tab") || start;
showTab(start);
showCam("drone");
connect();
refreshThumbs();
pollMessages();
drawLoop();
