// Situational map renderer, shared by the dashboard (index.html) and the
// printable SITREP (report.html). Draws the bridge's /resq/map data:
// local frame, metres east (e) / north (n) of the launch point.

const MAP_THEMES = {
  dark: {
    bg: "#0a0f16", grid: "rgba(148,163,184,0.07)", text: "#cbd5e1", dim: "#64748b",
    area: "rgba(148,163,184,0.55)", trail: "#38bdf8", footprint: "rgba(255,255,255,0.10)",
    footprintEdge: "rgba(255,255,255,0.45)", drone: "#f8fafc", route: "#4ade80",
    panel: "rgba(11,17,24,0.78)",
  },
  light: {
    bg: "#ffffff", grid: "rgba(15,23,42,0.06)", text: "#0f172a", dim: "#64748b",
    area: "rgba(15,23,42,0.5)", trail: "#0284c7", footprint: "rgba(15,23,42,0.06)",
    footprintEdge: "rgba(15,23,42,0.4)", drone: "#0f172a", route: "#16a34a",
    panel: "rgba(255,255,255,0.85)",
  },
};

const HAZARD_STYLE = {
  fire: { fill: "rgba(249,115,22,0.35)", edge: "#f97316", icon: "\u{1F525}" },
  flood: { fill: "rgba(59,130,246,0.35)", edge: "#3b82f6", icon: "\u{1F30A}" },
  structure: { fill: "rgba(148,163,184,0.25)", edge: "#94a3b8", icon: "\u{1F3DA}" },
};

// Buffers the ground route keeps from each hazard type (search_map.HAZARD_TYPES).
const HAZARD_BUFFER_M = { fire: 4.0, flood: 1.5, structure: 3.0 };

// Heat ramp for the probability map: dark violet (searched) -> amber (likely).
const HEAT_STOPS = [
  [0.0, [30, 27, 75, 0.0]],
  [0.12, [76, 29, 149, 0.35]],
  [0.4, [190, 24, 93, 0.5]],
  [0.75, [249, 115, 22, 0.62]],
  [1.0, [253, 224, 71, 0.75]],
];

function heatColor(v) {
  v = Math.max(0, Math.min(1, v));
  for (let i = 1; i < HEAT_STOPS.length; i++) {
    const [p1, c1] = HEAT_STOPS[i];
    if (v <= p1) {
      const [p0, c0] = HEAT_STOPS[i - 1];
      const t = (v - p0) / (p1 - p0);
      const c = c0.map((x, k) => x + (c1[k] - x) * t);
      return `rgba(${c[0] | 0},${c[1] | 0},${c[2] | 0},${c[3].toFixed(3)})`;
    }
  }
  return "rgba(253,224,71,0.75)";
}

const basemaps = {};
function basemapImage(url) {
  if (!basemaps[url]) {
    const img = new Image();
    img.src = url;
    basemaps[url] = img;
  }
  const img = basemaps[url];
  return img.complete && img.naturalWidth ? img : null;
}

let heatCache = { p: null, canvas: null };
function heatCanvas(p) {
  if (heatCache.p === p.p && heatCache.canvas) return heatCache.canvas;
  const canvas = heatCache.canvas && heatCache.canvas.width === p.nx && heatCache.canvas.height === p.ny
    ? heatCache.canvas : Object.assign(document.createElement("canvas"), { width: p.nx, height: p.ny });
  const ctx = canvas.getContext("2d");
  const img = ctx.createImageData(p.nx, p.ny);
  for (let r = 0; r < p.ny; r++) {
    for (let c = 0; c < p.nx; c++) {
      const v = p.p[r * p.nx + c];
      const o = ((p.ny - 1 - r) * p.nx + c) * 4;  // row 0 is the south edge
      if (v < 0.01) { img.data[o + 3] = 0; continue; }
      const rgba = heatRGBA(v);
      img.data[o] = rgba[0]; img.data[o + 1] = rgba[1]; img.data[o + 2] = rgba[2]; img.data[o + 3] = rgba[3] * 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  heatCache = { p: p.p, canvas };
  return canvas;
}

function heatRGBA(v) {
  v = Math.max(0, Math.min(1, v));
  for (let i = 1; i < HEAT_STOPS.length; i++) {
    const [p1, c1] = HEAT_STOPS[i];
    if (v <= p1) {
      const [p0, c0] = HEAT_STOPS[i - 1];
      const t = (v - p0) / (p1 - p0);
      return c0.map((x, k) => x + (c1[k] - x) * t);
    }
  }
  return HEAT_STOPS[HEAT_STOPS.length - 1][1];
}

// ---------------------------------------------------------------
// Wildlife layers (forest world, PS000183): risk, village, fields,
// towers and their view, the herd with its track and predicted path.
// ---------------------------------------------------------------

const LEVEL_COLOR = { none: "#64748b", watch: "#38bdf8", warning: "#f97316", critical: "#ef4444" };
const RISK_STOPS = [
  [0.0, [34, 197, 94, 0.0]],
  [0.25, [234, 179, 8, 0.18]],
  [0.6, [249, 115, 22, 0.32]],
  [1.0, [239, 68, 68, 0.45]],
];
const TOWER_RANGE_M = 60;

let riskCache = { v: null, canvas: null };
function riskCanvas(r) {
  if (riskCache.v === r.v && riskCache.canvas) return riskCache.canvas;
  const canvas = Object.assign(document.createElement("canvas"), { width: r.nx, height: r.ny });
  const ctx = canvas.getContext("2d");
  const img = ctx.createImageData(r.nx, r.ny);
  for (let row = 0; row < r.ny; row++) {
    for (let c = 0; c < r.nx; c++) {
      const v = r.v[row * r.nx + c] / 100;
      const o = ((r.ny - 1 - row) * r.nx + c) * 4;  // row 0 is the south edge
      let rgba = RISK_STOPS[RISK_STOPS.length - 1][1];
      for (let i = 1; i < RISK_STOPS.length; i++) {
        const [p1, c1] = RISK_STOPS[i];
        if (v <= p1) {
          const [p0, c0] = RISK_STOPS[i - 1];
          const t = (v - p0) / (p1 - p0);
          rgba = c0.map((x, k) => x + (c1[k] - x) * t);
          break;
        }
      }
      img.data[o] = rgba[0]; img.data[o + 1] = rgba[1]; img.data[o + 2] = rgba[2]; img.data[o + 3] = rgba[3] * 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  riskCache = { v: r.v, canvas };
  return canvas;
}

function polyPath(ctx, pts, X, Y) {
  ctx.beginPath();
  pts.forEach(([e, n], i) => (i ? ctx.lineTo(X(e), Y(n)) : ctx.moveTo(X(e), Y(n))));
  ctx.closePath();
}

// Under the trail and the drone: risk, places, towers.
function drawWildlifeGround(ctx, w, X, Y, M, now, theme, opts) {
  if (w.risk && opts.showRisk !== false) {
    const r = w.risk;
    ctx.imageSmoothingEnabled = true;
    ctx.globalAlpha = opts.theme === "light" ? 0.5 : 1;  // a wash, not a fill, on white
    ctx.drawImage(riskCanvas(r), X(r.e0), Y(r.n0 + r.ny * r.cell), M(r.nx * r.cell), M(r.ny * r.cell));
    ctx.globalAlpha = 1;
  }
  ctx.lineWidth = 1;
  ctx.strokeStyle = "rgba(234,179,8,0.55)";
  for (const f of w.fields || []) { polyPath(ctx, f, X, Y); ctx.stroke(); }
  if (w.forest_edge_n != null) {
    ctx.strokeStyle = "rgba(74,222,128,0.5)";
    ctx.setLineDash([6, 4]);
    ctx.beginPath(); ctx.moveTo(X(-100), Y(w.forest_edge_n)); ctx.lineTo(X(100), Y(w.forest_edge_n)); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = "rgba(74,222,128,0.75)";
    ctx.font = "600 10px system-ui, sans-serif";
    ctx.textAlign = "left";
    ctx.fillText("FOREST EDGE", X(-98), Y(w.forest_edge_n) - 4);
  }
  if (w.village) {
    const crit = w.level === "critical";
    ctx.fillStyle = crit ? `rgba(239,68,68,${0.10 + 0.08 * Math.sin(now / 200)})` : "rgba(239,68,68,0.06)";
    ctx.strokeStyle = "rgba(239,68,68,0.85)";
    ctx.lineWidth = crit ? 2.5 : 1.5;
    polyPath(ctx, w.village, X, Y);
    ctx.fill(); ctx.stroke();
    const top = Math.max(...w.village.map((p) => p[1]));
    const cx = w.village.reduce((s, p) => s + p[0], 0) / w.village.length;
    ctx.fillStyle = "rgba(239,68,68,0.95)";
    ctx.font = "700 11px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("VILLAGE", X(cx), Y(top) + 14);
  }
  // Paths: camera chains toward the village. A seen camera is filled; a
  // confirmed path (drone sent) is orange.
  for (const c of w.corridors || []) {
    const p = (w.paths || []).find((r) => r.corridor === c.id);
    ctx.strokeStyle = p && p.dispatch ? "rgba(249,115,22,0.85)" : "rgba(100,116,139,0.55)";
    ctx.lineWidth = 2;
    ctx.setLineDash([2, 5]);
    ctx.beginPath();
    c.cameras.forEach((cam, i) => (i ? ctx.lineTo(X(cam.e), Y(cam.n)) : ctx.moveTo(X(cam.e), Y(cam.n))));
    ctx.stroke();
    ctx.setLineDash([]);
    for (const cam of c.cameras) {
      const seen = p && p.cameras.some((x) => x.id === cam.id && x.seen);
      ctx.fillStyle = seen ? (p.dispatch ? "#f97316" : "#f59e0b") : "rgba(148,163,184,0.9)";
      ctx.beginPath(); ctx.arc(X(cam.e), Y(cam.n), 4, 0, 2 * Math.PI); ctx.fill();
    }
  }
  for (const t of w.towers || []) {
    // View wedge (towers' yaw is from +east, counter-clockwise).
    const a0 = t.yaw - t.hfov / 2;
    const a1 = t.yaw + t.hfov / 2;
    ctx.fillStyle = "rgba(56,189,248,0.07)";
    ctx.strokeStyle = "rgba(56,189,248,0.35)";
    ctx.beginPath();
    ctx.moveTo(X(t.e), Y(t.n));
    for (let k = 0; k <= 12; k++) {
      const a = a0 + ((a1 - a0) * k) / 12;
      ctx.lineTo(X(t.e + TOWER_RANGE_M * Math.cos(a)), Y(t.n + TOWER_RANGE_M * Math.sin(a)));
    }
    ctx.closePath();
    ctx.fill(); ctx.stroke();
    const seen = (w.sightings || []).some((s) => s.source === t.id && s.alert && Date.now() / 1000 - s.t < 15);
    ctx.fillStyle = seen ? "#ef4444" : "#38bdf8";
    ctx.beginPath(); ctx.arc(X(t.e), Y(t.n), 6, 0, 2 * Math.PI); ctx.fill();
    ctx.strokeStyle = theme.bg; ctx.lineWidth = 2; ctx.stroke();
    ctx.fillStyle = theme.text;
    ctx.font = "700 10px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.fillText(t.id, X(t.e), Y(t.n) + 18);
  }
}

// Over the trail: sightings, herd track, prediction, herd, standoff.
function drawWildlifeTop(ctx, w, X, Y, M, now, theme) {
  const nowS = Date.now() / 1000;
  for (const s of w.sightings || []) {
    if (s.e == null || s.source === "UAV" || (w.towers || []).some((t) => t.id === s.source)) continue;
    // People's reports and imported alerts: a pin each.
    const x = X(s.e); const y = Y(s.n);
    ctx.fillStyle = s.source === "community" ? "#a78bfa" : "#facc15";
    ctx.beginPath(); ctx.moveTo(x, y); ctx.arc(x, y - 12, 6, Math.PI * 0.75, Math.PI * 2.25); ctx.closePath(); ctx.fill();
    ctx.fillStyle = theme.bg;
    ctx.beginPath(); ctx.arc(x, y - 12, 2.5, 0, 2 * Math.PI); ctx.fill();
    if (nowS - s.t < 30) {
      ctx.fillStyle = theme.text;
      ctx.font = "600 9px system-ui, sans-serif";
      ctx.textAlign = "center";
      ctx.fillText(s.source === "community" ? "VILLAGER REPORT" : "IMPORTED ALERT", x, y - 22);
    }
  }
  const h = w.herd;
  const col = LEVEL_COLOR[w.level] || LEVEL_COLOR.watch;
  if (w.track && w.track.length > 1) {
    ctx.strokeStyle = col;
    ctx.globalAlpha = 0.75;
    ctx.lineWidth = 2;
    ctx.beginPath();
    w.track.forEach(([e, n], i) => (i ? ctx.lineTo(X(e), Y(n)) : ctx.moveTo(X(e), Y(n))));
    ctx.stroke();
    ctx.globalAlpha = 1;
  }
  if (!h) return;
  if (h.predicted && h.predicted.length > 1) {
    ctx.strokeStyle = col;
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 6]);
    ctx.lineDashOffset = -now / 50;
    ctx.beginPath();
    h.predicted.forEach(([e, n], i) => (i ? ctx.lineTo(X(e), Y(n)) : ctx.moveTo(X(e), Y(n))));
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.lineDashOffset = 0;
    // A tick per minute ahead.
    ctx.fillStyle = col;
    ctx.font = "600 9px system-ui, sans-serif";
    ctx.textAlign = "left";
    for (let k = 6; k < h.predicted.length; k += 6) {
      const [e, n] = h.predicted[k];
      ctx.beginPath(); ctx.arc(X(e), Y(n), 2.5, 0, 2 * Math.PI); ctx.fill();
      ctx.fillText(`+${k / 6}m`, X(e) + 5, Y(n) + 3);
    }
  }
  if (w.standoff) {
    const [e, n] = w.standoff;
    ctx.strokeStyle = "#e2e8f0";
    ctx.lineWidth = 1.2;
    ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(X(e), Y(n)); ctx.lineTo(X(h.e), Y(h.n)); ctx.stroke();
    ctx.setLineDash([]);
    ctx.beginPath(); ctx.arc(X(e), Y(n), 5, 0, 2 * Math.PI); ctx.stroke();
  }
  const x = X(h.e); const y = Y(h.n);
  const pulse = (now / 1100) % 1;
  ctx.strokeStyle = col;
  ctx.globalAlpha = 1 - pulse;
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(x, y, 12 + pulse * 18, 0, 2 * Math.PI); ctx.stroke();
  ctx.globalAlpha = 1;
  ctx.fillStyle = col;
  ctx.beginPath(); ctx.arc(x, y, 12, 0, 2 * Math.PI); ctx.fill();
  ctx.strokeStyle = "#fff"; ctx.lineWidth = 2; ctx.stroke();
  if (h.heading_deg != null) {
    const a = (h.heading_deg * Math.PI) / 180;
    ctx.strokeStyle = col; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(x + 14 * Math.sin(a), y - 14 * Math.cos(a)); ctx.lineTo(x + 26 * Math.sin(a), y - 26 * Math.cos(a)); ctx.stroke();
  }
  ctx.font = "15px system-ui, sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText("\u{1F418}", x, y + 1);
  ctx.textBaseline = "alphabetic";
  const label = `${h.count}${h.calf ? " incl. calf" : ""}${h.eta_s != null ? ` · village ${fmtTime(h.eta_s)}` : ""}`;
  ctx.font = "700 10px system-ui, sans-serif";
  const lw = ctx.measureText(label).width + 10;
  ctx.fillStyle = col;
  ctx.fillRect(x + 15, y - 24, lw, 15);
  ctx.fillStyle = "#fff";
  ctx.textAlign = "left";
  ctx.fillText(label, x + 20, y - 13);
}

function mapBounds(data) {
  const pts = [];
  if (data.wildlife && data.basemap) {
    // The whole forest world: towers, corridor, fields and village at once.
    const bm = data.basemap;
    pts.push([bm.e0 + 4, bm.n0 + 4], [bm.e0 + bm.w - 4, bm.n0 + bm.h - 4]);
  } else if (data.prob) {
    const p = data.prob;
    pts.push([p.e0, p.n0], [p.e0 + p.w, p.n0 + p.h]);
  } else if (data.drone) {
    pts.push([data.drone.e - 20, data.drone.n - 20], [data.drone.e + 20, data.drone.n + 20]);
  }
  if (data.staging) pts.push(data.staging);
  pts.push([0, 0]);
  const es = pts.map((p) => p[0]);
  const ns = pts.map((p) => p[1]);
  const m = 5;
  return { eMin: Math.min(...es) - m, eMax: Math.max(...es) + m, nMin: Math.min(...ns) - m, nMax: Math.max(...ns) + m };
}

function drawMap(canvas, data, opts = {}) {
  const theme = MAP_THEMES[opts.theme || "dark"];
  const now = opts.now || performance.now();
  const dpr = window.devicePixelRatio || 1;
  const cssW = canvas.clientWidth;
  const cssH = canvas.clientHeight;
  if (canvas.width !== Math.round(cssW * dpr) || canvas.height !== Math.round(cssH * dpr)) {
    canvas.width = Math.round(cssW * dpr);
    canvas.height = Math.round(cssH * dpr);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = theme.bg;
  ctx.fillRect(0, 0, cssW, cssH);
  if (!data) {
    ctx.fillStyle = theme.dim;
    ctx.font = "14px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("Map appears once the mission bridge is running", cssW / 2, cssH / 2);
    return;
  }

  let b = mapBounds(data);
  // opts.zoom > 1 zooms in around the drone (opts.follow) or the map centre.
  const zoom = Math.max(1, opts.zoom || 1);
  if (zoom > 1) {
    const follow = opts.follow && data.drone;
    const ce = follow ? data.drone.e : (b.eMin + b.eMax) / 2;
    const cn = follow ? data.drone.n : (b.nMin + b.nMax) / 2;
    const he = (b.eMax - b.eMin) / 2 / zoom;
    const hn = (b.nMax - b.nMin) / 2 / zoom;
    b = { eMin: ce - he, eMax: ce + he, nMin: cn - hn, nMax: cn + hn };
  }
  const pad = 14;
  const scale = Math.min((cssW - 2 * pad) / (b.eMax - b.eMin), (cssH - 2 * pad) / (b.nMax - b.nMin));
  const ox = (cssW - (b.eMax - b.eMin) * scale) / 2;
  const oy = (cssH - (b.nMax - b.nMin) * scale) / 2;
  const X = (e) => ox + (e - b.eMin) * scale;
  const Y = (n) => oy + (b.nMax - n) * scale;
  // For clicks: canvas pixels -> metres (live.js places reports with it).
  canvas._view = { toEN: (x, y) => [b.eMin + (x - ox) / scale, b.nMax - (y - oy) / scale] };
  const M = (m) => m * scale;

  // Metre grid (20 m over the 200 m forest world, else 5 m)
  const step = data.wildlife ? 20 : 5;
  ctx.strokeStyle = theme.grid;
  ctx.lineWidth = 1;
  for (let e = Math.ceil(b.eMin / step) * step; e <= b.eMax; e += step) {
    ctx.beginPath(); ctx.moveTo(X(e), Y(b.nMin)); ctx.lineTo(X(e), Y(b.nMax)); ctx.stroke();
  }
  for (let n = Math.ceil(b.nMin / step) * step; n <= b.nMax; n += step) {
    ctx.beginPath(); ctx.moveTo(X(b.eMin), Y(n)); ctx.lineTo(X(b.eMax), Y(n)); ctx.stroke();
  }

  // Pre-disaster basemap (region world)
  const bm = data.basemap && basemapImage(data.basemap.url);
  if (bm) {
    ctx.globalAlpha = opts.theme === "light" ? 0.55 : 0.42;
    ctx.drawImage(bm, X(data.basemap.e0), Y(data.basemap.n0 + data.basemap.h), M(data.basemap.w), M(data.basemap.h));
    ctx.globalAlpha = 1;
  }

  // Probability heatmap: one pixel per cell in a small offscreen canvas,
  // rebuilt only when the data changes, then scaled up in one draw (a
  // 200 m area has ~9,000 cells - too many to fill one by one per frame).
  if (data.prob) {
    const p = data.prob;
    const heat = heatCanvas(p);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(heat, X(p.e0), Y(p.n0 + p.h), M(p.w), M(p.h));
    ctx.imageSmoothingEnabled = true;
    ctx.setLineDash([6, 4]);
    ctx.strokeStyle = theme.area;
    ctx.lineWidth = 1.2;
    ctx.strokeRect(X(p.e0), Y(p.n0 + p.h), M(p.w), M(p.h));
    ctx.setLineDash([]);
  }

  ctx.save();
  ctx.beginPath();
  ctx.rect(0, 0, cssW, cssH);
  ctx.clip();

  if (data.wildlife) drawWildlifeGround(ctx, data.wildlife, X, Y, M, now, theme, opts);

  // Hazards: zone + dashed safety buffer
  for (const h of data.hazards || []) {
    const st = HAZARD_STYLE[h.type] || HAZARD_STYLE.structure;
    const pulse = h.type === "fire" ? 0.75 + 0.25 * Math.sin(now / 250) : 1;
    ctx.globalAlpha = pulse;
    ctx.fillStyle = st.fill;
    ctx.beginPath(); ctx.arc(X(h.e), Y(h.n), Math.max(M(h.radius), 5), 0, 2 * Math.PI); ctx.fill();
    ctx.globalAlpha = 1;
    ctx.strokeStyle = st.edge;
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.setLineDash([3, 4]);
    ctx.beginPath();
    ctx.arc(X(h.e), Y(h.n), M(h.radius + (HAZARD_BUFFER_M[h.type] || 2)), 0, 2 * Math.PI);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.font = "13px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(st.icon, X(h.e), Y(h.n));
    if (h.source !== "intel") {
      ctx.font = "600 10px system-ui, sans-serif";
      ctx.fillStyle = st.edge;
      ctx.fillText(h.id, X(h.e), Y(h.n) + Math.max(M(h.radius), 5) + 9);
    }
  }
  ctx.textBaseline = "alphabetic";

  // Remaining lawnmower waypoints
  if (data.waypoints && data.waypoints.length) {
    ctx.strokeStyle = "rgba(250,204,21,0.45)";
    ctx.setLineDash([2, 5]);
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(X(data.drone.e), Y(data.drone.n));
    for (const [e, n] of data.waypoints) ctx.lineTo(X(e), Y(n));
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // Safe ground routes from staging to each casualty
  for (const v of data.victims || []) {
    if (!v.route) continue;
    ctx.strokeStyle = theme.route;
    ctx.lineWidth = 2.5;
    ctx.setLineDash([8, 5]);
    ctx.lineDashOffset = -now / 60;
    ctx.beginPath();
    v.route.forEach(([e, n], i) => (i ? ctx.lineTo(X(e), Y(n)) : ctx.moveTo(X(e), Y(n))));
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.lineDashOffset = 0;
  }

  // Flight trail
  const trail = data.trail || [];
  if (trail.length > 1) {
    ctx.strokeStyle = theme.trail;
    ctx.lineWidth = 1.6;
    ctx.globalAlpha = 0.8;
    ctx.beginPath();
    trail.forEach(([e, n], i) => (i ? ctx.lineTo(X(e), Y(n)) : ctx.moveTo(X(e), Y(n))));
    ctx.stroke();
    ctx.globalAlpha = 1;
  }

  // Camera footprint
  if (data.footprint) {
    ctx.fillStyle = theme.footprint;
    ctx.strokeStyle = theme.footprintEdge;
    ctx.lineWidth = 1;
    ctx.beginPath();
    data.footprint.forEach(([e, n], i) => (i ? ctx.lineTo(X(e), Y(n)) : ctx.moveTo(X(e), Y(n))));
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
  }

  if (data.wildlife) drawWildlifeTop(ctx, data.wildlife, X, Y, M, now, theme);

  // Planner target
  if (data.target) {
    const [e, n] = data.target;
    ctx.strokeStyle = "#facc15";
    ctx.lineWidth = 1.5;
    ctx.setLineDash([4, 3]);
    ctx.beginPath(); ctx.moveTo(X(data.drone.e), Y(data.drone.n)); ctx.lineTo(X(e), Y(n)); ctx.stroke();
    ctx.setLineDash([]);
    const r = 7;
    ctx.beginPath(); ctx.arc(X(e), Y(n), r, 0, 2 * Math.PI); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(X(e) - r - 4, Y(n)); ctx.lineTo(X(e) + r + 4, Y(n));
    ctx.moveTo(X(e), Y(n) - r - 4); ctx.lineTo(X(e), Y(n) + r + 4);
    ctx.stroke();
  }

  // Rejected candidates
  ctx.strokeStyle = theme.dim;
  ctx.lineWidth = 2;
  for (const [e, n] of data.rejected || []) {
    ctx.beginPath();
    ctx.moveTo(X(e) - 5, Y(n) - 5); ctx.lineTo(X(e) + 5, Y(n) + 5);
    ctx.moveTo(X(e) + 5, Y(n) - 5); ctx.lineTo(X(e) - 5, Y(n) + 5);
    ctx.stroke();
  }

  // Candidate being verified
  if (data.candidate) {
    const [e, n] = data.candidate;
    const t = (now / 900) % 1;
    ctx.strokeStyle = `rgba(250,204,21,${1 - t})`;
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(X(e), Y(n), 6 + t * 18, 0, 2 * Math.PI); ctx.stroke();
    ctx.fillStyle = "#facc15";
    ctx.beginPath(); ctx.arc(X(e), Y(n), 4, 0, 2 * Math.PI); ctx.fill();
  }

  // Staging point
  if (data.staging) {
    const [e, n] = data.staging;
    ctx.fillStyle = "#f8fafc";
    ctx.strokeStyle = "#16a34a";
    ctx.lineWidth = 2;
    ctx.fillRect(X(e) - 8, Y(n) - 8, 16, 16);
    ctx.strokeRect(X(e) - 8, Y(n) - 8, 16, 16);
    ctx.fillStyle = "#dc2626";
    ctx.fillRect(X(e) - 1.5, Y(n) - 5, 3, 10);
    ctx.fillRect(X(e) - 5, Y(n) - 1.5, 10, 3);
    ctx.fillStyle = theme.text;
    ctx.font = "600 10px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("STAGING", X(e), Y(n) + 20);
  }

  // Casualties
  for (const v of data.victims || []) {
    const x = X(v.e);
    const y = Y(v.n);
    const pulse = (now / 1200) % 1;
    ctx.strokeStyle = `rgba(239,68,68,${0.8 * (1 - pulse)})`;
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(x, y, 9 + pulse * 16, 0, 2 * Math.PI); ctx.stroke();
    ctx.fillStyle = "#ef4444";
    ctx.beginPath(); ctx.arc(x, y, 9, 0, 2 * Math.PI); ctx.fill();
    ctx.strokeStyle = "#fff";
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = "#fff";
    ctx.font = "700 9px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(v.id, x, y + 0.5);
    ctx.textBaseline = "alphabetic";
    if (v.priority) {
      const label = `P${v.priority}`;
      ctx.font = "700 10px system-ui, sans-serif";
      const w = ctx.measureText(label).width + 8;
      ctx.fillStyle = v.priority === 1 ? "#dc2626" : "#b45309";
      ctx.fillRect(x + 10, y - 20, w, 14);
      ctx.fillStyle = "#fff";
      ctx.textAlign = "left";
      ctx.fillText(label, x + 14, y - 9);
    }
  }

  // Drone
  if (data.drone) {
    const x = X(data.drone.e);
    const y = Y(data.drone.n);
    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(data.drone.yaw);
    ctx.fillStyle = theme.drone;
    ctx.strokeStyle = theme.bg;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(0, -11); ctx.lineTo(7, 8); ctx.lineTo(0, 4); ctx.lineTo(-7, 8);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }
  ctx.restore();

  // North arrow + scale bar
  ctx.fillStyle = theme.text;
  ctx.strokeStyle = theme.text;
  ctx.lineWidth = 1.5;
  ctx.textAlign = "center";
  ctx.font = "600 10px system-ui, sans-serif";
  // North arrow bottom-left (top-right holds the map's zoom buttons).
  const nx = 22;
  const ny = cssH - 16;
  ctx.beginPath(); ctx.moveTo(nx, ny); ctx.lineTo(nx, ny - 18); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(nx - 4, ny - 13); ctx.lineTo(nx, ny - 20); ctx.lineTo(nx + 4, ny - 13); ctx.fill();
  ctx.fillText("N", nx, ny - 26);
  const barM = data.wildlife ? 20 : 10;
  const bx = cssW - 20 - M(barM);
  const by = cssH - 16;
  ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(bx + M(barM), by);
  ctx.moveTo(bx, by - 4); ctx.lineTo(bx, by + 4);
  ctx.moveTo(bx + M(barM), by - 4); ctx.lineTo(bx + M(barM), by + 4);
  ctx.stroke();
  ctx.fillText(`${barM} m`, bx + M(barM) / 2, by - 6);

  // Probability of success
  if (typeof data.pos === "number" && opts.showPos !== false) {
    const label = "PROBABILITY OF SUCCESS";
    ctx.fillStyle = theme.panel;
    ctx.fillRect(10, 10, 176, 44);
    ctx.fillStyle = theme.dim;
    ctx.font = "600 9px system-ui, sans-serif";
    ctx.textAlign = "left";
    ctx.fillText(label, 18, 24);
    ctx.fillStyle = theme.text;
    ctx.font = "700 16px system-ui, sans-serif";
    ctx.fillText(`${(data.pos * 100).toFixed(0)}%`, 18, 45);
    ctx.fillStyle = "rgba(148,163,184,0.25)";
    ctx.fillRect(66, 36, 110, 6);
    ctx.fillStyle = "#4ade80";
    ctx.fillRect(66, 36, 110 * Math.min(1, data.pos), 6);
  }
}

function fmtTime(s) {
  if (s == null) return "—";
  const m = Math.floor(s / 60);
  const r = Math.round(s % 60);
  return `${m}:${String(r).padStart(2, "0")}`;
}

function escapeHtml(text) {
  return String(text).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
}
