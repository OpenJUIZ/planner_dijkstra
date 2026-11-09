// planner_dijkstra のブラウザ画面
//
// juiz の HTTP ブローカーの /api/container_process/call で ContainerProcess を呼ぶ。
// - コンテナ dijkstra_planner: get_state(5 Hz)、get_map(または膨らませた get_costmap)と get_map_info(地図が変わったとき)、
//   set_goal(出発に amcl_pose を使うとき)、plan(出発に mobile_sim の真の姿勢を使うとき)、
//   set_map(multipart で画像を渡す)、load_map
// - mobile_sim(subsystem としてつながっていれば): get_state(真の姿勢)、get_map と get_map_info(地図)
//   planner_dijkstra の HTTP 経由で ipc://mobile_sim/... を呼ぶので、mobile_sim 側の CORS の設定は要らない
"use strict";

const POLL_MS = 200;
const REPLAN_MS = 1000;

// ---- juiz の呼び出し ----

class JuizClient {
  constructor() {
    this.ids = {}; // "planner_get_state" などの型名 → 識別子
    this.containers = {};
  }

  // ContainerProcess の一覧(subsystem の分も)から識別子を集める。同じ型が複数あれば最初のもの
  async discover() {
    const res = await fetch("/api/container_process/list?recursive=true");
    if (!res.ok) throw new Error(`container_process/list: ${res.status}`);
    for (const e of await res.json()) {
      const [type, container] = e.type_name.split(":");
      if (this.ids[type]) continue;
      const ns = e.namespace ? `${e.namespace}/` : "";
      this.ids[type] = `${e.broker_type_name}://${e.broker_name}/container_process/${ns}${e.name}::${e.type_name}`;
      this.containers[type] = container;
    }
    for (const k of ["get_state", "get_map", "get_map_info", "set_goal", "plan", "set_map", "load_map"]) {
      if (!this.ids[`planner_${k}`]) throw new Error(`planner_${k} が見つかりません`);
    }
  }

  has(type) { return Boolean(this.ids[type]); }

  async call(type, args = {}, asBlob = false) {
    const url = "/api/container_process/call?identifier=" + encodeURIComponent(this.ids[type]);
    const res = await fetch(url, {
      method: "PATCH",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(args),
    });
    if (!res.ok) throw new Error(`${type}: ${res.status} ${await res.text()}`);
    return asBlob ? res.blob() : res.json();
  }

  // 画像の引数を持つ Process を multipart で呼ぶ(画像は image/png、ほかは application/json の部分)
  async callMultipart(type, parts) {
    const form = new FormData();
    for (const [k, v] of Object.entries(parts)) {
      form.append(k, v instanceof Blob ? v : new Blob([JSON.stringify(v)], { type: "application/json" }));
    }
    const url = "/api/container_process/call?identifier=" + encodeURIComponent(this.ids[type]);
    const res = await fetch(url, { method: "PUT", body: form });
    if (!res.ok) throw new Error(`${type}: ${res.status} ${await res.text()}`);
    return res.json();
  }
}

// ---- 座標変換 ----

class View {
  constructor(canvas) {
    this.canvas = canvas;
    this.cx = 0; this.cy = 0; this.size = 20; // 表示する正方形の中心と一辺 [m]
  }

  setRange(xmin, ymin, xmax, ymax) {
    this.cx = (xmin + xmax) / 2;
    this.cy = (ymin + ymax) / 2;
    this.size = Math.max(xmax - xmin, ymax - ymin, 1);
  }

  get pxPerM() { return this.canvas.width / this.size; }

  toCanvas(x, y) {
    const k = this.pxPerM;
    return [(x - this.cx) * k + this.canvas.width / 2, this.canvas.height / 2 - (y - this.cy) * k];
  }

  // 画面上の位置 (CSS の px) → 世界座標
  fromClient(clientX, clientY) {
    const rect = this.canvas.getBoundingClientRect();
    const px = ((clientX - rect.left) / rect.width) * this.canvas.width;
    const py = ((clientY - rect.top) / rect.height) * this.canvas.height;
    const k = this.pxPerM;
    return [(px - this.canvas.width / 2) / k + this.cx, (this.canvas.height / 2 - py) / k + this.cy];
  }
}

// ---- 地図 ----

class MapLayer {
  constructor() {
    this.image = null;
    this.info = null;
    this.version = -1;
  }

  // costmap が true で planner_get_costmap があれば、膨らませた地図を描く
  async load(client, costmap = false) {
    const type = costmap && client.has("planner_get_costmap") ? "planner_get_costmap" : "planner_get_map";
    const blob = await client.call(type, {}, true);
    const info = await client.call("planner_get_map_info");
    this.image = await loadImage(blob);
    this.info = info;
    this.version = info.version;
  }

  draw(ctx, view) {
    if (!this.image) return;
    const { resolution: r, origin: [ox, oy], width: w, height: h } = this.info;
    const [x0, y0] = view.toCanvas(ox, oy + h * r); // 画像の左上
    const k = view.pxPerM * r;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(this.image, x0, y0, w * k, h * k);
  }
}

async function loadImage(blob) {
  const url = URL.createObjectURL(blob);
  try {
    const img = new Image();
    img.src = url;
    await img.decode();
    return img;
  } finally {
    URL.revokeObjectURL(url);
  }
}

// ---- 描画 ----

function draw(view, map, st, truth, radius, picking) {
  const ctx = view.canvas.getContext("2d");
  ctx.fillStyle = "#cdcdcd";
  ctx.fillRect(0, 0, view.canvas.width, view.canvas.height);
  map.draw(ctx, view);
  if (!st) return;
  const path = st.path;
  if (path.ok) drawPath(ctx, view, path.poses.map((p) => [p.position.x, p.position.y]), "#2d6ad6");
  if (truth) drawRobot(ctx, view, truth, radius, { stroke: "#1a8a3a" });
  if (st.start) drawRobot(ctx, view, flat(st.start.pose), radius, { stroke: "#e06000", dash: [4, 3] });
  if (path.start) drawRobot(ctx, view, flat(path.start), radius, { fill: "rgba(224, 96, 0, 0.85)", stroke: "#803000" });
  if (path.goal) drawArrow(ctx, view, flat(path.goal), path.ok ? "#d4006a" : "#888");
  if (picking) drawArrow(ctx, view, picking, "#d4006a");
}

// Pose2D({position: {x, y}, theta})→ {x, y, theta}
function flat(p) { return { x: p.position.x, y: p.position.y, theta: p.theta }; }

function drawPath(ctx, view, pts, color) {
  if (pts.length < 2) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = 3;
  ctx.lineJoin = "round";
  ctx.beginPath();
  pts.forEach(([x, y], i) => {
    const [cx, cy] = view.toCanvas(x, y);
    if (i === 0) ctx.moveTo(cx, cy); else ctx.lineTo(cx, cy);
  });
  ctx.stroke();
}

function drawRobot(ctx, view, p, radius, { fill = null, stroke, dash = [] }) {
  const [cx, cy] = view.toCanvas(p.x, p.y);
  const rad = Math.max(radius * view.pxPerM, 4);
  ctx.lineWidth = 2;
  ctx.setLineDash(dash);
  ctx.strokeStyle = stroke;
  ctx.beginPath();
  ctx.arc(cx, cy, rad, 0, 2 * Math.PI);
  if (fill) { ctx.fillStyle = fill; ctx.fill(); }
  ctx.stroke();
  ctx.setLineDash([]);
  // 向き(キャンバスは y が下向きなので角度の符号を反転)
  ctx.strokeStyle = fill ? "#fff" : stroke;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx + rad * Math.cos(-p.theta), cy + rad * Math.sin(-p.theta));
  ctx.stroke();
}

function drawArrow(ctx, view, { x, y, theta }, color) {
  const [cx, cy] = view.toCanvas(x, y);
  const len = 30;
  const ex = cx + len * Math.cos(-theta), ey = cy + len * Math.sin(-theta);
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.arc(cx, cy, 4, 0, 2 * Math.PI);
  ctx.fill();
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(ex, ey);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(ex, ey);
  ctx.lineTo(ex - 12 * Math.cos(-theta - 0.4), ey - 12 * Math.sin(-theta - 0.4));
  ctx.lineTo(ex - 12 * Math.cos(-theta + 0.4), ey - 12 * Math.sin(-theta + 0.4));
  ctx.fill();
}

// ---- 目標の指定(押した位置から、ドラッグした方を向く) ----

class PosePicker {
  constructor(view, button, onPick) {
    this.view = view;
    this.button = button;
    this.armed = false;
    this.current = null; // ドラッグ中の {x, y, theta}
    const canvas = view.canvas;
    button.addEventListener("click", () => this.arm(!this.armed));
    canvas.addEventListener("pointerdown", (e) => {
      if (!this.armed) return;
      canvas.setPointerCapture(e.pointerId);
      const [x, y] = view.fromClient(e.clientX, e.clientY);
      this.current = { x, y, theta: 0 };
    });
    canvas.addEventListener("pointermove", (e) => {
      if (!this.current) return;
      const [x, y] = view.fromClient(e.clientX, e.clientY);
      if (Math.hypot(x - this.current.x, y - this.current.y) > 1e-3) {
        this.current.theta = Math.atan2(y - this.current.y, x - this.current.x);
      }
    });
    canvas.addEventListener("pointerup", () => {
      if (!this.current) return;
      const p = this.current;
      this.current = null;
      this.arm(false);
      onPick(p);
    });
    canvas.addEventListener("pointercancel", () => { this.current = null; });
  }

  arm(on) {
    this.armed = on;
    this.button.classList.toggle("armed", on);
    this.view.canvas.classList.toggle("picking", on);
  }
}

// ---- 表示 ----

const $ = (id) => document.getElementById(id);
const fmt = (v, d = 2) => v.toFixed(d);
const fmtPose = (p) => `x ${fmt(p.x)}, y ${fmt(p.y)}, θ ${fmt(p.theta)}`;

const MESSAGES = {
  "the start is outside the map": "出発が地図の外です",
  "the goal is outside the map": "目標が地図の外です",
  "the start is on an obstacle": "出発が障害物の上です",
  "the goal is on an obstacle": "目標が障害物の上です",
  "no path from the start to the goal": "出発から目標へ行く道がありません(ロボットが通れる幅の道が無い)",
  "the start is too close to obstacles and no free cell is near it": "出発が障害物に近すぎ、近くに通れる場所がありません",
  "the goal is too close to obstacles and no free cell is near it": "目標が障害物に近すぎ、近くに通れる場所がありません",
  "no plan yet": "まだ計画していません",
};

function showState(st, truth) {
  const p = st.path;
  const status = $("s_status");
  status.textContent = p.ok ? "経路あり" : (MESSAGES[p.message] || p.message);
  status.className = p.ok ? "good" : p.goal ? "bad" : "";
  $("s_start").textContent = p.start ? fmtPose(flat(p.start)) : "-";
  $("s_goal").textContent = p.goal ? fmtPose(flat(p.goal)) : "-";
  $("s_length").textContent = p.ok ? `${fmt(p.length)} m` : "-";
  $("s_points").textContent = p.ok ? `${p.poses.length}` : "-";
  $("s_expanded").textContent = p.ok ? `${p.expanded}` : "-";
  $("s_time").textContent = p.ok ? `${fmt(p.time_ms, 1)} ms` : "-";
  $("s_amcl").textContent = st.start ? `${fmtPose(flat(st.start.pose))}(${st.start.header.frame_id})` : "まだ来ていない";
  $("s_truth").textContent = truth ? fmtPose(truth) : "(mobile_sim が無い)";
  const m = st.map;
  $("s_map").textContent = `${m.path}(${m.width}×${m.height}、${m.resolution} m)`;
  const o = st.options || {};
  $("s_inflation").textContent = o.robot_radius === undefined ? "-"
    : `半径 ${fmt(o.robot_radius)} m + 余裕 ${fmt(o.safety_margin)} m、重みを上げる範囲 ${fmt(o.inflation_radius)} m`;
  $("s_count").textContent = `${st.count} 回`;
}

function setStatus(id, text, cls) {
  const c = $(id);
  c.textContent = text;
  c.className = `status ${cls}`;
}

function showError(err) {
  console.error(err);
  setStatus("connection", `エラー: ${err.message}`, "error");
}

function say(text) { $("message").textContent = text; }

async function main() {
  const client = new JuizClient();
  await client.discover();
  const name = client.containers.planner_get_state;
  setStatus("connection", `接続済み(コンテナ ${name})`, "ok");
  const hasSim = client.has("mobile_sim_get_state");
  if (hasSim) setStatus("sim_connection", "mobile_sim につながっています", "ok");
  else setStatus("sim_connection", "mobile_sim は見つかりません(真の姿勢は使えません)", "warn");
  $("map_sim").hidden = !(client.has("mobile_sim_get_map") && client.has("mobile_sim_get_map_info"));

  const view = new View($("map"));
  const map = new MapLayer();
  let st = null, truth = null, radius = 0.2, goal = null, loadingMap = false;

  // 出発に使う姿勢の出どころ: "amcl"(プランナが覚えた amcl_pose)か "truth"(mobile_sim の真の姿勢)
  function startSource() {
    const want = $("start_source").value;
    if (want === "auto") return st && st.start ? "amcl" : "truth";
    return want;
  }

  async function plan(silent = false) {
    if (!goal) { if (!silent) say("先に目標を置いてください"); return; }
    const g = { position: { x: goal.x, y: goal.y }, theta: goal.theta };
    let result;
    if (startSource() === "amcl") {
      result = await client.call("planner_set_goal", {
        goal: { header: { stamp: { sec: 0, nanosec: 0 }, frame_id: st ? st.map.frame_id : "map" }, pose: g },
      });
    } else {
      if (!truth) { if (!silent) say("出発の姿勢がありません(mobile_sim も amcl_pose も無い)"); return; }
      result = await client.call("planner_plan", {
        start: { position: { x: truth.x, y: truth.y }, theta: truth.theta },
        goal: g,
      });
    }
    if (!silent) {
      const moved = [];
      if (result.start_moved_to) moved.push("出発が障害物に近いので、まず離れます");
      if (result.goal_moved_to) moved.push(`目標が障害物に近いので (${fmt(result.goal_moved_to.x)}, ${fmt(result.goal_moved_to.y)}) に寄せました`);
      say(result.ok
        ? `経路を求めました: ${fmt(result.length)} m、${fmt(result.time_ms, 1)} ms` + (moved.length ? `(${moved.join("。")})` : "")
        : `経路が求まりません: ${MESSAGES[result.message] || result.message}`);
    }
  }

  const picker = new PosePicker(view, $("set_goal"), (p) => {
    goal = p;
    plan().catch((err) => say(`できませんでした: ${err.message}`));
  });
  $("replan_now").addEventListener("click", () => plan().catch((err) => say(`できませんでした: ${err.message}`)));
  $("map_sim").addEventListener("click", async () => {
    try {
      const blob = await client.call("mobile_sim_get_map", {}, true);
      const info = await client.call("mobile_sim_get_map_info");
      await client.callMultipart("planner_set_map", {
        image: new Blob([blob], { type: "image/png" }),
        resolution: info.resolution,
        origin: info.origin,
        negate: Boolean(info.negate),
        occupied_thresh: info.occupied_thresh ?? 0.65,
        free_thresh: info.free_thresh ?? 0.196,
      });
      say("mobile_sim の地図を使います");
    } catch (err) { say(`できませんでした: ${err.message}`); }
  });
  $("map_load").addEventListener("click", async () => {
    try {
      const info = await client.call("planner_load_map", { map: $("map_path").value });
      say(`読みました: ${info.path}`);
    } catch (err) { say(`読めませんでした: ${err.message}`); }
  });
  $("show_costmap").addEventListener("change", () => { map.version = -1; });
  setInterval(() => {
    if ($("replan").checked && goal) plan(true).catch(showError);
  }, REPLAN_MS);

  let busy = false;
  setInterval(async () => {
    if (busy) return; // 前の周がまだ終わっていない
    busy = true;
    try {
      st = await client.call("planner_get_state");
      if (hasSim) {
        const sim = await client.call("mobile_sim_get_state");
        const robot = sim.robots[0];
        truth = robot ? robot.pose : null;
        if (robot && robot.radius) radius = robot.radius;
      }
      // 地図は変わったときだけ取り直す
      if (!loadingMap && st.map.version !== map.version) {
        loadingMap = true;
        map.load(client, $("show_costmap").checked).catch(showError).finally(() => { loadingMap = false; });
      }
      // 表示の範囲は地図の情報から決める(画像を読み終える前にも目標を置けるように)
      {
        const { resolution: r, origin: [ox, oy], width: w, height: h } = st.map;
        view.setRange(ox, oy, ox + w * r, oy + h * r);
      }
      draw(view, map, st, truth, radius, picker.current);
      showState(st, truth);
      setStatus("connection", `接続済み(コンテナ ${name})`, "ok");
    } catch (err) {
      showError(err);
    } finally {
      busy = false;
    }
  }, POLL_MS);
}

main().catch(showError);
