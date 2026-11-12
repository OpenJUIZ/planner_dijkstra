"""mobile_sim と planner_dijkstra を IPC でつないで走らせ、経路計画を確かめる(M2 の結合テスト)

mobile_sim の真の姿勢から目標までの経路を求め、壁の上の目標・地図の差し替え(mobile_sim の地図を画像で渡す)・
トピック amcl_pose と goal での計画を試す。

使い方:
  scripts/dev_plugin.sh
  python3 scripts/integration_test.py --mobile-sim <mobile_sim のリポジトリ> [--juiz <juiz のパス>] [--ui SCREENSHOT]
    mobile_sim は app/plugins/<target>/ にプラグインを置いておく(mobile_sim の scripts/dev_plugin.sh)
  python3 scripts/integration_test.py --mobile-sim-juizapp mobile_sim-0.3.4.juizapp --juizapp dist/planner_dijkstra-0.2.1.juizapp
    パッケージで動かす(ポートは mobile_sim 8000、planner_dijkstra 8003)
  python3 scripts/integration_test.py --sim-url http://localhost:8000 --planner-url http://localhost:8003
    もう動いている mobile_sim と planner_dijkstra を試す(docker compose など)

IPC の名前 mobile_sim と planner_dijkstra を使うので、同じ機械でそれらを動かしていないこと。
"""
import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import urllib.parse
import urllib.request
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--juiz", default="juiz")
    ap.add_argument("--mobile-sim", type=Path, help="mobile_sim のリポジトリ(app/app.conf を使う)")
    ap.add_argument("--mobile-sim-juizapp", type=Path, help="mobile_sim の .juizapp(juiz run で動かす)")
    ap.add_argument("--juizapp", type=Path, help="planner_dijkstra の .juizapp(juiz run で動かす。ポートは 8003)")
    ap.add_argument("--sim-port", type=int, default=18000)
    ap.add_argument("--port", type=int, default=18003)
    ap.add_argument("--sim-url", help="もう動いている mobile_sim(--planner-url と一緒に使う)")
    ap.add_argument("--planner-url", help="もう動いている planner_dijkstra")
    ap.add_argument("--ui", type=Path, metavar="SCREENSHOT",
                    help="続けてブラウザ画面のテスト (scripts/ui_test.js、Playwright が要る) を走らせ、画面を保存する")
    args = ap.parse_args()
    if args.sim_url or args.planner_url:
        if not (args.sim_url and args.planner_url):
            sys.exit("give both --sim-url and --planner-url")
        sim, base = args.sim_url.rstrip("/"), args.planner_url.rstrip("/")
        for url in (sim, base):
            wait_ready(url, None)
        run(sim, base)
        print("OK")
        return
    if not args.mobile_sim and not args.mobile_sim_juizapp:
        sys.exit("give --mobile-sim or --mobile-sim-juizapp (or --sim-url and --planner-url)")

    procs = []
    tmps = []
    try:
        if args.mobile_sim_juizapp:
            sim = "http://localhost:8000"
            work = tempfile.mkdtemp()
            env = dict(os.environ, JUIZ_CACHE_DIR=os.path.join(work, "cache"))
            procs.append(start([args.juiz, "run", str(args.mobile_sim_juizapp.resolve())], work, env))
        else:
            sim = f"http://localhost:{args.sim_port}"
            app = args.mobile_sim.resolve() / "app"
            conf = (app / "app.conf").read_text().replace('"port": 8000', f'"port": {args.sim_port}')
            tmp = app / ".planner_integration_test.conf"
            tmp.write_text(conf)
            tmps.append(tmp)
            procs.append(start([args.juiz, "-f", tmp.name, "-d", "--ipc", "mobile_sim"], app))
        wait_ready(sim, procs[-1])

        if args.juizapp:
            base = "http://localhost:8003"
            work = tempfile.mkdtemp()
            env = dict(os.environ, JUIZ_CACHE_DIR=os.path.join(work, "cache"))
            procs.append(start([args.juiz, "run", str(args.juizapp.resolve())], work, env))
        else:
            base = f"http://localhost:{args.port}"
            conf = (ROOT / "app" / "app.conf").read_text().replace('"port": 8003', f'"port": {args.port}')
            tmp = ROOT / "app" / ".integration_test.conf"
            tmp.write_text(conf)
            tmps.append(tmp)
            procs.append(start([args.juiz, "-f", tmp.name, "-d"], ROOT / "app"))
        wait_ready(base, procs[-1])

        run(sim, base)
        if args.ui:
            print("# ui_test.js")
            subprocess.run(["node", str(ROOT / "scripts" / "ui_test.js"), f"{base}/", str(args.ui.resolve())], check=True)
        print("OK")
    except SystemExit:
        for p in procs:
            dump_log(p)
        raise
    finally:
        for p in reversed(procs):
            p.terminate()
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
        for t in tmps:
            t.unlink(missing_ok=True)


def start(cmd, cwd, env=None):
    log = tempfile.TemporaryFile()
    p = subprocess.Popen(cmd, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT)
    p.log = log
    p.cmd = cmd
    return p


def dump_log(p):
    p.log.seek(0)
    text = p.log.read().decode(errors="replace")
    print(f"--- log of {' '.join(map(str, p.cmd))} ---\n{text[-4000:]}", file=sys.stderr)


def wait_ready(base, proc):
    for _ in range(150):
        if proc is not None and proc.poll() is not None:
            proc.log.seek(0)
            sys.exit(f"juiz exited ({base}):\n" + proc.log.read().decode(errors="replace")[-4000:])
        try:
            with urllib.request.urlopen(f"{base}/api/container/list", timeout=1) as r:
                if json.load(r):
                    return
        except OSError:
            pass
        time.sleep(0.2)
    sys.exit(f"juiz did not become ready ({base})")


def ident(base, name, type_name, container):
    return f"{base}/container_process/{name}::{type_name}:{container}"


def call(base, name, type_name, container, args=None, raw=False):
    url = f"{base}/api/container_process/call?identifier=" + urllib.parse.quote(ident(base, name, type_name, container), safe="")
    req = urllib.request.Request(
        url, data=json.dumps(args or {}).encode(), method="PATCH", headers={"Content-Type": "application/json"}
    )
    with urllib.request.urlopen(req, timeout=30) as r:
        body = r.read()
        return body if raw else json.loads(body)


def call_multipart(base, name, type_name, container, parts):
    """画像の引数を持つ Process を multipart で呼ぶ(画像は image/png、ほかは application/json)"""
    boundary = uuid.uuid4().hex
    body = b""
    for key, (ctype, data) in parts.items():
        body += f"--{boundary}\r\nContent-Disposition: form-data; name=\"{key}\"\r\nContent-Type: {ctype}\r\n\r\n".encode()
        body += data + b"\r\n"
    body += f"--{boundary}--\r\n".encode()
    url = f"{base}/api/container_process/call?identifier=" + urllib.parse.quote(ident(base, name, type_name, container), safe="")
    req = urllib.request.Request(url, data=body, method="PUT",
                                 headers={"Content-Type": f"multipart/form-data; boundary={boundary}"})
    with urllib.request.urlopen(req, timeout=30) as r:
        return json.loads(r.read())


def sim_call(sim, name, kind, args=None, raw=False):
    return call(sim, name, f"mobile_sim_{kind}", "sim0", args, raw)


def planner_call(base, name, kind, args=None, raw=False):
    return call(base, name, f"planner_{kind}", "planner0", args, raw)


def push_topic(base, topic, value):
    url = f"{base}/api/topic/push?topic_name={topic}"
    req = urllib.request.Request(url, data=json.dumps({"input": value}).encode(), method="PATCH",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        r.read()


def check(cond, msg):
    if not cond:
        sys.exit("FAILED: " + msg)
    print("ok -", msg)


def pose(x, y, theta=0.0):
    return {"position": {"x": x, "y": y}, "theta": theta}


def stamped(x, y, theta=0.0, frame="map"):
    return {"header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": frame}, "pose": pose(x, y, theta)}


def xy(p):
    return p["position"]["x"], p["position"]["y"]


def wait_for(fn, timeout=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        v = fn()
        if v:
            return v
        time.sleep(0.1)
    return None


def check_path(path, start, goal, label):
    check(path["ok"], f"{label}: a path is found ({path['message']})")
    poses = path["poses"]
    check(xy(poses[0]) == start[:2] and xy(poses[-1]) == goal[:2] and poses[-1]["theta"] == goal[2],
          f"{label}: it starts at the start and ends at the goal pose")
    # 両端は格子の中心でなく与えた姿勢なので、隣の格子の中心まで最大で 1.5 格子の斜め(0.106 m)
    steps = [math.dist(xy(a), xy(b)) for a, b in zip(poses, poses[1:])]
    check(max(steps) < 0.11, f"{label}: consecutive points are neighbouring cells (max step {max(steps):.3f} m)")
    straight = math.dist(start[:2], goal[:2])
    check(straight <= path["length"] < straight * 2.0, f"{label}: length {path['length']:.2f} m (straight {straight:.2f} m)")
    check(path["header"]["frame_id"] == "map", f"{label}: frame_id is map")


def run(sim, base):
    # 0) mobile_sim の真の姿勢(既定の初期位置 -2, 0)
    truth = sim_call(sim, "get_pose0", "get_pose")["pose"]
    start = (truth["position"]["x"], truth["position"]["y"], truth["theta"])
    print(f"# truth {start}")
    info = planner_call(base, "get_map_info0", "get_map_info")
    check(info["width"] == 200 and info["resolution"] == 0.05 and info["origin"][:2] == [-5.0, -5.0],
          f"planner loaded the default map ({info['path']})")

    # 1) 真の姿勢から、仕切り壁(x = 1.5、出入口は y = -1〜1)の向こうの右の部屋へ
    goal = (3.0, -2.5, 1.57)
    path = planner_call(base, "plan0", "plan", {"start": pose(*start), "goal": pose(*goal)})
    check_path(path, start, goal, "right room")
    check(all(-1.0 < xy(p)[1] < 1.0 for p in path["poses"] if 1.4 < xy(p)[0] < 1.6),
          "the path goes through the doorway of the partition")
    check(planner_call(base, "get_path0", "get_path") == path, "get_path returns the last path")

    # 2) 壁の上・地図の外の目標は ok: false と理由
    bad = planner_call(base, "plan0", "plan", {"start": pose(*start), "goal": pose(-4.97, 0.0)})
    check(not bad["ok"] and bad["message"] == "the goal is on an obstacle" and bad["poses"] == [],
          f"a goal in the outer wall is rejected ({bad['message']})")
    bad = planner_call(base, "plan0", "plan", {"start": pose(*start), "goal": pose(9.0, 0.0)})
    check(not bad["ok"] and bad["message"] == "the goal is outside the map", f"a goal outside is rejected ({bad['message']})")

    # 3) mobile_sim の地図を画像で渡す(画面の「mobile_sim の地図を使う」と同じ)
    png = sim_call(sim, "get_map0", "get_map", raw=True)
    sim_info = sim_call(sim, "get_map_info0", "get_map_info")
    j = lambda v: ("application/json", json.dumps(v).encode())
    new_info = call_multipart(base, "set_map0", "planner_set_map", "planner0", {
        "image": ("image/png", png),
        "resolution": j(sim_info["resolution"]),
        "origin": j(sim_info["origin"]),
        "negate": j(bool(sim_info["negate"])),
        "occupied_thresh": j(sim_info["occupied_thresh"]),
        "free_thresh": j(sim_info["free_thresh"]),
    })
    check(new_info["path"] == "(image)" and new_info["width"] == sim_info["width"]
          and new_info["version"] == info["version"] + 1, "set_map takes the map image of mobile_sim")
    check(planner_call(base, "get_path0", "get_path")["message"] == "no plan yet", "set_map drops the old path")
    again = planner_call(base, "plan0", "plan", {"start": pose(*start), "goal": pose(*goal)})
    check(again["ok"] and abs(again["length"] - path["length"]) < 1e-6, "the same path on the map of mobile_sim")
    img = planner_call(base, "get_map0", "get_map", raw=True)
    check(img[:8] == b"\x89PNG\r\n\x1a\n", "get_map returns a PNG")

    # 4) トピック: amcl_pose で姿勢を、goal で目標を受けて計画する
    s2, g2 = (-3.0, 2.0, 0.0), (0.5, 3.5, -1.0)
    push_topic(base, "amcl_pose", stamped(*s2))
    got = wait_for(lambda: planner_call(base, "get_state0", "get_state")["start"])
    check(got is not None and xy(got["pose"]) == s2[:2], "the topic amcl_pose sets the start")
    push_topic(base, "goal", stamped(*g2))
    p2 = wait_for(lambda: (lambda p: p if p["ok"] and xy(p["poses"][-1]) == g2[:2] else None)(
        planner_call(base, "get_path0", "get_path")))
    check(p2 is not None, "the topic goal triggers planning")
    check_path(p2, s2, g2, "topic")
    st = planner_call(base, "get_state0", "get_state")
    check(xy(st["goal"]["pose"]) == g2[:2] and st["count"] >= 4, f"state has the goal and the count ({st['count']})")

    # 5) set_goal を直接呼んでも同じ(画面が amcl_pose を使うとき)
    p3 = planner_call(base, "set_goal0", "set_goal", {"goal": stamped(*goal)})
    check_path(p3, s2, goal, "set_goal")

    # 6) YAML の地図を読み直す
    back = planner_call(base, "load_map0", "load_map", {"map": "assets/maps/default.yaml"})
    check(back["path"].endswith("assets/maps/default.yaml") and back["version"] == new_info["version"] + 1,
          "load_map reads the YAML again")

    # 7) ロボットの大きさ(issue #8): 経路は扉の角から robot_radius + safety_margin より離れ、壁のそばの目標は寄せる
    opts = planner_call(base, "get_state0", "get_state")["options"]
    check(opts["robot_radius"] == 0.2 and opts["safety_margin"] == 0.05, f"inflation options from app.conf ({opts})")
    door = planner_call(base, "plan0", "plan", {"start": pose(*start), "goal": pose(2.5, -3.0, 0.0)})
    check(door["ok"], "a path to (2.5, -3.0) through the doorway")
    at_door = [xy(p)[1] for p in door["poses"] if 1.4 < xy(p)[0] < 1.6]
    check(at_door and all(-0.7 < y < 0.7 for y in at_door),
          f"the path keeps away from the corners of the doorway (y at the partition: {min(at_door):.2f}..{max(at_door):.2f})")
    near = planner_call(base, "plan0", "plan", {"start": pose(*start), "goal": pose(2.5, -4.85, 0.0)})
    moved = near.get("goal_moved_to")
    check(near["ok"] and moved and moved["y"] > -4.7 and xy(near["poses"][-1]) == (moved["x"], moved["y"]),
          f"a goal next to the outer wall is moved away from it ({near['message']})")
    # 膨らませ方を替える。0 にすると 0.1.0 と同じく角をかすめる
    o = planner_call(base, "set_inflation0", "set_inflation",
                     {"robot_radius": 0.0, "safety_margin": 0.0, "inflation_radius": 0.0, "inflation_weight": 0.0})
    check(o["robot_radius"] == 0.0 and o["inflation_weight"] == 0.0, "set_inflation changes the options")
    raw = planner_call(base, "plan0", "plan", {"start": pose(*start), "goal": pose(2.5, -3.0, 0.0)})
    raw_door = [xy(p)[1] for p in raw["poses"] if 1.4 < xy(p)[0] < 1.6]
    check(raw["ok"] and min(raw_door) < -0.85, f"without inflation the path hugs the corner (y {min(raw_door):.2f})")
    o = planner_call(base, "set_inflation0", "set_inflation", {"robot_radius": 0.2, "safety_margin": 0.05,
                                                              "inflation_radius": 0.6, "inflation_weight": 2.0})
    check(o["robot_radius"] == 0.2, "set_inflation restores the options")
    o = planner_call(base, "set_inflation0", "set_inflation", {"safety_margin": None, "robot_radius": None,
                                                              "inflation_radius": None, "inflation_weight": None})
    check(o["robot_radius"] == 0.2 and o["inflation_weight"] == 2.0, "set_inflation keeps the values not given")
    img = planner_call(base, "get_costmap0", "get_costmap", raw=True)
    check(img[:8] == b"\x89PNG\r\n\x1a\n", "get_costmap returns a PNG")
    # 画面のテストのために、最後の経路を捨てておく
    planner_call(base, "load_map0", "load_map", {"map": "assets/maps/default.yaml"})


if __name__ == "__main__":
    main()
