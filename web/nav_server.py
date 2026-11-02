#!/usr/bin/env python3
"""镭神 CX 导航网页（端口 7011）。启停通过 ../nav.sh，运行中的指令转发给 scripts/nav_bridge.py。

    GET  /                          导航页面（web/dist/nav.html）
    GET  /api/maps                  成熟地图列表
    GET  /api/maps/<录制>/grid.png  2D 栅格图
    GET  /api/maps/<录制>/traj      建图轨迹 [[x, y], ...]
    GET  /api/nav/status            导航运行状态
    POST /api/nav/start             {"session", "dry_run"}
    POST /api/nav/stop
    GET  /api/nav/stream            实时状态 SSE（转发 nav_bridge）
    POST /api/nav/{reloc,goal,cancel,estop,speed,heartbeat}   转发 nav_bridge
"""
import os
import threading
from http.server import ThreadingHTTPServer

import numpy as np

from webcommon import (DATA_ROOT, PROJECT_DIR, BaseHandler, LastError, last_line, parse_kv, read_json, run_script,
                       session_dir)

PORT = int(os.environ.get("NAV_WEB_PORT", "7011"))
NAV_SH = PROJECT_DIR / "nav.sh"
BRIDGE = "http://127.0.0.1:7013"
BRIDGE_CMDS = {"reloc", "goal", "cancel", "estop", "speed", "heartbeat"}

busy = threading.Lock()
last_error = LastError()


def list_maps():
    # nav/ 只在生成成功后整体替换（make_mature.sh），有 meta.json 即可用
    maps = []
    if DATA_ROOT.is_dir():
        for d in sorted(DATA_ROOT.iterdir(), reverse=True):
            meta = read_json(d / "nav/meta.json")
            if meta:
                maps.append(meta)
    return maps


def map_traj(d, max_pts=600):
    t = np.loadtxt(d / "nav/traj.csv", delimiter=",", skiprows=1, ndmin=2)
    step = max(1, len(t) // max_pts)
    return np.round(t[::step, 1:3], 3).tolist()


def nav_status():
    _, out = run_script(NAV_SH, "status", timeout=10)
    st = parse_kv(out)
    if busy.locked() and st.get("status") in (None, "idle"):
        st["status"] = "starting"
    st["error"] = last_error.get()
    _, rec = run_script(PROJECT_DIR / "record.sh", "status", timeout=10)
    st["recording"] = parse_kv(rec).get("status") not in (None, "idle")
    return st


def do_stop():
    with busy:
        code, out = run_script(NAV_SH, "stop", timeout=120)
        last_error.set("" if code == 0 else last_line(out, "停止失败"))


class Handler(BaseHandler):
    index_page = "nav.html"

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        parts = path.strip("/").split("/")
        if path == "/api/maps":
            self.send(200, list_maps())
        elif len(parts) == 4 and parts[:2] == ["api", "maps"]:
            d = session_dir(parts[2])
            if d is None or not (d / "nav/meta.json").exists():
                return self.send(404, {"error": "no such map"})
            if parts[3] == "grid.png":
                self.send_file(d / "nav/grid.png")
            elif parts[3] == "traj":
                self.send(200, map_traj(d))
            else:
                self.send(404, {"error": "not found"})
        elif path == "/api/nav/status":
            self.send(200, nav_status())
        elif path == "/api/nav/stream":
            self.proxy_sse(f"{BRIDGE}/stream", "导航未运行")
        else:
            self.static(path)

    def do_POST(self):
        path = self.path
        if path == "/api/nav/start":
            body = self.body_json()
            d = session_dir(body.get("session"))
            if d is None or not (d / "nav/meta.json").exists():
                return self.send(400, {"ok": False, "msg": "请选择成熟地图"})
            if not busy.acquire(blocking=False):
                return self.send(409, {"ok": False, "msg": "正在执行其他操作"})
            try:
                args = ["start", str(d)] + (["--dry-run"] if body.get("dry_run") else [])
                code, out = run_script(NAV_SH, *args, timeout=150)
            finally:
                busy.release()
            last_error.set("" if code == 0 else last_line(out, "启动失败"))
            self.send(200 if code == 0 else 500, {"ok": code == 0, "msg": out})
        elif path == "/api/nav/stop":
            if busy.locked():
                return self.send(409, {"ok": False, "msg": "正在执行其他操作"})
            threading.Thread(target=do_stop, daemon=True).start()
            self.send(202, {"ok": True, "msg": "正在停止"})
        elif path.startswith("/api/nav/") and path.rsplit("/", 1)[1] in BRIDGE_CMDS:
            self.proxy_post(f"{BRIDGE}/{path.rsplit('/', 1)[1]}", self.body_json())
        else:
            self.send(404, {"error": "not found"})


if __name__ == "__main__":
    print(f"Leishen-CX 导航: http://0.0.0.0:{PORT}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
