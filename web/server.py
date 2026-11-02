#!/usr/bin/env python3
"""镭神 CX 录制控制网页（端口 7010）。录制操作都通过 ../record.sh 完成。

    GET  /             控制页面（web/frontend 构建产物 web/dist/index.html）
    GET  /api/status   当前状态 + 历史录制列表（含成熟地图状态）
    GET  /api/live     实时地图 SSE 流（转发录制中的 scripts/live_map.py）
    POST /api/start    开始录制
    POST /api/stop     停止录制（后台执行，状态变为 stopping）
    POST /api/mature   {"session", "name"} 把一次录制处理成成熟地图（后台执行 scripts/make_mature.sh）
"""
import os
import subprocess
import threading
from http.server import ThreadingHTTPServer

from webcommon import (DATA_ROOT, PROJECT_DIR, BaseHandler, last_line, parse_kv, read_json, run_script,
                       session_dir)

PORT = int(os.environ.get("RECORD_WEB_PORT", "7010"))
RECORD_SH = PROJECT_DIR / "record.sh"
MAKE_MATURE = PROJECT_DIR / "scripts/make_mature.sh"
LIVE_URL = "http://127.0.0.1:7012/stream"

busy = threading.Lock()   # 同一时间只允许一个 start/stop
last_error = {"msg": ""}


def list_sessions(limit=30):
    if not DATA_ROOT.is_dir():
        return []
    sessions = []
    for d in sorted(DATA_ROOT.iterdir(), reverse=True)[:limit]:
        if not d.is_dir():
            continue
        info_file = d / "info.txt"
        info = parse_kv(info_file.read_text(errors="replace")) if info_file.exists() else {}
        sessions.append({
            "name": d.name,
            "status": info.get("status", "recording"),
            "duration_s": info.get("duration_s"),
            "bag_size": info.get("bag_size"),
            "map_points": info.get("map_points"),
            "error": info.get("error"),
            "has_bag": (d / "raw.bag").exists(),
            "mature": read_json(d / "nav/progress.json"),
        })
    return sessions


def status_payload():
    _, out = run_script(RECORD_SH, "status", timeout=10)
    st = parse_kv(out)
    if busy.locked() and st.get("status") in (None, "idle"):
        st["status"] = "starting"
    st["error"] = last_error["msg"]
    st["data_root"] = str(DATA_ROOT)
    st["sessions"] = list_sessions()
    return st


def do_stop():
    with busy:
        code, out = run_script(RECORD_SH, "stop", timeout=600)
        last_error["msg"] = "" if code == 0 else last_line(out, "停止失败")


def start_mature(body):
    d = session_dir(body.get("session"))
    name = str(body.get("name", "")).strip()[:40]
    if d is None or not (d / "raw.bag").exists():
        return 400, "录制不存在或缺少 raw.bag"
    if not name:
        return 400, "请填写地图名称"
    if (read_json(d / "nav/progress.json") or {}).get("status") == "running":
        return 409, "这次录制正在生成成熟地图"
    if any((read_json(s / "nav/progress.json") or {}).get("status") == "running" for s in DATA_ROOT.iterdir()):
        return 409, "另一张成熟地图正在生成，请稍后"
    log = open(PROJECT_DIR / "run" / f"mature_{d.name}.log", "w")
    subprocess.Popen([str(MAKE_MATURE), str(d), name], stdout=log, stderr=subprocess.STDOUT,
                     stdin=subprocess.DEVNULL, start_new_session=True)
    return 202, "开始生成成熟地图（约为录制时长的 1~1.5 倍）"


class Handler(BaseHandler):
    index_page = "index.html"

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/api/status":
            self.send(200, status_payload())
        elif path == "/api/live":
            self.proxy_sse(LIVE_URL, "未在录制")
        else:
            self.static(path)

    def do_POST(self):
        if self.path == "/api/start":
            if not busy.acquire(blocking=False):
                return self.send(409, {"ok": False, "msg": "正在执行其他操作"})
            try:
                code, out = run_script(RECORD_SH, "start", timeout=120)
            finally:
                busy.release()
            last_error["msg"] = "" if code == 0 else last_line(out, "启动失败")
            self.send(200 if code == 0 else 500, {"ok": code == 0, "msg": out})
        elif self.path == "/api/stop":
            if busy.locked():
                return self.send(409, {"ok": False, "msg": "正在执行其他操作"})
            threading.Thread(target=do_stop, daemon=True).start()
            self.send(202, {"ok": True, "msg": "正在停止并保存"})
        elif self.path == "/api/mature":
            code, msg = start_mature(self.body_json())
            self.send(code, {"ok": code < 300, "msg": msg})
        else:
            self.send(404, {"error": "not found"})


if __name__ == "__main__":
    (PROJECT_DIR / "run").mkdir(exist_ok=True)
    print(f"Leishen-CX 录制控制: http://0.0.0.0:{PORT}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
