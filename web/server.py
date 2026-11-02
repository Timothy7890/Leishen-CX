#!/usr/bin/env python3
"""镭神 CX 录制控制网页（端口 7010）。所有操作都通过 ../record.sh 完成。

    GET  /             控制页面（web/frontend 构建产物 web/dist）
    GET  /api/status   当前状态 + 历史录制列表 (JSON)
    GET  /api/live     实时地图 SSE 流（转发录制中的 scripts/live_map.py）
    POST /api/start    开始录制
    POST /api/stop     停止录制（后台执行，状态变为 stopping）
"""
import json
import mimetypes
import os
import subprocess
import threading
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PORT = int(os.environ.get("RECORD_WEB_PORT", "7010"))
PROJECT_DIR = Path(__file__).resolve().parent.parent
RECORD_SH = PROJECT_DIR / "record.sh"
DATA_ROOT = Path(os.environ.get("LIDAR_DATA_ROOT", Path.home() / "yx/Data/Lidar_data"))
DIST_DIR = Path(__file__).resolve().parent / "dist"
LIVE_URL = "http://127.0.0.1:7011/stream"

busy = threading.Lock()   # 同一时间只允许一个 start/stop
last_error = {"msg": ""}


def run_record(*args, timeout=None):
    r = subprocess.run([str(RECORD_SH), *args], capture_output=True, text=True, timeout=timeout)
    return r.returncode, (r.stdout + r.stderr).strip()


def parse_kv(text):
    out = {}
    for line in text.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            out[k.strip()] = v.strip()
    return out


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
            "has_map": (d / "map.pcd").exists(),
        })
    return sessions


def status_payload():
    _, out = run_record("status", timeout=10)
    st = parse_kv(out)
    if busy.locked() and st.get("status") in (None, "idle"):
        st["status"] = "starting"
    st["error"] = last_error["msg"]
    st["data_root"] = str(DATA_ROOT)
    st["sessions"] = list_sessions()
    return st


def do_stop():
    with busy:
        code, out = run_record("stop", timeout=600)
        last_error["msg"] = "" if code == 0 else out.splitlines()[-1] if out else "停止失败"


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/api/status":
            self._send(200, status_payload())
        elif path == "/api/live":
            self._proxy_live()
        else:
            self._static(path)

    def _static(self, path):
        f = (DIST_DIR / path.lstrip("/")).resolve()
        if path == "/" or not f.is_file():
            f = DIST_DIR / "index.html"   # 单页应用：未知路径都返回 index.html
        if DIST_DIR.resolve() not in f.resolve().parents or not f.is_file():
            return self._send(503, "前端未构建，请运行 web/frontend 下的 npm run build".encode(),
                              "text/plain; charset=utf-8")
        ctype = mimetypes.guess_type(f.name)[0] or "application/octet-stream"
        if ctype.startswith("text/") or ctype.endswith("javascript"):
            ctype += "; charset=utf-8"
        self._send(200, f.read_bytes(), ctype)

    def _proxy_live(self):
        try:
            # 上游每 10 s 有心跳，读超时 30 s 说明录制已结束
            upstream = urllib.request.urlopen(LIVE_URL, timeout=30)
        except OSError:
            return self._send(503, {"error": "未在录制"})
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        try:
            while True:
                line = upstream.readline()
                if not line:
                    break
                self.wfile.write(line)
                if line == b"\n":
                    self.wfile.flush()
        except OSError:
            pass
        finally:
            upstream.close()

    def do_POST(self):
        if self.path == "/api/start":
            if not busy.acquire(blocking=False):
                return self._send(409, {"ok": False, "msg": "正在执行其他操作"})
            try:
                code, out = run_record("start", timeout=120)
            finally:
                busy.release()
            last_error["msg"] = "" if code == 0 else (out.splitlines()[-1] if out else "启动失败")
            self._send(200 if code == 0 else 500, {"ok": code == 0, "msg": out})
        elif self.path == "/api/stop":
            if busy.locked():
                return self._send(409, {"ok": False, "msg": "正在执行其他操作"})
            threading.Thread(target=do_stop, daemon=True).start()
            self._send(202, {"ok": True, "msg": "正在停止并保存"})
        else:
            self._send(404, {"error": "not found"})

    def log_message(self, fmt, *args):
        if "/api/" in str(args[0] if args else "") and "/api/status" not in str(args[0]):
            super().log_message(fmt, *args)


if __name__ == "__main__":
    print(f"Leishen-CX 录制控制: http://0.0.0.0:{PORT}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
