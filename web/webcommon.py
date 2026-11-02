"""server.py（7010 录制）与 nav_server.py（7011 导航）共用的 HTTP 工具。"""
import json
import mimetypes
import os
import re
import subprocess
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent.parent
DATA_ROOT = Path(os.environ.get("LIDAR_DATA_ROOT", Path.home() / "yx/Data/Lidar_data"))
DIST_DIR = Path(__file__).resolve().parent / "dist"
SESSION_RE = re.compile(r"^\d{8}_\d{6}$")


class LastError:
    """最近一次启动/停止失败的提示，只在一段时间内显示，避免状态恢复后旧报错一直挂在页面上。"""

    def __init__(self, ttl=60):
        self.ttl, self.msg, self.t = ttl, "", 0.0

    def set(self, msg):
        self.msg, self.t = msg, time.time()

    def get(self):
        return self.msg if time.time() - self.t < self.ttl else ""


def run_script(script, *args, timeout=None):
    r = subprocess.run([str(script), *args], capture_output=True, text=True, timeout=timeout)
    return r.returncode, (r.stdout + r.stderr).strip()


def last_line(text, default):
    return text.splitlines()[-1] if text else default


def parse_kv(text):
    out = {}
    for line in text.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            out[k.strip()] = v.strip()
    return out


def session_dir(name):
    """校验录制目录名（防止路径穿越），返回其路径或 None。"""
    if not isinstance(name, str) or not SESSION_RE.match(name):
        return None
    d = DATA_ROOT / name
    return d if d.is_dir() else None


def read_json(path, default=None):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


def mature_running(d):
    """是否有 make_mature.sh 正在处理该录制目录。"""
    for cmd in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            args = cmd.read_bytes().split(b"\0")
        except OSError:
            continue
        if any(a.endswith(b"make_mature.sh") for a in args) and str(d).encode() in args:
            return True
    return False


def mature_info(d):
    """录制目录的成熟地图状态：已有地图（nav/meta.json）+ 最近一次生成的进度。"""
    meta = read_json(d / "nav/meta.json")
    # 旧版 make_mature.sh 把进度写在 nav/progress.json
    progress = read_json(d / "mature_progress.json") or read_json(d / "nav/progress.json")
    if not meta and not progress:
        return None
    if progress and progress.get("status") == "running" and not mature_running(d):
        progress = {**progress, "status": "failed", "step": "上次生成被中断（如断电），请重新生成"}
    return {"map": meta, "progress": progress}


class BaseHandler(BaseHTTPRequestHandler):
    index_page = "index.html"

    def send(self, code, body, ctype="application/json; charset=utf-8"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def body_json(self):
        n = int(self.headers.get("Content-Length") or 0)
        try:
            return json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return {}

    def send_file(self, f):
        ctype = mimetypes.guess_type(f.name)[0] or "application/octet-stream"
        if ctype.startswith("text/") or ctype.endswith("javascript"):
            ctype += "; charset=utf-8"
        self.send(200, f.read_bytes(), ctype)

    def static(self, path):
        f = (DIST_DIR / path.lstrip("/")).resolve()
        if path == "/" or not f.is_file():
            f = DIST_DIR / self.index_page    # 单页应用：未知路径都返回入口页
        if DIST_DIR.resolve() not in f.resolve().parents or not f.is_file():
            return self.send(503, "前端未构建，请运行 web/frontend 下的 npm run build".encode(),
                             "text/plain; charset=utf-8")
        self.send_file(f)

    def proxy_sse(self, url, unavailable_msg):
        try:
            upstream = urllib.request.urlopen(url, timeout=30)
        except OSError:
            return self.send(503, {"error": unavailable_msg})
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

    def proxy_post(self, url, body):
        req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=10) as r:
                return self.send(r.status, r.read())
        except urllib.error.HTTPError as e:
            return self.send(e.code, e.read())
        except OSError:
            return self.send(503, {"ok": False, "msg": "导航未运行"})

    def log_message(self, fmt, *args):
        line = str(args[0] if args else "")
        if "/api/" in line and not any(s in line for s in ("/status", "/heartbeat", "/maps")):
            super().log_message(fmt, *args)
