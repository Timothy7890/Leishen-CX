#!/usr/bin/env python3
"""导航 ROS <-> 网页桥接（由 nav.sh 启动，监听 127.0.0.1:7013，由 web/nav_server.py 转发）。

    GET  /stream     SSE，每 0.2 s 一个 state 事件：位姿、定位/安全门/串口状态、目标、全局路径、障碍点
    POST /reloc      {"mode": "auto"} 或 {"mode": "manual", "x", "y", "yaw"}
    POST /goal       {"x", "y", "yaw"}
    POST /cancel
    POST /estop      {"on": true|false}
    POST /speed      {"v": 0.3}
    POST /heartbeat  网页每秒调用一次；超过 5 s 未调用，safety_gate 停车
"""
import os

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import base64
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import actionlib
import numpy as np
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Path
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, Float64, String

from vehicle import FRONT, HALF_WID, REAR, drop_self

FOOTPRINT = [round(FRONT, 3), round(REAR, 3), round(HALF_WID, 3)]   # 网页画车：body 系前/后/半宽

PORT = 7013
SCAN_MAX = 2500
SCAN_UNIT = 0.02            # 障碍点以 2 cm 为单位编码成 int16
GOAL_SNAP_R = 1.0           # 目标点离障碍太近时，在该半径内找最近的可达点
INSCRIBED = 99              # 代价地图（0~100）中 ≥ 该值表示车身会碰撞
GOAL_STATES = {GoalStatus.PENDING: "pending", GoalStatus.ACTIVE: "active", GoalStatus.SUCCEEDED: "succeeded",
               GoalStatus.ABORTED: "aborted", GoalStatus.PREEMPTED: "canceled", GoalStatus.RECALLED: "canceled",
               GoalStatus.REJECTED: "rejected", GoalStatus.LOST: "idle"}


class Bridge:
    def __init__(self, nav_dir):
        with open(os.path.join(nav_dir, "meta.json")) as f:
            self.floor_map = json.load(f)["floor_z"]     # 地图系地面高度
        self.lock = threading.Lock()
        self.pose = None            # [x, y, yaw]
        self.T_mb = None
        self.loc = self.gate = self.serial = {}
        self.goal = None
        self.goal_sent = False
        self.plan = []
        self.scan = ""
        self.speed = 0.3
        self.estop = False
        self.costmap = None

        self.pub_reloc = rospy.Publisher("/nav/reloc_request", String, queue_size=1)
        self.pub_estop = rospy.Publisher("/nav/estop", Bool, queue_size=1, latch=True)
        self.pub_speed = rospy.Publisher("/nav/max_speed", Float32, queue_size=1, latch=True)
        self.pub_hb = rospy.Publisher("/nav/heartbeat", Float64, queue_size=1)
        self.pub_estop.publish(Bool(False))
        self.pub_speed.publish(Float32(self.speed))

        rospy.Subscriber("/nav/robot_pose", PoseStamped, self.on_pose, queue_size=1)
        rospy.Subscriber("/nav/loc_status", String, lambda m: self._set("loc", m), queue_size=1)
        rospy.Subscriber("/nav/gate_status", String, lambda m: self._set("gate", m), queue_size=1)
        rospy.Subscriber("/nav/serial_status", String, lambda m: self._set("serial", m), queue_size=1)
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.on_plan, queue_size=1)
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid,
                         lambda m: setattr(self, "costmap", m), queue_size=1)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.on_cloud, queue_size=1, buff_size=2 ** 24)
        self.mb = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.dwa = None

    def _set(self, key, msg):
        try:
            setattr(self, key, json.loads(msg.data))
        except ValueError:
            pass

    def on_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        R = Rotation.from_quat([q.x, q.y, q.z, q.w])
        with self.lock:
            self.T_mb = (R.as_matrix(), np.array([p.x, p.y, p.z]))
            self.pose = [round(p.x, 3), round(p.y, 3), round(float(R.as_euler("zyx")[0]), 4)]

    def on_plan(self, msg):
        pts = [(round(s.pose.position.x, 2), round(s.pose.position.y, 2)) for s in msg.poses]
        step = max(1, len(pts) // 300)
        with self.lock:
            self.plan = pts[::step] + pts[-1:] if pts else []

    def on_cloud(self, msg):
        """障碍高度区间内的点，变换到地图系供网页显示（2 Hz）。"""
        now = time.time()
        if now - getattr(self, "_scan_t", 0) < 0.5:
            return
        self._scan_t = now
        with self.lock:
            T = self.T_mb
        if T is None:
            return
        off = {f.name: f.offset for f in msg.fields}
        dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                       "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step})
        p = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
        xyz = np.stack([p["x"], p["y"], p["z"]], 1).astype(np.float64)
        xyz = xyz[np.isfinite(xyz).all(1) & (np.einsum("ij,ij->i", xyz, xyz) > 0.25)]
        w = drop_self(xyz) @ T[0].T + T[1]
        h = w[:, 2] - self.floor_map
        w = w[(h > 0.1) & (h < 1.6)]
        if len(w) > SCAN_MAX:
            w = w[np.random.choice(len(w), SCAN_MAX, replace=False)]
        q = np.clip(np.round(w[:, :2] / SCAN_UNIT), -32000, 32000).astype("<i2")
        with self.lock:
            self.scan = base64.b64encode(q.tobytes()).decode()

    # ------------------------------------------------------------ 指令
    def goal_state(self):
        if not self.goal_sent:
            return "idle"
        return GOAL_STATES.get(self.mb.get_state(), "idle")

    def snap_goal(self, x, y):
        """GlobalPlanner 不支持目标容差：目标在车身会碰撞的区域时，挪到 GOAL_SNAP_R 内最近的可达格。"""
        cm = self.costmap
        if cm is None:
            return x, y, None
        i = cm.info
        a = np.array(cm.data, np.int16).reshape(i.height, i.width)
        res, ox, oy = i.resolution, i.origin.position.x, i.origin.position.y
        r, c = int((y - oy) / res), int((x - ox) / res)
        if not (0 <= r < i.height and 0 <= c < i.width):
            return None, None, "目标在地图范围外"
        if 0 <= a[r, c] < INSCRIBED:
            return x, y, None
        k = int(GOAL_SNAP_R / res)
        r0, c0 = max(0, r - k), max(0, c - k)
        win = a[r0:r + k + 1, c0:c + k + 1]
        rr, cc = np.nonzero((win >= 0) & (win < INSCRIBED))
        if len(rr) == 0:
            return None, None, f"目标附近 {GOAL_SNAP_R:.0f} m 内没有可到达的位置"
        j = np.argmin((rr + r0 - r) ** 2 + (cc + c0 - c) ** 2)
        nx, ny = ox + (cc[j] + c0 + 0.5) * res, oy + (rr[j] + r0 + 0.5) * res
        return nx, ny, f"目标离障碍太近，已移动 {math.hypot(nx - x, ny - y):.2f} m 到最近的可达位置"

    def send_goal(self, x, y, yaw):
        if not self.mb.wait_for_server(rospy.Duration(2.0)):
            return False, "move_base 未就绪"
        if self.loc.get("state") != "localized":
            return False, "请先完成定位"
        x, y, note = self.snap_goal(x, y)
        if x is None:
            return False, note
        g = MoveBaseGoal()
        g.target_pose.header.frame_id, g.target_pose.header.stamp = "map", rospy.Time.now()
        g.target_pose.pose.position.x, g.target_pose.pose.position.y = x, y
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        g.target_pose.pose.orientation.z, g.target_pose.pose.orientation.w = qz, qw
        self.mb.send_goal(g)
        self.goal, self.goal_sent = [round(x, 3), round(y, 3), round(yaw, 4)], True
        return True, note or "目标已下发"

    def cancel(self):
        if self.goal_sent:
            self.mb.cancel_all_goals()
        return True, "已取消目标"

    def set_estop(self, on):
        self.estop = bool(on)
        self.pub_estop.publish(Bool(self.estop))
        if self.estop:
            self.cancel()
        return True, "急停" if self.estop else "已解除急停"

    def set_speed(self, v):
        self.speed = max(0.05, min(0.6, float(v)))
        self.pub_speed.publish(Float32(self.speed))
        try:
            if self.dwa is None:
                import dynamic_reconfigure.client
                self.dwa = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS", timeout=2)
            self.dwa.update_configuration({"max_vel_x": self.speed, "max_vel_trans": self.speed})
        except Exception as e:
            rospy.logwarn("dynamic_reconfigure failed: %s", e)
        return True, f"限速 {self.speed:.2f} m/s"

    def state_json(self):
        with self.lock:
            s = {"pose": self.pose, "loc": self.loc, "gate": self.gate, "serial": self.serial,
                 "goal": self.goal, "goal_state": self.goal_state(), "plan": self.plan,
                 "scan": self.scan, "scan_unit": SCAN_UNIT, "speed": self.speed, "estop": self.estop,
                 "footprint": FOOTPRINT}
        return json.dumps(s, ensure_ascii=False, separators=(",", ":"))


bridge = None


class Handler(BaseHTTPRequestHandler):
    def _json(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path != "/stream":
            return self._json(404, {"error": "not found"})
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            while not rospy.is_shutdown():
                self.wfile.write(f"event: state\ndata: {bridge.state_json()}\n\n".encode())
                self.wfile.flush()
                time.sleep(0.2)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            return self._json(400, {"ok": False, "msg": "bad json"})
        p = self.path
        try:
            if p == "/heartbeat":
                bridge.pub_hb.publish(Float64(time.time()))
                return self._json(200, {"ok": True})
            if p == "/reloc":
                if bridge.loc.get("state") == "relocalizing":
                    ok, msg = False, "重定位进行中，已忽略本次请求"
                else:
                    bridge.pub_reloc.publish(String(json.dumps(body)))
                    ok, msg = True, "已开始重定位"
            elif p == "/goal":
                ok, msg = bridge.send_goal(float(body["x"]), float(body["y"]), float(body["yaw"]))
            elif p == "/cancel":
                ok, msg = bridge.cancel()
            elif p == "/estop":
                ok, msg = bridge.set_estop(body.get("on", True))
            elif p == "/speed":
                ok, msg = bridge.set_speed(body["v"])
            else:
                return self._json(404, {"ok": False, "msg": "not found"})
        except (KeyError, ValueError) as e:
            return self._json(400, {"ok": False, "msg": f"参数错误: {e}"})
        self._json(200 if ok else 409, {"ok": ok, "msg": msg})

    def log_message(self, *args):
        pass


def main():
    global bridge
    import sys
    rospy.init_node("nav_bridge")
    bridge = Bridge(sys.argv[1])
    server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    rospy.loginfo("nav bridge on 127.0.0.1:%d", PORT)
    rospy.spin()
    server.shutdown()


if __name__ == "__main__":
    main()
