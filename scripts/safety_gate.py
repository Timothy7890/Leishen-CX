#!/usr/bin/env python3
"""安全门：move_base 的速度指令必须经过这里才会发给小车。

    /cmd_vel_nav (move_base) ──► 安全门 ──► /cmd_vel (car_serial.py)

以下任一情况输出零速度：
  - 网页急停（/nav/estop，锁存，需网页解除）
  - 定位未就绪或丢失（/nav/loc_status）
  - 网页心跳超过 HEARTBEAT_TIMEOUT 秒未更新（/nav/heartbeat）
  - 雷达数据中断，或 move_base 指令超时
  - 障碍物：前进时车头前方区域、原地转向时车角扫过的圆内有障碍
    -> 停车等待，障碍持续消失 CLEAR_HOLD 秒后自动继续（不绕行）

另外限速：线速度 [0, max_speed]（不倒车），角速度 ±MAX_W。
状态以 JSON 发布在 /nav/gate_status。
"""
import os

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import json
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, Float64, String

from vehicle import FRONT, HALF_WID, SELF_MARGIN, SWEEP_R, inside_body, lidar_to_body

# 车身轮廓来自 config/nav/costmap_common.yaml（见 scripts/vehicle.py），轮廓内的点视为车自身
FRONT_MIN, FRONT_PER_V = 0.5, 1.0   # 前方检测距离 = 车头外 0.5 m + 1.0 s × 速度
# 前方检测区左右各比车宽多出。固定的墙由路径规划避开，这里主要拦突然出现的障碍；
# 取 0.1 时侧屋窄通道（车身右侧离墙 6~10 cm）会一直误停，故取 0.03
SIDE_MARGIN = 0.03
TURN_MARGIN = 0.1           # 原地转向：车角扫过的圆再外扩
OBST_LOW, OBST_HIGH = 0.10, 1.6     # 地面以上该高度区间才算障碍
MIN_POINTS = 5              # 区域内至少这么多点才判为障碍（抗噪）
CLEAR_HOLD = 2.0            # 障碍消失后等待秒数
MAX_W = 0.8
CMD_TIMEOUT = 0.5
SCAN_TIMEOUT = 0.5
HEARTBEAT_TIMEOUT = 5.0


class Gate:
    def __init__(self):
        self.lock = threading.Lock()
        self.cmd, self.cmd_t = Twist(), 0.0
        self.scan_t = 0.0
        self.estop = False
        self.max_speed = float(rospy.get_param("~max_speed", 0.3))
        self.heartbeat = 0.0
        self.loc_state = "unlocalized"
        self.floor = rospy.get_param("~floor_z", None)   # body 坐标系下的地面高度
        self.floor_samples = []
        self.front_hits = self.ring_hits = 0
        self.blocked_since = None
        self.clear_since = None
        self.state = None

        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pub_status = rospy.Publisher("/nav/gate_status", String, queue_size=1, latch=True)
        rospy.Subscriber("/cmd_vel_nav", Twist, self.on_cmd, queue_size=1)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.on_cloud, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber("/nav/estop", Bool, lambda m: setattr(self, "estop", m.data), queue_size=1)
        rospy.Subscriber("/nav/max_speed", Float32,
                         lambda m: setattr(self, "max_speed", max(0.0, min(0.6, m.data))), queue_size=1)
        rospy.Subscriber("/nav/heartbeat", Float64, lambda m: setattr(self, "heartbeat", time.time()), queue_size=1)
        rospy.Subscriber("/nav/loc_status", String, self.on_loc, queue_size=1)

    def on_cmd(self, msg):
        with self.lock:
            self.cmd, self.cmd_t = msg, time.time()

    def on_loc(self, msg):
        try:
            self.loc_state = json.loads(msg.data).get("state", "unlocalized")
        except ValueError:
            pass

    def on_cloud(self, msg):
        off = {f.name: f.offset for f in msg.fields}
        dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                       "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step})
        p = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
        xyz = np.stack([p["x"], p["y"], p["z"]], 1).astype(np.float64)
        # 统一在 body 系里判断（外参与车身轮廓见 vehicle.py）
        x, y, z =lidar_to_body(xyz[np.isfinite(xyz).all(1)]).T

        if self.floor is None:
            # 自动标定地面：车周 1~5 m、低于车体原点的点，高度直方图峰值
            r = np.hypot(x, y)
            m = (r > 1.0) & (r < 5.0) & (z < -0.1)
            if m.sum() > 500:
                self.floor_samples.append(z[m])
            if len(self.floor_samples) >= 10:
                zz = np.concatenate(self.floor_samples)
                hist, edges = np.histogram(zz, bins=np.arange(zz.min(), zz.max() + 0.02, 0.02))
                i = int(np.argmax(hist))
                self.floor = float((edges[i] + edges[i + 1]) / 2)
                rospy.loginfo("floor z (body frame) = %.3f", self.floor)
            return

        h = z - self.floor
        band = (h > OBST_LOW) & (h < OBST_HIGH)
        x, y = x[band], y[band]
        not_self = ~inside_body(x, y)
        with self.lock:
            v = self.cmd.linear.x
        reach = FRONT + FRONT_MIN + FRONT_PER_V * max(0.0, v)
        front = (x > FRONT + SELF_MARGIN) & (x < reach) & (np.abs(y) < HALF_WID + SIDE_MARGIN)
        ring = not_self & (np.hypot(x, y) < SWEEP_R + TURN_MARGIN)
        with self.lock:
            self.front_hits, self.ring_hits = int(front.sum()), int(ring.sum())
            self.scan_t = time.time()

    def decide(self):
        now = time.time()
        with self.lock:
            cmd, cmd_age = self.cmd, now - self.cmd_t
            scan_age = now - self.scan_t
            front, ring = self.front_hits, self.ring_hits
        v = min(max(cmd.linear.x, 0.0), self.max_speed)
        w = max(-MAX_W, min(MAX_W, cmd.angular.z))

        if self.estop:
            return "estop", "急停中", 0, 0
        if self.floor is None:
            return "init", "正在标定地面高度", 0, 0
        if self.loc_state != "localized":
            return "no_loc", "未定位或定位丢失", 0, 0
        if now - self.heartbeat > HEARTBEAT_TIMEOUT:
            return "no_heartbeat", "网页连接中断，已停车", 0, 0
        if scan_age > SCAN_TIMEOUT:
            return "no_scan", "雷达数据中断", 0, 0
        if cmd_age > CMD_TIMEOUT or (abs(cmd.linear.x) < 1e-3 and abs(cmd.angular.z) < 1e-3):
            self.blocked_since = None
            return "idle", "待命", 0, 0

        moving_fwd = v > 0.02
        hit = front >= MIN_POINTS if moving_fwd else ring >= MIN_POINTS
        if hit:
            self.blocked_since = self.blocked_since or now
            self.clear_since = None
            where = "前方" if moving_fwd else "转向范围内"
            return "blocked", f"{where}有障碍，等待中（{now - self.blocked_since:.0f} s）", 0, 0
        if self.blocked_since is not None:
            self.clear_since = self.clear_since or now
            if now - self.clear_since < CLEAR_HOLD:
                return "blocked", "障碍已移开，即将继续", 0, 0
            self.blocked_since = self.clear_since = None
        return "moving", "行驶中", v, w

    def run(self):
        rate = rospy.Rate(10)
        last_pub = 0.0
        while not rospy.is_shutdown():
            state, msg, v, w = self.decide()
            out = Twist()
            out.linear.x, out.angular.z = v, w
            self.pub_cmd.publish(out)
            if state != self.state or time.time() - last_pub > 0.5:
                self.state, last_pub = state, time.time()
                self.pub_status.publish(String(json.dumps(
                    {"state": state, "msg": msg, "v": round(v, 3), "w": round(w, 3),
                     "max_speed": self.max_speed, "estop": self.estop,
                     "front_points": self.front_hits, "ring_points": self.ring_hits},
                    ensure_ascii=False)))
            rate.sleep()


def main():
    rospy.init_node("safety_gate")
    gate = Gate()
    rospy.on_shutdown(lambda: gate.pub_cmd.publish(Twist()))
    gate.run()


if __name__ == "__main__":
    main()
