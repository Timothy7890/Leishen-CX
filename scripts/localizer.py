#!/usr/bin/env python3
"""地图内定位：全局重定位 + 持续跟踪（由 nav.sh 启动）。

坐标系: map（成熟地图）-> camera_init（本次 FAST-LIO 里程计）-> body（IMU）
本节点求 T_map_odom 并以 TF map->camera_init 发布；FAST-LIO 负责 camera_init->body。

重定位（参考 D5G-beacon reloc.py 的多起点 ICP；本车无相机，候选点改为沿建图轨迹撒点）:
  1. 候选位置：建图轨迹每隔 ~1.5 m 一个点（自动），或网页点选的位置周围（手动）
  2. 每个位置 × 多个朝向，粗 ICP（0.3 m 体素，点到面）打分
  3. 前 8 名做精 ICP（0.08 m 体素 + Tukey 鲁棒核），fitness 达标者里取 rmse 最低
  4. 与次优的不同解比较，分数接近则判为有歧义，需要手动点选

话题:
  订阅 雷达点云（vehicle.LIDAR_TOPIC） /Odometry /nav/reloc_request(String JSON)
  发布 /nav/loc_status(String JSON, latched)  /nav/robot_pose(PoseStamped, map 系)
"""
import os

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import collections
import json
import math
import sys
import threading
import time

import numpy as np
import open3d as o3d
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from vehicle import LIDAR_TOPIC, drop_self

SCAN_VOXEL = 0.1
RELOC_SCANS = 10            # 重定位用最近 1 s 的扫描
TRACK_SCANS = 5
CAND_SPACING = 1.5          # 自动重定位候选点间距 (m)
N_YAW = 12
CROP_R = 25.0               # 目标地图裁剪半径 (m)
RMSE_ACCEPT = 0.15
FIT_ACCEPT = 0.45
TRACK_PERIOD = 1.0
TRACK_MAX_JUMP = (0.5, math.radians(8))     # 单次跟踪修正上限
LOST_AFTER = 5                              # 连续失败次数 -> lost

reg = o3d.pipelines.registration
P2PL = reg.TransformationEstimationPointToPlane()
P2PL_TUKEY = reg.TransformationEstimationPointToPlane(reg.TukeyLoss(k=0.1))


def T_from(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def yaw_of(T):
    return math.atan2(T[1, 0], T[0, 0])


def voxel_np(xyz, v):
    k = np.floor(xyz / v).astype(np.int64)
    _, idx = np.unique(k, axis=0, return_index=True)
    return xyz[idx]


def to_pcd(xyz, nrm=None):
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz.astype(np.float64)))
    if nrm is not None:
        pc.normals = o3d.utility.Vector3dVector(nrm.astype(np.float64))
    return pc


class MapData:
    def __init__(self, nav_dir):
        d = np.load(os.path.join(nav_dir, "reloc.npz"))
        self.fine_pts, self.fine_nrm = d["fine_pts"], d["fine_nrm"]
        self.coarse_pts, self.coarse_nrm = d["coarse_pts"], d["coarse_nrm"]
        traj = np.loadtxt(os.path.join(nav_dir, "traj.csv"), delimiter=",", skiprows=1)
        self.traj = traj[:, 1:4]
        # 沿轨迹等间距取候选点
        cands, last = [self.traj[0]], self.traj[0]
        for p in self.traj[1:]:
            if np.linalg.norm(p[:2] - last[:2]) >= CAND_SPACING:
                cands.append(p)
                last = p
        self.candidates = np.array(cands)

    def crop(self, center, fine=True, r=CROP_R):
        pts, nrm = (self.fine_pts, self.fine_nrm) if fine else (self.coarse_pts, self.coarse_nrm)
        m = np.sum((pts[:, :2] - center[:2]) ** 2, axis=1) < r * r
        return to_pcd(pts[m], nrm[m])


class Localizer:
    def __init__(self, nav_dir):
        self.map = MapData(nav_dir)
        self.lock = threading.Lock()
        self.odom_hist = collections.deque(maxlen=50)     # (t, T_odom_body)
        self.pending = collections.deque(maxlen=10)       # (t_end, xyz in body)，等对应时刻的里程计
        self.scans = collections.deque(maxlen=RELOC_SCANS)  # (t, xyz in odom)
        self.T_map_odom = None
        self.state = "unlocalized"
        self.fail = 0
        self.busy = False
        self.requests = collections.deque()

        self.pub_status = rospy.Publisher("/nav/loc_status", String, queue_size=1, latch=True)
        self.pub_pose = rospy.Publisher("/nav/robot_pose", PoseStamped, queue_size=1)
        # move_base 的局部规划需要当前速度；FAST-LIO 的 /Odometry 不带速度，这里由相邻位姿差分得到
        self.pub_odom = rospy.Publisher("/odom", Odometry, queue_size=5)
        self.prev_odom = None       # (t, T)
        self.vel = np.zeros(2)      # 车体系前进速度 (m/s)、偏航角速度 (rad/s)，平滑后
        self.tfb = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("/Odometry", Odometry, self.on_odom, queue_size=20)
        rospy.Subscriber(LIDAR_TOPIC, PointCloud2, self.on_cloud, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber("/nav/reloc_request", String, self.on_request, queue_size=5)
        self.status(state="unlocalized", msg=f"地图已加载，{len(self.map.candidates)} 个候选位置")

    # ------------------------------------------------------------ 状态
    def status(self, **kw):
        kw.setdefault("state", self.state)
        self.state = kw["state"]
        kw["t"] = time.time()
        self.last_status = kw
        self.pub_status.publish(String(json.dumps(kw, ensure_ascii=False)))
        rospy.loginfo("loc: %s", json.dumps(kw, ensure_ascii=False))

    # ------------------------------------------------------------ 回调
    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        T = T_from(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), [p.x, p.y, p.z])
        stamp = msg.header.stamp.to_sec()
        with self.lock:
            self.odom_hist.append((stamp, T))
            # FAST-LIO 处理完一帧才发布对应位姿（比点云晚约 15 ms），在这里配对
            for t_end, xyz in list(self.pending):
                if abs(t_end - stamp) < 0.03:
                    self.scans.append((stamp, xyz @ T[:3, :3].T + T[:3, 3]))
            while self.pending and self.pending[0][0] < stamp + 0.03:
                self.pending.popleft()
            T_mo = self.T_map_odom
        self.publish_velocity(msg, stamp, T)
        if T_mo is None:
            return
        self.publish_tf(T_mo, msg.header.stamp)
        T_mb = T_mo @ T
        ps = PoseStamped()
        ps.header.stamp, ps.header.frame_id = msg.header.stamp, "map"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = T_mb[:3, 3]
        qq = Rotation.from_matrix(T_mb[:3, :3]).as_quat()
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = qq
        self.pub_pose.publish(ps)

    def publish_velocity(self, msg, stamp, T):
        prev, self.prev_odom = self.prev_odom, (stamp, T)
        if prev is None or not 0.02 < stamp - prev[0] < 0.5:
            return
        dt, Rp = stamp - prev[0], prev[1][:3, :3]
        v = (Rp.T @ (T[:3, 3] - prev[1][:3, 3]))[0] / dt
        w = Rotation.from_matrix(Rp.T @ T[:3, :3]).as_rotvec()[2] / dt
        self.vel = 0.5 * self.vel + 0.5 * np.array([v, w])
        out = Odometry()
        out.header.stamp, out.header.frame_id, out.child_frame_id = msg.header.stamp, "camera_init", "base_link"
        out.pose.pose = msg.pose.pose
        out.twist.twist.linear.x, out.twist.twist.angular.z = self.vel
        self.pub_odom.publish(out)

    def publish_tf(self, T, stamp):
        ts = TransformStamped()
        ts.header.stamp, ts.header.frame_id, ts.child_frame_id = stamp, "map", "camera_init"
        ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = T[:3, 3]
        q = Rotation.from_matrix(T[:3, :3]).as_quat()
        ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w = q
        self.tfb.sendTransform(ts)

    def on_cloud(self, msg):
        # 镭神与 Livox 驱动 header.stamp 都为扫描起始，一帧 0.1 s；FAST-LIO 位姿时刻 = 扫描结束
        t_end = msg.header.stamp.to_sec() + 0.1
        off = {f.name: f.offset for f in msg.fields}
        dt = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                       "offsets": [off["x"], off["y"], off["z"]], "itemsize": msg.point_step})
        p = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
        xyz = np.stack([p["x"], p["y"], p["z"]], 1).astype(np.float64)
        r2 = np.einsum("ij,ij->i", xyz, xyz)
        xyz = voxel_np(xyz[np.isfinite(r2) & (r2 > 0.25) & (r2 < 900)], SCAN_VOXEL)
        xyz = drop_self(xyz)
        with self.lock:
            # FAST-LIO 算得比这里快时（Mid-360 点少），对应位姿已经到了，直接配对
            for stamp, T in reversed(self.odom_hist):
                if abs(t_end - stamp) < 0.03:
                    self.scans.append((stamp, xyz @ T[:3, :3].T + T[:3, 3]))
                    return
            self.pending.append((t_end, xyz))

    def on_request(self, msg):
        # 重定位进行中（或已有待处理请求）时直接忽略，不排队
        if self.busy or self.requests:
            rospy.loginfo("reloc request ignored: relocalization in progress")
            return
        try:
            self.requests.append(json.loads(msg.data))
        except ValueError:
            rospy.logwarn("bad reloc request: %s", msg.data)

    # ------------------------------------------------------------ 点云
    def source_in_body(self, n):
        """最近 n 帧扫描（odom 系）拼接后变换到当前 body 系。"""
        with self.lock:
            if len(self.scans) < n or not self.odom_hist:
                return None, None
            scans = list(self.scans)[-n:]
            _, T_ob = self.odom_hist[-1]
        xyz = voxel_np(np.vstack([s for _, s in scans]), SCAN_VOXEL)
        T_bo = np.linalg.inv(T_ob)
        return xyz @ T_bo[:3, :3].T + T_bo[:3, 3], T_ob

    # ------------------------------------------------------------ 重定位
    def relocalize(self, req):
        t0 = time.time()
        manual = req.get("mode") == "manual"
        self.status(state="relocalizing", msg="手动重定位中…" if manual else "自动重定位中…")
        src_xyz, T_ob = self.source_in_body(RELOC_SCANS)
        if src_xyz is None:
            return self.status(state="unlocalized", msg="雷达数据不足，稍后重试")

        # 保留当前倾斜（roll/pitch），只搜 x, y, yaw
        rp = Rotation.from_matrix(T_ob[:3, :3]).as_euler("zyx")
        R_tilt = Rotation.from_euler("zyx", [0.0, rp[1], rp[2]]).as_matrix()
        body_z = float(np.median(self.map.traj[:, 2]))

        if manual:
            cx, cy, cyaw = float(req["x"]), float(req["y"]), float(req["yaw"])
            ring = [(cx, cy)] + [(cx + 1.0 * math.cos(a), cy + 1.0 * math.sin(a))
                                 for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)]
            starts = [(x, y, body_z, cyaw + math.radians(d)) for x, y in ring for d in range(-30, 31, 15)]
        else:
            yaws = np.linspace(-math.pi, math.pi, N_YAW, endpoint=False)
            starts = [(c[0], c[1], c[2], y) for c in self.map.candidates for y in yaws]

        src = to_pcd(src_xyz)
        src_c = src.voxel_down_sample(0.3)
        src_f = src.voxel_down_sample(0.08)

        # 粗段：所有起点打分（按候选位置分组裁剪目标，避免重复构建）
        coarse = []
        groups = collections.defaultdict(list)
        for s in starts:
            groups[(round(s[0], 1), round(s[1], 1))].append(s)
        for (gx, gy), ss in groups.items():
            tgt = self.map.crop(np.array([gx, gy]), fine=False)
            if len(tgt.points) < 50:
                continue
            for x, y, z, yaw in ss:
                init = T_from(Rotation.from_euler("z", yaw).as_matrix() @ R_tilt, [x, y, z])
                r = reg.registration_icp(src_c, tgt, 1.0, init, P2PL,
                                         reg.ICPConvergenceCriteria(max_iteration=15))
                coarse.append((r.fitness, -r.inlier_rmse, r.transformation))
        if not coarse:
            return self.status(state="unlocalized", msg="候选位置附近没有地图点")
        coarse.sort(key=lambda c: (c[0], c[1]), reverse=True)

        # 精段：前 8 名
        fine = []
        for _, _, T0 in coarse[:8]:
            tgt = self.map.crop(T0[:3, 3], fine=True)
            r = reg.registration_icp(src_f, tgt, 0.3, T0, P2PL_TUKEY,
                                     reg.ICPConvergenceCriteria(max_iteration=40))
            fine.append((r.fitness, r.inlier_rmse, r.transformation))
        best_fit = max(f[0] for f in fine)
        ok = [f for f in fine if f[0] >= 0.7 * best_fit]
        fit, rmse, T_mb = min(ok, key=lambda f: f[1])

        # 歧义检查：与最优解相距较远、分数却接近的另一个解
        rival = None
        for f in fine:
            d = np.linalg.norm(f[2][:2, 3] - T_mb[:2, 3])
            dyaw = abs((yaw_of(f[2]) - yaw_of(T_mb) + math.pi) % (2 * math.pi) - math.pi)
            if (d > 1.0 or dyaw > math.radians(20)) and f[0] >= 0.9 * fit:
                rival = f
                break

        info = dict(fitness=round(fit, 3), rmse=round(rmse, 3), starts=len(starts),
                    time_s=round(time.time() - t0, 1),
                    pose=[round(T_mb[0, 3], 2), round(T_mb[1, 3], 2), round(yaw_of(T_mb), 3)])
        accepted = fit >= FIT_ACCEPT and rmse <= RMSE_ACCEPT and (rival is None or manual)
        if not accepted:
            why = "有多个相似位置，请在地图上点选大概位置" if rival is not None else "匹配度不够，请在地图上点选大概位置"
            return self.status(state="unlocalized", msg=why, **info)

        with self.lock:
            self.T_map_odom = T_mb @ np.linalg.inv(T_ob)
            self.fail = 0
        self.status(state="localized", msg="定位成功", **info)

    # ------------------------------------------------------------ 跟踪
    def track(self):
        src_xyz, T_ob = self.source_in_body(TRACK_SCANS)
        if src_xyz is None:
            return
        with self.lock:
            T_mo = self.T_map_odom
        T_pred = T_mo @ T_ob
        tgt = self.map.crop(T_pred[:3, 3], fine=True)
        r = reg.registration_icp(to_pcd(src_xyz).voxel_down_sample(0.08), tgt, 0.3, T_pred, P2PL_TUKEY,
                                 reg.ICPConvergenceCriteria(max_iteration=20))
        T_new = r.transformation
        jump = np.linalg.norm(T_new[:2, 3] - T_pred[:2, 3])
        jyaw = abs((yaw_of(T_new) - yaw_of(T_pred) + math.pi) % (2 * math.pi) - math.pi)
        good = (r.fitness >= FIT_ACCEPT and r.inlier_rmse <= RMSE_ACCEPT
                and jump < TRACK_MAX_JUMP[0] and jyaw < TRACK_MAX_JUMP[1])
        if good:
            with self.lock:
                self.T_map_odom = T_new @ np.linalg.inv(T_ob)
                self.fail = 0
            if self.state != "localized":
                self.status(state="localized", msg="定位恢复")
            self.track_info = (r.fitness, r.inlier_rmse)
        else:
            self.fail += 1
            if self.fail >= LOST_AFTER and self.state == "localized":
                self.status(state="lost", msg="定位丢失，已停车；请重新定位",
                            fitness=round(r.fitness, 3), rmse=round(r.inlier_rmse, 3))

    # ------------------------------------------------------------ 主循环
    def run(self):
        start = time.time()
        # 启动参数 _auto_reloc:=false 时不自动重定位，等网页手动点选（或点自动重定位）
        auto_done = not rospy.get_param("~auto_reloc", True)
        if auto_done:
            self.status(state="unlocalized", msg="请点「手动点选」在地图上指定车的位置，或点「自动重定位」")
        last_track = 0.0
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            req = None
            if self.requests:
                req = self.requests[0]
            elif not auto_done and time.time() - start > 3 and len(self.scans) >= RELOC_SCANS:
                req = {"mode": "auto"}
            if req is not None:
                self.busy = True
                try:
                    self.relocalize(req)
                finally:
                    self.requests.clear()
                    self.busy = False
                auto_done = True
            elif self.state in ("localized", "lost") and time.time() - last_track > TRACK_PERIOD:
                last_track = time.time()
                self.track()
            # 定期重发状态，便于网页刷新后取到
            if time.time() - self.last_status["t"] > 2 and self.state == "localized":
                info = getattr(self, "track_info", None)
                extra = dict(fitness=round(info[0], 3), rmse=round(info[1], 3)) if info else {}
                self.status(msg="跟踪中", **extra)
            rate.sleep()


def main():
    rospy.init_node("localizer")
    nav_dir = sys.argv[1] if len(sys.argv) > 1 else rospy.get_param("~nav_dir")
    Localizer(nav_dir).run()


if __name__ == "__main__":
    main()
