#!/usr/bin/env python3
"""由 raw.bag + 离线 FAST-LIO 位姿（nav/odom.bag）生成成熟地图。

用法: build_mature_map.py <录制目录> <地图名称> [输出目录，默认 <录制目录>/nav]

输出目录中需已有离线 FAST-LIO 位姿 odom.bag（make_mature.sh 生成）。输出:
    map_dense.pcd   5 cm 体素的高密度地图（逐点去畸变）
    grid.pgm/.yaml  2D 占据栅格（map_server 格式，供 move_base）
    grid.png        网页显示用（RGBA）
    reloc.npz       ICP 用的降采样点 + 法线（粗 0.3 m / 细 0.08 m）
    traj.csv        建图轨迹 t,x,y,z,yaw（地图坐标系）
    meta.json       名称、地面高度、栅格参数等
"""
import os

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import json
import sys
import time

import cv2
import numpy as np
import rosbag
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

from vehicle import inside_body, lidar_to_body

VOXEL = 0.05                # 高密度地图体素 (m)
GRID_RES = 0.05             # 2D 栅格分辨率 (m)
RANGE_MIN, RANGE_MAX = 0.5, 40.0
MIN_HITS = 2                # 体素至少被扫到 2 次才保留（去掉走动的人等动态噪点）
OBST_LOW, OBST_HIGH = 0.15, 1.6   # 地面以上该高度区间内的点视为障碍
OBST_MIN_HITS = 5           # 栅格障碍只用被扫到 ≥5 次的体素：墙会被扫到上百次，路过的人只有几次
SPECKLE_CELLS = 3           # 小于该面积（格）的孤立障碍视为噪点
# 建图轨迹两侧该半径内清除障碍：车开过的地方必然可通行（取车半宽）
SWEPT_CLEAR_R = 0.4
OFF = 1 << 20               # 体素索引偏移，便于编码成一个 int64


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def load_odom(path):
    t, p, q = [], [], []
    for _, m, _ in rosbag.Bag(path).read_messages(topics=["/Odometry"]):
        t.append(m.header.stamp.to_sec())
        pp, qq = m.pose.pose.position, m.pose.pose.orientation
        p.append([pp.x, pp.y, pp.z])
        q.append([qq.x, qq.y, qq.z, qq.w])
    t = np.array(t)
    order = np.argsort(t)
    t, p, q = t[order], np.array(p)[order], np.array(q)[order]
    keep = np.r_[True, np.diff(t) > 1e-6]
    return t[keep], p[keep], Rotation.from_quat(q[keep])


def encode(ix, iy, iz):
    return ((ix + OFF) << 42) | ((iy + OFF) << 21) | (iz + OFF)


def decode(k):
    return np.stack([(k >> 42) - OFF, ((k >> 21) & (2 * OFF - 1)) - OFF, (k & (2 * OFF - 1)) - OFF], 1)


def merge(keys, counts, new_keys):
    allk = np.concatenate([keys, new_keys])
    allc = np.concatenate([counts, np.ones(len(new_keys), np.int64)])
    uk, inv = np.unique(allk, return_inverse=True)
    return uk, np.bincount(inv, weights=allc).astype(np.int64)


def build_dense(bag_path, ot, op, orot):
    slerp = Slerp(ot, orot)
    keys, counts = np.empty(0, np.int64), np.empty(0, np.int64)
    batch, n_scan = [], 0
    for _, msg, _ in rosbag.Bag(bag_path).read_messages(topics=["/velodyne_points"]):
        off = {f.name: f.offset for f in msg.fields}
        dt = np.dtype({"names": ["x", "y", "z", "t"], "formats": ["<f4"] * 4,
                       "offsets": [off["x"], off["y"], off["z"], off["time"]], "itemsize": msg.point_step})
        pts = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
        xyz = np.stack([pts["x"], pts["y"], pts["z"]], 1).astype(np.float64)
        r = np.linalg.norm(xyz, axis=1)
        ok = np.isfinite(r) & (r > RANGE_MIN) & (r < RANGE_MAX)
        b = lidar_to_body(xyz[ok])
        ta = msg.header.stamp.to_sec() + pts["t"][ok].astype(np.float64)
        # 去掉车身轮廓内的点（车尾部件）和位姿范围外的点
        keep = (ta >= ot[0]) & (ta <= ot[-1]) & ~inside_body(b[:, 0], b[:, 1])
        if keep.sum() < 100:
            continue
        b, ta = b[keep], ta[keep]
        # 逐点按扫描时刻插值位姿（去畸变）
        R = slerp(ta)
        trans = np.stack([np.interp(ta, ot, op[:, i]) for i in range(3)], 1)
        w = R.apply(b) + trans
        iv = np.floor(w / VOXEL).astype(np.int64)
        batch.append(np.unique(encode(iv[:, 0], iv[:, 1], iv[:, 2])))
        n_scan += 1
        if len(batch) >= 50:
            keys, counts = merge(keys, counts, np.concatenate(batch))
            batch = []
            log(f"  已处理 {n_scan} 帧，体素 {len(keys):,}")
    if batch:
        keys, counts = merge(keys, counts, np.concatenate(batch))
    keep = counts >= MIN_HITS
    log(f"共 {n_scan} 帧，保留体素 {keep.sum():,}（≥{MIN_HITS} 次）")
    return (decode(keys[keep]) + 0.5) * VOXEL, counts[keep]


def write_pcd(path, xyz):
    xyz = np.ascontiguousarray(xyz, dtype="<f4")
    hdr = ("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
           f"WIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n")
    with open(path, "wb") as f:
        f.write(hdr.encode())
        f.write(xyz.tobytes())


def estimate_floor(xyz, traj_xyz):
    """取轨迹附近、低于车体的点，高度直方图的峰值即地面。"""
    tree = cKDTree(traj_xyz[:, :2])
    d, _ = tree.query(xyz[:, :2], distance_upper_bound=2.0)
    near = xyz[(d < 2.0) & (xyz[:, 2] < np.median(traj_xyz[:, 2]))]
    hist, edges = np.histogram(near[:, 2], bins=np.arange(near[:, 2].min(), near[:, 2].max() + 0.02, 0.02))
    i = int(np.argmax(hist))
    return float((edges[i] + edges[i + 1]) / 2)


def build_grid(xyz, hits, floor, traj_xyz):
    x0, y0 = xyz[:, 0].min() - 1.0, xyz[:, 1].min() - 1.0
    W = int(np.ceil((xyz[:, 0].max() + 1.0 - x0) / GRID_RES))
    H = int(np.ceil((xyz[:, 1].max() + 1.0 - y0) / GRID_RES))
    ix = ((xyz[:, 0] - x0) / GRID_RES).astype(int)
    iy = ((xyz[:, 1] - y0) / GRID_RES).astype(int)
    h = xyz[:, 2] - floor

    obst = np.zeros((H, W), np.int32)
    m = (h > OBST_LOW) & (h < OBST_HIGH) & (hits >= OBST_MIN_HITS)
    np.add.at(obst, (iy[m], ix[m]), 1)
    ground = np.zeros((H, W), np.uint8)
    m = np.abs(h) < 0.1
    ground[iy[m], ix[m]] = 1

    occupied = (obst >= 2).astype(np.uint8)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(occupied, connectivity=8)
    small = np.where(stats[:, cv2.CC_STAT_AREA] < SPECKLE_CELLS)[0]
    occupied[np.isin(labels, small[small > 0])] = 0

    # 车走过的地方一定可通行：轨迹画成折线后膨胀
    tx = ((traj_xyz[:, 0] - x0) / GRID_RES).astype(np.int32)
    ty = ((traj_xyz[:, 1] - y0) / GRID_RES).astype(np.int32)
    line = np.zeros((H, W), np.uint8)
    cv2.polylines(line, [np.stack([tx, ty], 1).reshape(-1, 1, 2)], False, 1, 1)
    disk = lambda r: cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * int(r / GRID_RES) + 1,) * 2)
    swept_clear = cv2.dilate(line, disk(SWEPT_CLEAR_R)).astype(bool)
    occupied = occupied.astype(bool) & ~swept_clear

    # 地面点稀疏处补齐（雷达线间空隙），加上车身扫过的范围，再扣掉障碍
    free = cv2.morphologyEx(ground, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)).astype(bool)
    free |= cv2.dilate(line, disk(0.45)).astype(bool)
    free &= ~occupied

    img = np.full((H, W), 205, np.uint8)       # 未知
    img[free] = 254
    img[occupied] = 0
    return np.flipud(img), (float(x0), float(y0)), int(occupied.sum()), int(free.sum())


def build_reloc(xyz, out):
    import open3d as o3d
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    res = {}
    for name, voxel, radius in (("fine", 0.08, 0.5), ("coarse", 0.3, 0.6)):
        d = pc.voxel_down_sample(voxel)
        d.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=20))
        res[f"{name}_pts"] = np.asarray(d.points, np.float32)
        res[f"{name}_nrm"] = np.asarray(d.normals, np.float32)
        log(f"  ICP {name} 点数 {len(d.points):,}")
    np.savez_compressed(out, **res)


def main():
    session, name = sys.argv[1], sys.argv[2]
    nav = sys.argv[3] if len(sys.argv) > 3 else os.path.join(session, "nav")
    ot, op, orot = load_odom(os.path.join(nav, "odom.bag"))
    if len(ot) < 20:
        raise SystemExit("FAST-LIO 位姿太少，回放可能失败")
    log(f"位姿 {len(ot)} 个，时长 {ot[-1] - ot[0]:.1f} s")

    yaw = orot.as_euler("zyx")[:, 0]
    np.savetxt(os.path.join(nav, "traj.csv"), np.column_stack([ot, op, yaw]), delimiter=",",
               header="t,x,y,z,yaw", comments="", fmt=["%.6f", "%.4f", "%.4f", "%.4f", "%.5f"])

    log("生成高密度地图")
    xyz, hits = build_dense(os.path.join(session, "raw.bag"), ot, op, orot)
    write_pcd(os.path.join(nav, "map_dense.pcd"), xyz)

    floor = estimate_floor(xyz, op)
    log(f"地面高度 z = {floor:.3f}（车体坐标原点在地面以上 {np.median(op[:, 2]) - floor:.2f} m）")

    log("生成 2D 栅格")
    img, origin, n_occ, n_free = build_grid(xyz, hits, floor, op)
    cv2.imwrite(os.path.join(nav, "grid.pgm"), img)
    with open(os.path.join(nav, "grid.yaml"), "w") as f:
        f.write(f"image: grid.pgm\nresolution: {GRID_RES}\norigin: [{origin[0]:.3f}, {origin[1]:.3f}, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    rgba = np.zeros(img.shape + (4,), np.uint8)
    rgba[img == 254] = (235, 237, 240, 255)
    rgba[img == 0] = (40, 44, 52, 255)
    cv2.imwrite(os.path.join(nav, "grid.png"), cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA))

    log("生成 ICP 定位数据")
    build_reloc(xyz, os.path.join(nav, "reloc.npz"))

    path_len = float(np.sum(np.linalg.norm(np.diff(op[:, :2], axis=0), axis=1)))
    meta = {
        "name": name,
        "session": os.path.basename(session.rstrip("/")),
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        "floor_z": round(floor, 3),
        "grid": {"resolution": GRID_RES, "origin": [round(origin[0], 3), round(origin[1], 3)],
                 "width": img.shape[1], "height": img.shape[0], "occupied": n_occ, "free": n_free},
        "dense_points": len(xyz),
        "path_length_m": round(path_len, 1),
    }
    with open(os.path.join(nav, "meta.json"), "w") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    log(f"完成：{json.dumps(meta, ensure_ascii=False)}")


if __name__ == "__main__":
    main()
