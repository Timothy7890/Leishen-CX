"""车辆几何：雷达外参与车身轮廓，唯一来源是配置文件。

    外参   config/fastlio_360.yaml    mapping/extrinsic_R、extrinsic_T：p_body = R @ p_lidar + T
    车身   config/nav/costmap_common.yaml  footprint（body 系 xy，与 move_base 一致）

车尾（雷达后方约 0.55~0.7 m、离地约 0.3 m 的两处部件）会被最下面两条激光线扫到，
这些点跟着车走，必须用 drop_self() 去掉，否则会在地图上沿行驶路线留下一层假障碍。
"""
from pathlib import Path

import numpy as np
import yaml

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"
SELF_MARGIN = 0.05          # 车身轮廓外扩该距离内的点也视为车自身


def _load():
    with open(CONFIG_DIR / "fastlio_360.yaml") as f:
        m = yaml.safe_load(f)["mapping"]
    with open(CONFIG_DIR / "nav/costmap_common.yaml") as f:
        fp = np.array(yaml.safe_load(f)["footprint"], float)
    return np.array(m["extrinsic_R"], float).reshape(3, 3), np.array(m["extrinsic_T"], float), fp


R, T, FOOTPRINT = _load()
FRONT, REAR = FOOTPRINT[:, 0].max(), FOOTPRINT[:, 0].min()
HALF_WID = np.abs(FOOTPRINT[:, 1]).max()
SWEEP_R = float(np.hypot(FOOTPRINT[:, 0], FOOTPRINT[:, 1]).max())   # 原地转向时车角扫过的半径


def lidar_to_body(xyz):
    return xyz @ R.T + T


def inside_body(x, y, margin=SELF_MARGIN):
    """body 系 xy 是否落在车身轮廓（矩形）外扩 margin 以内。"""
    return (x > REAR - margin) & (x < FRONT + margin) & (np.abs(y) < HALF_WID + margin)


def drop_self(xyz_lidar):
    """去掉车身轮廓内的点，返回 body 系坐标。"""
    b = lidar_to_body(xyz_lidar)
    return b[~inside_body(b[:, 0], b[:, 1])]
