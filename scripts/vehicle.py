"""车辆与传感器配置，唯一来源是配置文件。

    传感器  config/sensor                       当前使用的传感器名（每台车本地设置，不提交），缺省 cx32；
                                                环境变量 LEISHEN_SENSOR 优先（成熟地图按录制时的传感器生成）
            config/sensors/<名称>/sensor.yaml   见下
            config/sensors/<名称>/fastlio.yaml  mapping/extrinsic_R、extrinsic_T：p_body = R @ p_lidar + T
    车身    config/nav/costmap_common.yaml      footprint 以 base_link（车体几何中心）为原点，与 move_base 一致

sensor.yaml 字段：
    lidar_ip            启动前 ping 检查
    lidar_topic         雷达点云（PointCloud2，雷达坐标系），录进 raw.bag，各模块订阅
    imu_topic           IMU，录进 raw.bag
    launch              依次启动的驱动 launch 文件（同目录）
    devices             启动前检查存在的设备文件
    base_link_in_body   base_link 在 body 系的 xy 位置（随雷达安装位置而变）
    chassis_reverse     车头与底盘前进方向相反时为 true：线速度取反后发给底盘
    floor_z             body 系下的地面高度 (m)；null 时安全门启动后自动标定

车尾等车身部件可能被最下面几条激光线扫到，这些点跟着车走，
必须用 drop_self() 去掉，否则会在地图上沿行驶路线留下一层假障碍。

`python3 vehicle.py --shell` 输出 shell 变量，供 record.sh / nav.sh / make_mature.sh 使用。
"""
import os
import shlex
import sys
from pathlib import Path

import numpy as np
import yaml

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"
SELF_MARGIN = 0.05          # 车身轮廓外扩该距离内的点也视为车自身


def _sensor_name():
    name = os.environ.get("LEISHEN_SENSOR", "").strip()
    if not name:
        try:
            name = (CONFIG_DIR / "sensor").read_text().strip()
        except OSError:
            pass
    return name or "cx32"


SENSOR = _sensor_name()
SENSOR_DIR = CONFIG_DIR / "sensors" / SENSOR
if not (SENSOR_DIR / "sensor.yaml").is_file():
    sys.exit(f"未知传感器 {SENSOR}：没有 {SENSOR_DIR}/sensor.yaml")


def _load():
    with open(SENSOR_DIR / "sensor.yaml") as f:
        s = yaml.safe_load(f)
    with open(SENSOR_DIR / "fastlio.yaml") as f:
        m = yaml.safe_load(f)["mapping"]
    with open(CONFIG_DIR / "nav/costmap_common.yaml") as f:
        c = yaml.safe_load(f)
    return (s, np.array(m["extrinsic_R"], float).reshape(3, 3), np.array(m["extrinsic_T"], float),
            np.array(c["footprint"], float))


SPEC, R, T, FOOTPRINT = _load()
LIDAR_TOPIC, IMU_TOPIC = SPEC["lidar_topic"], SPEC["imu_topic"]
BASE_XY = np.array(SPEC["base_link_in_body"], float)
CHASSIS_REVERSE = bool(SPEC.get("chassis_reverse", False))
FLOOR_Z = SPEC.get("floor_z")
FOOTPRINT_BODY = FOOTPRINT + BASE_XY                 # body 系下的车身轮廓
FRONT, REAR = FOOTPRINT_BODY[:, 0].max(), FOOTPRINT_BODY[:, 0].min()
HALF_WID = np.abs(FOOTPRINT_BODY[:, 1]).max()
# 原地转向绕 base_link（几何中心），车角扫过的半径
SWEEP_R = float(np.hypot(FOOTPRINT[:, 0], FOOTPRINT[:, 1]).max())


def lidar_to_body(xyz):
    return xyz @ R.T + T


def inside_body(x, y, margin=SELF_MARGIN):
    """body 系 xy 是否落在车身轮廓（矩形）外扩 margin 以内。"""
    return (x > REAR - margin) & (x < FRONT + margin) & (np.abs(y) < HALF_WID + margin)


def drop_self(xyz_lidar):
    """去掉车身轮廓内的点，返回 body 系坐标。"""
    b = lidar_to_body(xyz_lidar)
    return b[~inside_body(b[:, 0], b[:, 1])]


def base_xy(x, y, yaw):
    """body 位姿 -> base_link 在同一坐标系下的 xy。"""
    c, s = np.cos(yaw), np.sin(yaw)
    return x + c * BASE_XY[0] - s * BASE_XY[1], y + s * BASE_XY[0] + c * BASE_XY[1]


if __name__ == "__main__" and sys.argv[1:] == ["--shell"]:
    q = shlex.quote
    print(f"SENSOR={q(SENSOR)}")
    print(f"SENSOR_DIR={q(str(SENSOR_DIR))}")
    print(f"LIDAR_IP={q(SPEC['lidar_ip'])}")
    print(f"LIDAR_TOPIC={q(LIDAR_TOPIC)}")
    print(f"IMU_TOPIC={q(IMU_TOPIC)}")
    print(f"SENSOR_LAUNCH=({' '.join(q(x) for x in SPEC['launch'])})")
    print(f"SENSOR_DEVICES=({' '.join(q(x) for x in SPEC.get('devices') or [])})")
    print(f"BASE_XY={q(' '.join(str(v) for v in BASE_XY))}")
