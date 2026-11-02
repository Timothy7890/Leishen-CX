# Leishen-CX

镭神 CX 32 线激光雷达 + FDILink IMU 的 **360° 录制与 FAST-LIO 实时建图**，独立于 `~/workspace` 下的原有工程。

## 快速使用

```bash
cd ~/yx/project/Leishen-CX
./record.sh start          # 开始录制（加 --rviz 可在车载显示器上看实时建图）
./record.sh status         # 查看状态
./record.sh stop           # 停止并保存
```

或在浏览器打开 `http://<车IP>:7010`，用页面上的按钮开始/停止。网页与命令行效果相同，可以混用。

录制中，网页右侧（手机上在按钮下方）实时显示正在建的俯视地图（约 10 帧/秒）：

- 颜色 = 高度（-1 ~ 1.5 m，FAST-LIO 世界坐标系），白点 = 当前一帧扫描，粉线 = 轨迹，白色箭头 = 车
- 拖动平移，滚轮 / 双指缩放；"跟随车辆" 让视图跟着车走，"全图" 缩放到整张地图
- 实时显示用的是原始点云 + FAST-LIO 位姿（5 cm 栅格），比保存的 `map.pcd`（0.5 m 降采样）更密
- 录制结束后保留最后的画面；刷新页面后清空

> 录制前请确认原有的 `slam.sh` / `run.sh` 等没有在运行（同一时间只能有一个 ROS master）。`record.sh start` 会自动检查。

## 输出

每次录制一个目录：`~/yx/Data/Lidar_data/<YYYYMMDD_HHMMSS>/`

| 文件 | 内容 |
|---|---|
| `raw.bag` | 原始数据 `/velodyne_points` + `/imu`，可离线重新建图 |
| `map.pcd` | FAST-LIO 建好的点云地图（世界坐标系 `camera_init`） |
| `trajectory.csv` | 车辆轨迹 `timestamp,x,y,z,qx,qy,qz,qw`（约 10 Hz） |
| `config/` | 本次使用的配置快照 |
| `logs/` | 各节点日志 |
| `info.txt` | 起止时间、时长、包大小、地图点数 |

## 目录结构

```
Leishen-CX/
├── record.sh                 录制控制（start / stop / status）
├── config/
│   ├── lidar_360.launch      雷达驱动，360° 不裁剪
│   ├── imu.launch            IMU 驱动（/dev/imu，无需 sudo）
│   └── fastlio_360.yaml      FAST-LIO 参数
├── scripts/
│   ├── record_tf.py          轨迹记录（camera_init -> body）
│   ├── live_map.py           实时俯视地图推送（127.0.0.1:7011，随录制启停）
│   └── build.sh              编译 catkin_ws
├── web/
│   ├── server.py             7010 端口服务：API + 静态页面 + 转发实时地图
│   ├── web.sh                网页服务 start / stop / restart / status / autostart
│   ├── frontend/             Vue 3 + Vite 源码
│   └── dist/                 前端构建产物（server.py 提供）
├── catkin_ws/src/            源码（从 ~/workspace 复制）
│   ├── lslidar_cx_driver     镭神 CX 驱动
│   ├── fdilink_ahrs          IMU 驱动
│   ├── FAST_LIO              建图
│   └── livox_ros_driver      仅消息定义（FAST-LIO 编译依赖）
└── run/                      运行时状态与网页日志（自动生成）
```

## 与原建图配置的区别

| 参数 | 原配置 | 本项目 |
|---|---|---|
| 雷达裁剪角度 `angle_disable_min/max` | 20000 / 34000（裁掉 200°–340°） | 0 / 0（不裁剪） |
| FAST-LIO `fov_degree` | 180 | 360 |
| FAST-LIO `blind`（近距离盲区） | 2 m | 0.5 m |
| FAST-LIO `dense_publish_en` | true（保存每帧全部点） | false（保存 0.5 m 降采样后的点） |
| FAST-LIO `timestamp_unit` | 2（微秒，错误） | 0（秒，与驱动输出一致） |

`timestamp_unit` 错误会使 FAST-LIO 的运动补偿失效，转弯后航向误差累积，往返路段的墙面出现重影。
2026-11-02 21:20 之前的录制受影响，可用其 `raw.bag` 离线重建。

`map.pcd` 为降采样地图，内存占用小，可长时间录制；需要高密度地图时用 `raw.bag` 离线重建。
原始数据包约 1.1 GB/分钟，注意剩余磁盘空间。

注意：360° 时跟在车后的人也会被扫进地图，建图时尽量离车远一些或走在侧面。

## 网页服务

```bash
web/web.sh start        # 启动（端口 7010）
web/web.sh status
web/web.sh autostart    # 开机自启（crontab @reboot，无需 sudo）
```

从 Mac 通过 SSH 访问：`ssh -L 7010:localhost:7010 Wangtao-613`，然后打开 `http://localhost:7010`。

## 重新编译

```bash
scripts/build.sh                     # ROS 部分
```

前端（修改 `web/frontend/src` 后）：

```bash
source ~/.nvm/nvm.sh
cd web/frontend
HTTPS_PROXY=http://127.0.0.1:17897 npm install   # 首次或依赖变化时；联网需走 17897 代理
npm run build                                    # 输出到 web/dist，刷新网页即生效
```

## 硬件

| 设备 | 连接 | 说明 |
|---|---|---|
| 镭神 CX 32 线 | 网口，雷达 IP 192.168.1.200 → 本机 192.168.1.102，UDP 2368/2369 | 本机 IP 不要改 |
| FDILink IMU | USB（CP2102）→ `/dev/imu`，921600 | udev 规则 `/etc/udev/rules.d/99-imu.rules` |
