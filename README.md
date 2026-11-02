# Leishen-CX

镭神 CX 32 线激光雷达 + FDILink IMU 的 **360° 录制、FAST-LIO 建图与地图内导航**，独立于 `~/workspace` 下的原有工程。

- `http://<车IP>:7010` 录制：开始/停止录制、实时地图、把录制设为成熟地图
- `http://<车IP>:7011` 导航：选择成熟地图、自动/手动重定位、点选目标、急停

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
| `nav/` | 成熟地图（见下） |

## 成熟地图

在 7010 页面录制列表点"设为成熟地图"并命名（或 `scripts/make_mature.sh <录制目录> <名称>`），
后台用当前配置离线重跑 `raw.bag`（独立 ROS master 11312，可与录制/导航同时进行），耗时约为录制时长的 1~1.5 倍。
旧录制会顺带修正时间单位问题。已有成熟地图的录制可点"重新生成"（需确认）：新图先生成到 `nav.tmp/`，
成功后才替换 `nav/`，生成期间和失败时原地图照常可用；进度在 `<录制目录>/mature_progress.json`。输出在 `<录制目录>/nav/`：

| 文件 | 内容 |
|---|---|
| `map_dense.pcd` | 5 cm 体素高密度地图（逐点去畸变） |
| `grid.pgm/.yaml` | 2D 占据栅格，供路径规划；地面以上 0.15~1.6 m 的静态物体为障碍 |
| `reloc.npz` | 重定位用的降采样点 + 法线 |
| `traj.csv` | 建图轨迹 `t,x,y,z,yaw` |
| `meta.json` | 名称、地面高度、栅格参数 |
| `fastlio_360.yaml` / `costmap_common.yaml` | 生成时使用的配置快照 |

栅格去噪：只用被扫到 ≥5 次的点（排除建图时跟车的人），去掉孤立噪点，建图轨迹两侧 0.35 m 内强制可通行。

## 导航（7011）

```bash
./nav.sh start ~/yx/Data/Lidar_data/<录制> [--dry-run]   # 一般直接用网页
./nav.sh stop
```

流程：选成熟地图 → 启动导航（约 20 s）→ 自动重定位（约 10 s）→ "点选目标"：在地图上按下定位置、拖动定朝向 → 车自动前往。

- **重定位**：本车无相机，沿建图轨迹每 1.5 m × 12 个朝向撒点做多起点 ICP（方法参考 D5G-beacon）。
  有多个相似位置或匹配度不够时提示手动：点"手动点选"，在地图上点车的大概位置并拖出车头朝向。
- **持续定位**：FAST-LIO 里程计 + 每秒一次与地图 ICP 校正；连续 5 次失败判为丢失并停车。
- **规划**：move_base（GlobalPlanner + DWA），只在建图时确认可通行的区域内规划，不倒车。
- **遇障停车（不绕行）**：`scripts/safety_gate.py` 检查实时雷达，前进时车头前方 0.5 m + 1 s×速度
  （宽度为车宽每侧外扩 3 cm）、原地转向时车角扫过的圆（外扩 0.1 m）内有障碍即停车，障碍消失 2 s 后自动继续。
- **安全**：网页急停（空格键也可）；限速默认 0.3 m/s（网页可调，上限 0.6）；
  定位丢失、网页心跳中断 5 s（关闭页面/手机息屏）、雷达中断时自动停车；底盘串口 0.5 s 无指令发零速。
- **空跑**：勾选"空跑"则整套流程运行但不向 `/dev/car` 发送，用于检查规划路径。
- **断电恢复**：`nav.sh` / `record.sh` 发现状态文件早于本次开机（或进程已全部退出）时自动清理，可直接重新启动；
  被中断的录制在 `info.txt` 标记为失败（`raw.bag.active` 可用 `rosbag reindex` 修复）。

实车前提：小车面板切到 **ROS 控制**；`/dev/car` 需可读写（一次性执行）：

```bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="car", MODE="0666"' | sudo tee /etc/udev/rules.d/99-car.rules && sudo udevadm control --reload && sudo udevadm trigger
```

## 目录结构

```
Leishen-CX/
├── record.sh                 录制控制（start / stop / status）
├── nav.sh                    导航控制（start <录制> [--dry-run] / stop / status）
├── config/
│   ├── lidar_360.launch      雷达驱动，360° 不裁剪
│   ├── imu.launch            IMU 驱动（/dev/imu，无需 sudo）
│   ├── fastlio_360.yaml      FAST-LIO 参数
│   └── nav/                  move_base 与代价地图参数
├── scripts/
│   ├── record_tf.py          轨迹记录（camera_init -> body）
│   ├── live_map.py           录制实时地图推送（127.0.0.1:7012）
│   ├── make_mature.sh        成熟地图：离线重跑 FAST-LIO
│   ├── build_mature_map.py   成熟地图：高密度地图、2D 栅格、重定位数据
│   ├── localizer.py          重定位 + 持续定位（发布 TF map -> camera_init）
│   ├── safety_gate.py        安全门：急停、心跳、遇障停车、限速
│   ├── car_serial.py         /cmd_vel -> /dev/car（Modbus，协议同原 cmd_vel_to_serial.py）
│   ├── nav_bridge.py         导航 ROS <-> 网页（127.0.0.1:7013）
│   └── build.sh              编译 catkin_ws
├── web/
│   ├── server.py             7010：录制 API + 页面 + 转发实时地图
│   ├── nav_server.py         7011：导航 API + 页面 + 转发 nav_bridge
│   ├── webcommon.py          两个服务共用的 HTTP 工具
│   ├── web.sh                网页服务 start / stop / restart / status / autostart（两个端口）
│   ├── frontend/             Vue 3 + Vite 源码（index.html 录制，nav.html 导航）
│   └── dist/                 前端构建产物
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
| FAST-LIO `blind`（近距离盲区） | 2 m | 0.8 m（去掉车尾自身，见下） |
| FAST-LIO `dense_publish_en` | true（保存每帧全部点） | false（保存 0.5 m 降采样后的点） |
| FAST-LIO `timestamp_unit` | 2（微秒，错误） | 0（秒，与驱动输出一致） |

`timestamp_unit` 错误会使 FAST-LIO 的运动补偿失效，转弯后航向误差累积，往返路段的墙面出现重影。
2026-11-02 21:20 之前的录制受影响，可用其 `raw.bag` 离线重建。

`map.pcd` 为降采样地图，内存占用小，可长时间录制；需要高密度地图时用 `raw.bag` 离线重建。
原始数据包约 1.1 GB/分钟，注意剩余磁盘空间。

注意：360° 时跟在车后的人也会被扫进地图，建图时尽量离车远一些或走在侧面。

**车自身遮挡**：车尾（雷达后方 0.68 m）会被最下面几条激光线扫到。
车身实测相对雷达中心：前 +0.16、后 -0.68、左右各 0.28 m（雷达离地约 0.45 m）。`config/nav/costmap_common.yaml` 中
`footprint` 以车体几何中心 base_link 为原点（±0.42 × ±0.28 m），`base_link_in_body: [-0.26, 0]` 为它相对雷达/IMU 的位置；
`scripts/vehicle.py` 读取它们和外参，建图、定位、实时地图、安全门都去掉轮廓内的点；FAST-LIO 只能按半径过滤，用 `blind: 0.8`。
注意 `extrinsic_T` 是雷达相对 IMU 的位置（0.28 m），不是雷达离地高度。

## 网页服务

```bash
web/web.sh start        # 启动（端口 7010 + 7011）
web/web.sh status
web/web.sh autostart    # 开机自启（crontab @reboot，无需 sudo）
```

从 Mac 通过 SSH 访问：`ssh -L 7010:localhost:7010 -L 7011:localhost:7011 Wangtao-613`，
然后打开 `http://localhost:7010` / `http://localhost:7011`。

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
| 小车底盘 | USB（CH340）→ `/dev/car`，115200，Modbus RTU | udev 规则 `/etc/udev/rules.d/99-car.rules`；车身约 0.84 × 0.56 m（雷达高 0.45 m） |
