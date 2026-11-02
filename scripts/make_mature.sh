#!/usr/bin/env bash
# ============================================================================
#  把一次录制处理成"成熟地图"（供 7011 导航使用）
#
#  用法: scripts/make_mature.sh <录制目录> <地图名称>
#
#  1. 用当前配置离线回放 raw.bag 跑 FAST-LIO（独立 ROS master 11312，不影响录制/导航），
#     记录位姿 -> nav/odom.bag
#  2. build_mature_map.py：原始点云 + 位姿 -> 高密度地图、2D 栅格、ICP 法线缓存
#  输出在 <录制目录>/nav/，进度写在 nav/progress.json
# ============================================================================
SESSION=$(cd "${1:?录制目录}" && pwd)
NAME=${2:?地图名称}
PROJECT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
NAV=$SESSION/nav
PROGRESS=$NAV/progress.json

PATH=$(echo "$PATH" | tr ':' '\n' | grep -viE 'conda' | paste -sd: -)
unset PYTHONPATH CONDA_PREFIX
source /opt/ros/noetic/setup.bash
source "$PROJECT_DIR/catkin_ws/devel/setup.bash"
export ROS_MASTER_URI=http://localhost:11312
export PYTHONUNBUFFERED=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1

progress() {
    printf '{"status": "%s", "step": "%s", "name": "%s", "updated": %s}\n' \
        "$1" "$2" "$NAME" "$(date +%s)" > "$PROGRESS"
    echo "[$(date +%H:%M:%S)] $2"
}

[[ -f $SESSION/raw.bag ]] || { echo "缺少 $SESSION/raw.bag" >&2; exit 1; }
if rostopic list > /dev/null 2>&1; then
    echo "端口 11312 上已有 ROS master，可能另一个成熟地图正在生成" >&2
    exit 1
fi

rm -rf "$NAV"
mkdir -p "$NAV/logs"
export ROS_LOG_DIR=$NAV/logs/ros
PIDS=()
cleanup() {
    for p in "${PIDS[@]}"; do kill -INT -- "-$p" 2> /dev/null; done
    sleep 2
    for p in "${PIDS[@]}"; do kill -KILL -- "-$p" 2> /dev/null; done
}
trap cleanup EXIT

progress running "离线重跑 FAST-LIO（约为录制时长的一半）"
setsid roscore -p 11312 > "$NAV/logs/roscore.log" 2>&1 < /dev/null &
PIDS+=($!)
for ((i = 0; i < 30; i++)); do rostopic list > /dev/null 2>&1 && break; sleep 0.5; done

# 与录制相同的配置，但不让 FAST-LIO 自己存图（避免和正在进行的录制争用 PCD/scans.pcd）
rosparam load "$PROJECT_DIR/config/fastlio_360.yaml"
rosparam set /pcd_save/pcd_save_en false
cp "$PROJECT_DIR/config/fastlio_360.yaml" "$NAV/fastlio.yaml"

setsid rosrun fast_lio fastlio_mapping > "$NAV/logs/fastlio.log" 2>&1 < /dev/null &
PIDS+=($!)
sleep 3
setsid rosbag record -O "$NAV/odom.bag" /Odometry > "$NAV/logs/rosbag.log" 2>&1 < /dev/null &
REC=$!
PIDS+=($REC)
sleep 2
# rosbag play 会把 stdin 当键盘（空格=暂停），必须断开
rosbag play -q -r 2 "$SESSION/raw.bag" < /dev/null > "$NAV/logs/play.log" 2>&1 || {
    progress failed "回放失败"; exit 1; }
sleep 5
kill -INT -- "-$REC"
for ((i = 0; i < 20; i++)); do kill -0 "$REC" 2> /dev/null || break; sleep 0.5; done
cleanup
PIDS=()

progress running "生成高密度地图、2D 栅格和定位数据"
if python3 "$PROJECT_DIR/scripts/build_mature_map.py" "$SESSION" "$NAME" > "$NAV/logs/build.log" 2>&1; then
    progress done "完成"
else
    progress failed "地图生成失败，见 nav/logs/build.log"
    exit 1
fi
