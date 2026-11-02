#!/usr/bin/env bash
# ============================================================================
#  镭神 CX 360° 录制 + FAST-LIO 实时建图
#
#  用法:
#    ./record.sh start [--rviz]   开始录制（--rviz 在车载显示器上打开 rviz）
#    ./record.sh stop             停止录制，保存地图、轨迹、原始包
#    ./record.sh status           查看录制状态
#
#  每次录制输出到 $DATA_ROOT/<开始时间>/:
#    raw.bag  map.pcd  trajectory.csv  config/  logs/  info.txt
# ============================================================================

PROJECT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DATA_ROOT=${LIDAR_DATA_ROOT:-$HOME/yx/Data/Lidar_data}
RUN_DIR=$PROJECT_DIR/run              # 运行时状态（当前录制、各进程 PID）
STATE_FILE=$RUN_DIR/state
PID_FILE=$RUN_DIR/pids
LOCK_DIR=$RUN_DIR/lock
LIDAR_IP=192.168.1.200
FASTLIO_PCD=$PROJECT_DIR/catkin_ws/src/FAST_LIO/PCD/scans.pcd

mkdir -p "$RUN_DIR" "$DATA_ROOT"

# ---------------------------------------------------------------- 环境
# 去掉 conda，避免 rospy 节点用到 conda 里的 python
PATH=$(echo "$PATH" | tr ':' '\n' | grep -viE 'conda' | paste -sd: -)
unset PYTHONPATH CONDA_PREFIX
source /opt/ros/noetic/setup.bash
source "$PROJECT_DIR/catkin_ws/devel/setup.bash"
export ROS_MASTER_URI=http://localhost:11311

log() { echo "[$(date +%H:%M:%S)] $*"; }

state_get() { [[ -f $STATE_FILE ]] && grep -m1 "^$1=" "$STATE_FILE" | cut -d= -f2-; }
state_set() {
    touch "$STATE_FILE"
    sed -i "/^$1=/d" "$STATE_FILE"
    echo "$1=$2" >> "$STATE_FILE"
}

# 获取互斥锁，防止网页和命令行同时操作
lock() {
    if ! mkdir "$LOCK_DIR" 2>/dev/null; then
        echo "另一个 start/stop 操作正在进行，请稍后再试" >&2
        exit 2
    fi
    trap 'rmdir "$LOCK_DIR" 2>/dev/null' EXIT
}

# 以独立进程组启动，便于整组发送 SIGINT
launch() {
    local name=$1; shift
    setsid "$@" > "$SESSION/logs/$name.log" 2>&1 < /dev/null &
    echo "$name $!" >> "$PID_FILE"
}

pid_of() { [[ -f $PID_FILE ]] && awk -v n="$1" '$1 == n {print $2}' "$PID_FILE"; }

# 向进程组发 SIGINT，最多等待 $2 秒，超时则 SIGKILL
stop_proc() {
    local name=$1 timeout=${2:-15} pid
    pid=$(pid_of "$name")
    [[ -z $pid ]] && return 0
    kill -0 "$pid" 2>/dev/null || return 0
    kill -INT -- "-$pid" 2>/dev/null
    for ((i = 0; i < timeout * 2; i++)); do
        kill -0 "$pid" 2>/dev/null || return 0
        sleep 0.5
    done
    log "  $name 未在 ${timeout}s 内退出，强制结束"
    kill -KILL -- "-$pid" 2>/dev/null
}

wait_topic() {
    local topic=$1 timeout=$2
    timeout "$timeout" rostopic echo -n 1 "$topic/header/stamp" > /dev/null 2>&1
}

is_recording() { [[ $(state_get status) == recording || $(state_get status) == stopping ]]; }

# ---------------------------------------------------------------- stop（内部）
shutdown_all() {
    # 先停 rosbag 保证包完整，再停 FAST-LIO 触发保存地图，最后停驱动和 roscore
    stop_proc rosbag 30
    stop_proc live 5
    stop_proc trajectory 10
    stop_proc fastlio 300
    stop_proc rviz 5
    stop_proc imu 10
    stop_proc lidar 10
    stop_proc roscore 10
    rm -f "$PID_FILE"
}

# ---------------------------------------------------------------- start
cmd_start() {
    local rviz=false
    [[ ${1:-} == --rviz ]] && rviz=true
    lock

    if is_recording; then
        echo "已在录制中: $(state_get session)" >&2
        exit 1
    fi
    if rostopic list > /dev/null 2>&1; then
        echo "检测到已有 ROS 在运行（可能是 ~/workspace 下的脚本），请先关闭" >&2
        exit 1
    fi
    if ! ping -c 1 -W 1 "$LIDAR_IP" > /dev/null 2>&1; then
        echo "雷达 $LIDAR_IP 无法连通，请检查网线和电源" >&2
        exit 1
    fi
    if [[ ! -e /dev/imu ]]; then
        echo "未找到 IMU 设备 /dev/imu，请检查 USB 连接" >&2
        exit 1
    fi

    SESSION="$DATA_ROOT/$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$SESSION/logs" "$SESSION/config"
    cp "$PROJECT_DIR"/config/* "$SESSION/config/"
    export ROS_LOG_DIR="$SESSION/logs/ros"
    rm -f "$PID_FILE" "$STATE_FILE" "$FASTLIO_PCD"

    state_set status starting
    state_set session "$SESSION"
    state_set started "$(date +%s)"
    log "录制目录: $SESSION"

    fail() {
        log "启动失败: $1（日志见 $SESSION/logs/）"
        shutdown_all
        state_set status idle
        echo "status=failed" >> "$SESSION/info.txt"
        echo "error=$1" >> "$SESSION/info.txt"
        rm -f "$STATE_FILE"
        exit 1
    }

    log "启动 roscore"
    launch roscore roscore
    for ((i = 0; i < 30; i++)); do rostopic list > /dev/null 2>&1 && break; sleep 0.5; done
    rostopic list > /dev/null 2>&1 || fail "roscore 未启动"

    log "启动雷达驱动（360° 不裁剪）"
    launch lidar roslaunch "$PROJECT_DIR/config/lidar_360.launch"
    wait_topic /velodyne_points 20 || fail "未收到雷达点云 /velodyne_points"

    log "启动 IMU 驱动"
    launch imu roslaunch "$PROJECT_DIR/config/imu.launch"
    wait_topic /imu 15 || fail "未收到 IMU 数据 /imu"

    log "启动 FAST-LIO 实时建图"
    rosparam load "$PROJECT_DIR/config/fastlio_360.yaml"
    launch fastlio rosrun fast_lio fastlio_mapping
    wait_topic /Odometry 20 || fail "FAST-LIO 未输出里程计 /Odometry"

    log "启动网页实时地图推送"
    launch live nice -n 10 python3 "$PROJECT_DIR/scripts/live_map.py"

    log "启动轨迹记录"
    launch trajectory python3 "$PROJECT_DIR/scripts/record_tf.py" "_output_file:=$SESSION/trajectory.csv"

    log "启动原始数据录制"
    launch rosbag rosbag record -O "$SESSION/raw.bag" /velodyne_points /imu

    if $rviz; then
        DISPLAY=${DISPLAY:-:0} launch rviz rosrun rviz rviz -d "$PROJECT_DIR/catkin_ws/src/FAST_LIO/rviz_cfg/loam_livox.rviz"
    fi

    state_set status recording
    log "录制中。停止请运行: $0 stop"
}

# ---------------------------------------------------------------- stop
cmd_stop() {
    lock
    if ! is_recording; then
        echo "当前没有在录制" >&2
        exit 1
    fi
    SESSION=$(state_get session)
    local started end
    started=$(state_get started)
    state_set status stopping
    log "停止录制，正在保存（地图较大时需等待一段时间）..."

    shutdown_all
    end=$(date +%s)

    if [[ -f $FASTLIO_PCD ]]; then
        mv "$FASTLIO_PCD" "$SESSION/map.pcd"
    else
        log "警告: FAST-LIO 未生成地图文件"
    fi

    local points traj_rows bag_size
    points=$(grep -a -m1 '^POINTS' "$SESSION/map.pcd" 2>/dev/null | awk '{print $2}')
    traj_rows=$(($(wc -l < "$SESSION/trajectory.csv" 2>/dev/null || echo 1) - 1))
    bag_size=$(du -h "$SESSION/raw.bag" 2>/dev/null | cut -f1)
    cat > "$SESSION/info.txt" << EOF
status=done
start=$(date -d "@$started" '+%Y-%m-%d %H:%M:%S')
end=$(date -d "@$end" '+%Y-%m-%d %H:%M:%S')
duration_s=$((end - started))
bag_size=${bag_size:-无}
map_points=${points:-0}
trajectory_poses=$traj_rows
lidar=LeiShen CX 32 线, 360° 不裁剪
EOF
    rm -f "$STATE_FILE"
    log "已保存到 $SESSION"
    cat "$SESSION/info.txt"
}

# ---------------------------------------------------------------- status
cmd_status() {
    local status session started now
    status=$(state_get status)
    if [[ -z $status ]]; then
        echo "status=idle"
        return
    fi
    # 进程意外退出（如断电重启）后状态文件残留：以 rosbag 进程是否存活为准
    if [[ $status == recording ]] && ! kill -0 "$(pid_of rosbag)" 2>/dev/null; then
        status=error
    fi
    session=$(state_get session)
    started=$(state_get started)
    now=$(date +%s)
    echo "status=$status"
    echo "session=$session"
    echo "elapsed_s=$((now - started))"
    echo "size=$(du -sh "$session" 2>/dev/null | cut -f1)"
}

case ${1:-} in
    start)  shift; cmd_start "$@" ;;
    stop)   cmd_stop ;;
    status) cmd_status ;;
    *)
        echo "用法: $0 start [--rviz] | stop | status"
        exit 1
        ;;
esac
