#!/usr/bin/env bash
# ============================================================================
#  在成熟地图上导航
#
#  用法:
#    ./nav.sh start <录制目录> [--dry-run]   启动定位与导航（--dry-run 不向小车发指令）
#    ./nav.sh stop
#    ./nav.sh status
#
#  启动后由 7011 网页（web/nav_server.py -> scripts/nav_bridge.py）重定位、下发目标、急停。
#  与 record.sh 共用雷达，同一时间只能运行一个。
# ============================================================================

PROJECT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
RUN_DIR=$PROJECT_DIR/run
STATE_FILE=$RUN_DIR/nav_state
PID_FILE=$RUN_DIR/nav_pids
LOCK_DIR=$RUN_DIR/nav_lock
LOG_DIR=$RUN_DIR/nav_logs
LIDAR_IP=192.168.1.200

mkdir -p "$RUN_DIR"

PATH=$(echo "$PATH" | tr ':' '\n' | grep -viE 'conda' | paste -sd: -)
unset PYTHONPATH CONDA_PREFIX
source /opt/ros/noetic/setup.bash
source "$PROJECT_DIR/catkin_ws/devel/setup.bash"
export ROS_MASTER_URI=http://localhost:11311
export PYTHONUNBUFFERED=1

log() { echo "[$(date +%H:%M:%S)] $*"; }

state_get() { [[ -f $STATE_FILE ]] && grep -m1 "^$1=" "$STATE_FILE" | cut -d= -f2-; }
state_set() {
    touch "$STATE_FILE"
    sed -i "/^$1=/d" "$STATE_FILE"
    echo "$1=$2" >> "$STATE_FILE"
}

lock() {
    if ! mkdir "$LOCK_DIR" 2> /dev/null; then
        echo "另一个 start/stop 操作正在进行，请稍后再试" >&2
        exit 2
    fi
    trap 'rmdir "$LOCK_DIR" 2>/dev/null' EXIT
}

launch() {
    local name=$1; shift
    setsid "$@" > "$LOG_DIR/$name.log" 2>&1 < /dev/null &
    echo "$name $!" >> "$PID_FILE"
}

pid_of() { [[ -f $PID_FILE ]] && awk -v n="$1" '$1 == n {print $2}' "$PID_FILE"; }

stop_proc() {
    local name=$1 timeout=${2:-10} pid
    pid=$(pid_of "$name")
    [[ -z $pid ]] && return 0
    kill -0 "$pid" 2> /dev/null || return 0
    kill -INT -- "-$pid" 2> /dev/null
    for ((i = 0; i < timeout * 2; i++)); do
        kill -0 "$pid" 2> /dev/null || return 0
        sleep 0.5
    done
    kill -KILL -- "-$pid" 2> /dev/null
}

wait_topic() { timeout "$2" rostopic echo -n 1 "$1/header/stamp" > /dev/null 2>&1; }

shutdown_all() {
    # 先断开底盘（发零速后退出），再停其它
    stop_proc car_serial 5
    stop_proc safety_gate 5
    for p in bridge move_base map_server base_tf localizer fastlio imu lidar; do stop_proc "$p" 10; done
    stop_proc roscore 10
    rm -f "$PID_FILE"
}

cmd_start() {
    local session=$1 dry=false
    [[ ${2:-} == --dry-run ]] && dry=true
    lock
    [[ -n $session && -f $session/nav/meta.json ]] || { echo "不是成熟地图: $session" >&2; exit 1; }
    session=$(cd "$session" && pwd)
    if [[ -n $(state_get status) ]]; then
        echo "导航已在运行: $(state_get session)" >&2; exit 1
    fi
    if rostopic list > /dev/null 2>&1; then
        echo "检测到已有 ROS 在运行（可能正在录制），请先停止" >&2; exit 1
    fi
    ping -c 1 -W 1 "$LIDAR_IP" > /dev/null 2>&1 || { echo "雷达 $LIDAR_IP 无法连通" >&2; exit 1; }
    [[ -e /dev/imu ]] || { echo "未找到 IMU 设备 /dev/imu" >&2; exit 1; }

    rm -rf "$LOG_DIR"; mkdir -p "$LOG_DIR"
    export ROS_LOG_DIR=$LOG_DIR/ros
    rm -f "$PID_FILE" "$STATE_FILE"
    state_set status starting
    state_set session "$session"
    state_set dry_run "$dry"
    state_set started "$(date +%s)"

    fail() {
        log "启动失败: $1（日志见 $LOG_DIR）"
        shutdown_all
        rm -f "$STATE_FILE"
        exit 1
    }

    log "启动 roscore"
    launch roscore roscore
    for ((i = 0; i < 30; i++)); do rostopic list > /dev/null 2>&1 && break; sleep 0.5; done
    rostopic list > /dev/null 2>&1 || fail "roscore 未启动"

    log "启动雷达与 IMU"
    launch lidar roslaunch "$PROJECT_DIR/config/lidar_360.launch"
    wait_topic /velodyne_points 20 || fail "未收到雷达点云"
    launch imu roslaunch "$PROJECT_DIR/config/imu.launch"
    wait_topic /imu 15 || fail "未收到 IMU 数据"

    log "启动 FAST-LIO 里程计"
    rosparam load "$PROJECT_DIR/config/fastlio_360.yaml"
    rosparam set /pcd_save/pcd_save_en false
    launch fastlio rosrun fast_lio fastlio_mapping
    wait_topic /Odometry 20 || fail "FAST-LIO 未输出里程计"

    log "启动地图定位（自动重定位）"
    launch localizer python3 "$PROJECT_DIR/scripts/localizer.py" "$session/nav"
    launch base_tf rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 body base_link
    launch map_server rosrun map_server map_server "$session/nav/grid.yaml" _frame_id:=map

    log "启动路径规划"
    rosparam load "$PROJECT_DIR/config/nav/costmap_common.yaml" /move_base/global_costmap
    rosparam load "$PROJECT_DIR/config/nav/costmap_common.yaml" /move_base/local_costmap
    rosparam load "$PROJECT_DIR/config/nav/move_base.yaml" /move_base
    # 可执行文件默认节点名是 move_base_node，必须改名才能读到 /move_base 下的参数
    launch move_base rosrun move_base move_base __name:=move_base cmd_vel:=/cmd_vel_nav

    log "启动安全门与底盘串口$($dry && echo '（空跑：不发送）')"
    launch safety_gate python3 "$PROJECT_DIR/scripts/safety_gate.py"
    if $dry; then
        launch car_serial python3 "$PROJECT_DIR/scripts/car_serial.py" _dry_run:=true
    else
        launch car_serial python3 "$PROJECT_DIR/scripts/car_serial.py"
    fi
    launch bridge python3 "$PROJECT_DIR/scripts/nav_bridge.py" "$session/nav"

    state_set status running
    log "导航已启动。打开 7011 网页查看定位并下发目标"
}

cmd_stop() {
    lock
    [[ -n $(state_get status) ]] || { echo "导航未运行" >&2; exit 1; }
    log "停止导航"
    shutdown_all
    rm -f "$STATE_FILE"
    log "已停止"
}

cmd_status() {
    local status
    status=$(state_get status)
    if [[ -z $status ]]; then
        echo "status=idle"
        return
    fi
    if [[ $status == running ]] && ! kill -0 "$(pid_of car_serial)" 2> /dev/null; then
        status=error
    fi
    echo "status=$status"
    echo "session=$(state_get session)"
    echo "dry_run=$(state_get dry_run)"
    echo "elapsed_s=$(($(date +%s) - $(state_get started)))"
}

case ${1:-} in
    start)  shift; cmd_start "$@" ;;
    stop)   cmd_stop ;;
    status) cmd_status ;;
    *)
        echo "用法: $0 start <录制目录> [--dry-run] | stop | status"
        exit 1
        ;;
esac
