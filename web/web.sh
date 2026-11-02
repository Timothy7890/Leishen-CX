#!/usr/bin/env bash
# 7010 录制控制网页服务
# 用法: web/web.sh start | stop | restart | status | autostart
WEB_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
RUN_DIR=$WEB_DIR/../run
PID_FILE=$RUN_DIR/web.pid
LOG_FILE=$RUN_DIR/web.log
PORT=${RECORD_WEB_PORT:-7010}
mkdir -p "$RUN_DIR"

running() { [[ -f $PID_FILE ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; }

case ${1:-} in
    start)
        if running; then echo "已在运行 (pid $(cat "$PID_FILE"))，端口 $PORT"; exit 0; fi
        setsid /usr/bin/python3 "$WEB_DIR/server.py" >> "$LOG_FILE" 2>&1 < /dev/null &
        echo $! > "$PID_FILE"
        sleep 1
        if running; then echo "已启动，端口 $PORT"; else echo "启动失败，见 $LOG_FILE"; exit 1; fi
        ;;
    stop)
        running && kill "$(cat "$PID_FILE")" && echo "已停止"
        rm -f "$PID_FILE"
        ;;
    restart)
        "$0" stop; "$0" start
        ;;
    status)
        if running; then echo "运行中 (pid $(cat "$PID_FILE"))，端口 $PORT"; else echo "未运行"; fi
        ;;
    autostart)
        # 写入 crontab @reboot（不需要 sudo），重复执行不会重复添加
        line="@reboot sleep 20 && $WEB_DIR/web.sh start"
        (crontab -l 2>/dev/null | grep -vF "$WEB_DIR/web.sh"; echo "$line") | crontab -
        echo "已设置开机自启: $line"
        ;;
    *)
        echo "用法: $0 start | stop | restart | status | autostart"
        exit 1
        ;;
esac
