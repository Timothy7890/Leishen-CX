#!/usr/bin/env bash
# 编译 catkin_ws。FAST-LIO 未声明消息依赖，并行编译首次可能报找不到 *.h，自动重试一次即可
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/../catkin_ws" && pwd)
PATH=$(echo "$PATH" | tr ':' '\n' | grep -viE 'conda' | paste -sd: -)
source /opt/ros/noetic/setup.bash
cd "$WS" || exit 1
catkin_make -DCMAKE_BUILD_TYPE=Release -j4 || catkin_make -DCMAKE_BUILD_TYPE=Release -j4
