#!/usr/bin/env bash
# 编译 Livox-SDK2（首次）和 catkin_ws。FAST-LIO 未声明消息依赖，并行编译首次可能报找不到 *.h，自动重试一次即可
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
WS=$ROOT/catkin_ws
PATH=$(echo "$PATH" | tr ':' '\n' | grep -viE 'conda' | paste -sd: -)

SDK=$ROOT/thirdparty/Livox-SDK2
if [ ! -f "$ROOT/thirdparty/livox_sdk2/lib/liblivox_lidar_sdk_static.a" ]; then
  cmake -S "$SDK" -B "$SDK/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$ROOT/thirdparty/livox_sdk2" &&
    cmake --build "$SDK/build" -j4 && cmake --install "$SDK/build" || exit 1
fi

source /opt/ros/noetic/setup.bash
cd "$WS" || exit 1
catkin_make -DCMAKE_BUILD_TYPE=Release -j4 || catkin_make -DCMAKE_BUILD_TYPE=Release -j4
