#!/usr/bin/env bash
# 编译比赛工作空间（在独立 conda ROS2 环境里，无需 sudo）
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${HERE}/env.sh"

cd "${ROS_WS}"
echo ">>> 开始编译: $(pwd)"
colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --packages-skip-build-finished 2>&1 | tee "${LLM_COMP_ROOT}/07_日志/colcon_build.log"
echo
echo ">>> 编译结束，重新加载环境:"
echo "    source ${HERE}/env.sh"
