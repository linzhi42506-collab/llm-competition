#!/usr/bin/env bash
# 启动 Nav2 导航 + rviz（必须在 Gazebo 启动之后再运行）
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${HERE}/env.sh"

# 让机器人从场景原点开始；如需指定初始位姿，可在 rviz 里用 "2D Pose Estimate"
echo ">>> 启动 Nav2（bot_navigation/nav_bringup_gazebo.launch.py）"
echo "    提示: 启动后如提示缺少 map->odom，请在 rviz 用 2D Pose Estimate 指定初始位姿"
ros2 launch bot_navigation nav_bringup_gazebo.launch.py
