#!/usr/bin/env bash
# 启动 Gazebo 仿真：底盘 + 6 自由度机械臂 + 传感器 + 场景
# 对应评分项 1「系统启动」10 分
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${HERE}/env.sh"

echo ">>> 启动 Gazebo 比赛场景（competition.world）"
echo "    场景文件由 06_脚本/生成比赛场景.py 生成；想用官方原始场景加 world:=room.world"
ros2 launch llm_comp competition_world.launch.py
