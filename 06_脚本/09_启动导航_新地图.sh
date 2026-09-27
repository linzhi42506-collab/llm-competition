#!/usr/bin/env bash
# 启动导航（含三个必要修复）：
#   1. 地图换成重新生成的 competition_map.yaml
#   2. 参数用修好的 nav2_competition.yaml
#      （补了 use_sim_time、关掉 consider_footprint）
#   3. 启动 odom_tf_broadcaster 补上 odom→base_footprint
#      （平台自带插件只在运动时偶发发布，导致代价地图卡在 activating）
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${HERE}/env.sh"
MAP="${ROS_WS}/src/yzbot/bot_navigation/maps/competition_map.yaml"
PARAMS="${ROS_WS}/src/llm_comp/config/nav2_competition.yaml"

echo ">>> 启动里程计 TF 补丁 ..."
ros2 run llm_comp odom_tf_broadcaster > "${LLM_COMP_ROOT}/07_日志/odom_tf.log" 2>&1 &
sleep 2

echo ">>> 启动 Nav2"
echo "    地图: ${MAP}"
echo "    参数: ${PARAMS}"
exec ros2 launch bot_navigation nav_bringup_gazebo.launch.py \
    map:="${MAP}" params_file:="${PARAMS}"
