#!/usr/bin/env bash
# ============================================================================
#  重新建图：场景改造后官方地图失效，用本脚本自动跑一圈生成新地图
#  前置：Gazebo 仿真已经在运行（02_启动仿真.sh 或 competition_world.launch.py）
#  用法: ./07_自动建图.sh [巡游秒数，默认 240]
# ============================================================================
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${HERE}/env.sh"

DURATION="${1:-240}"
MAPDIR="${ROS_WS}/src/yzbot/bot_navigation/maps"
MAPNAME="competition_map"
LOGDIR="${LLM_COMP_ROOT}/07_日志"

mkdir -p "${MAPDIR}"

echo ">>> [1/3] 启动 SLAM (slam_toolbox online_async) ..."
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
    > "${LOGDIR}/slam.log" 2>&1 &
SLAM_PID=$!
sleep 8

cleanup() {
    echo
    echo ">>> 停止 SLAM 与巡游 ..."
    kill "${MAPPER_PID:-0}" "${SLAM_PID:-0}" 2>/dev/null || true
    # 停车
    timeout 5 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo ">>> [2/3] 自动巡游 ${DURATION} 秒（机器人自己跑一圈建图）..."
ros2 run llm_comp auto_mapper --ros-args \
    -p duration:="${DURATION}.0" -p forward_speed:=0.22 -p safety_distance:=0.75 \
    > "${LOGDIR}/auto_mapper.log" 2>&1 &
MAPPER_PID=$!
wait "${MAPPER_PID}" || true

echo ">>> [3/3] 保存地图到 ${MAPDIR}/${MAPNAME}.pgm/.yaml ..."
ros2 run nav2_map_server map_saver_cli -f "${MAPDIR}/${MAPNAME}" --ros-args -p use_sim_time:=true
sleep 2

cat <<MSG

============================================================
 新地图已保存:
   ${MAPDIR}/${MAPNAME}.pgm
   ${MAPDIR}/${MAPNAME}.yaml

 接下来把 Nav2 指向新地图（二选一）:

  A) 临时验证（不动任何文件）:
     ros2 launch bot_navigation nav_bringup_gazebo.launch.py \\
        map:="${MAPDIR}/${MAPNAME}.yaml"

  B) 固化下来: 编辑
     01_dev_ws/dev_ws/src/llm_comp/config/competition.yaml
     把 nav.map 改成 ${MAPNAME}.yaml，
     并在 bot_navigation 的 launch 里把 map_yaml_file 改成同名文件。

 之后启动 Nav2，在 rviz 里用 "2D Pose Estimate" 给机器人一个初始位姿，
 map->odom 就会建立，rviz 里就能看到地图、激光点和机器人了。
============================================================
MSG
