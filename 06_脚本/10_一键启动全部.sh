#!/usr/bin/env bash
# ============================================================================
#  一键启动全部（顺序已按依赖排好，含所有已修复的坑）
#
#  用法:
#     ./10_一键启动全部.sh              # 正常启动
#     ./10_一键启动全部.sh --no-sim     # Gazebo 已经在跑，只起其余部分
#     ./10_一键启动全部.sh --dry-run    # 不真动机器人，只验证面板
#
#  启动顺序（很重要）:
#     1. Gazebo 场景      → 约 90 秒加载（PAL_office 模型较大）
#     2. odom TF 补丁     → 平台缺 odom→base_footprint，Nav2 起不来
#     3. Nav2 + rviz      → 用新地图 + 修好的参数
#     4. 自动初始位姿     → 免去手点 2D Pose Estimate
#     5. rosbridge        → 可视化面板数据源
#     6. 比赛功能节点     → 解析/调度/识别/面板/障碍物
#     7. 发题             → 触发整条链路
# ============================================================================
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${HERE}/env.sh"

LOGDIR="${LLM_COMP_ROOT}/07_日志"
mkdir -p "${LOGDIR}"

START_SIM=true
DRY_RUN=false
PROVIDER=local
for arg in "$@"; do
    case "$arg" in
        --no-sim)   START_SIM=false ;;
        --dry-run)  DRY_RUN=true ;;
        --provider=*) PROVIDER="${arg#*=}" ;;
    esac
done

PIDS=()
cleanup() {
    echo; echo ">>> 停止全部进程 ..."
    for pid in "${PIDS[@]}"; do kill "${pid}" 2>/dev/null || true; done
    wait 2>/dev/null || true
    echo ">>> 已停止"
}
trap cleanup EXIT INT TERM

# ---------------------------- 1. Gazebo ------------------------------------ #
if [ "${START_SIM}" = true ]; then
    echo "[1/7] 启动 Gazebo 比赛场景 ..."
    ros2 launch llm_comp competition_world.launch.py > "${LOGDIR}/gazebo.log" 2>&1 &
    PIDS+=($!)
    echo "      等待场景加载（90 秒）..."
    sleep 90
else
    echo "[1/7] 跳过仿真"
fi

# ---------------------------- 2. odom TF 补丁 ------------------------------ #
echo "[2/7] 启动里程计 TF 补丁（修复平台缺失的 odom→base_footprint）..."
ros2 run llm_comp odom_tf_broadcaster > "${LOGDIR}/odom_tf.log" 2>&1 &
PIDS+=($!)
sleep 3

# ---------------------------- 3. Nav2 -------------------------------------- #
echo "[3/7] 启动 Nav2（新地图 + 修好的参数 + RPP 控制器）..."
"${HERE}/09_启动导航_新地图.sh" > "${LOGDIR}/nav2.log" 2>&1 &
PIDS+=($!)
sleep 20

# ---------------------------- 4. 自动初始位姿 ------------------------------ #
echo "[4/7] 启动自动初始位姿（等价于手点 2D Pose Estimate）..."
ros2 run llm_comp auto_initial_pose --ros-args -p duration:=60.0 \
    > "${LOGDIR}/auto_init.log" 2>&1 &
PIDS+=($!)
sleep 25

# ---------------------------- 5. rosbridge --------------------------------- #
echo "[5/7] 启动 rosbridge（面板数据源，端口 9090）..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    > "${LOGDIR}/rosbridge.log" 2>&1 &
PIDS+=($!)
sleep 4

# ---------------------------- 6. 比赛功能节点 ------------------------------ #
echo "[6/7] 启动比赛功能节点 ..."
ros2 run llm_comp llm_task_parser --ros-args -p provider:="${PROVIDER}" \
    > "${LOGDIR}/parser.log" 2>&1 &
PIDS+=($!)
ros2 run llm_comp object_detector > "${LOGDIR}/detector.log" 2>&1 &
PIDS+=($!)
ros2 run llm_comp panel_node > "${LOGDIR}/panel.log" 2>&1 &
PIDS+=($!)
ros2 run llm_comp task_manager --ros-args -p dry_run:="${DRY_RUN}" \
    > "${LOGDIR}/manager.log" 2>&1 &
PIDS+=($!)
ros2 run llm_comp dynamic_obstacles > "${LOGDIR}/obstacles.log" 2>&1 &
PIDS+=($!)
sleep 3

# ---------------------------- 7. 出题 -------------------------------------- #
echo "[7/7] 发布比赛题目 ..."
ros2 run llm_comp question_generator --ros-args -p count:=1 > "${LOGDIR}/generator.log" 2>&1 &
PIDS+=($!)

cat <<EOF

============================================================
  全部启动完成  (dry_run=${DRY_RUN}, provider=${PROVIDER})
------------------------------------------------------------
  可视化面板 : ws://127.0.0.1:9090  → Foxglove / coStudio
  面板文本   : /competition/panel_text
  结构化指令 : /competition/task_plan

  自检命令（三个都应该是 active）:
    ros2 lifecycle get /amcl
    ros2 lifecycle get /controller_server
    ros2 lifecycle get /bt_navigator

  日志目录   : ${LOGDIR}
  按 Ctrl+C 停止全部
============================================================
EOF
wait
