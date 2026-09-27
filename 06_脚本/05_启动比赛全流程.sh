#!/usr/bin/env bash
# ============================================================================
#  比赛全流程一键启动（彩排 / 正式比赛通用）
#  顺序: 本地大模型 → Gazebo → Nav2 → 大模型解析 → 调度 → 识别 → 面板 → 障碍物
#  用法: ./05_启动比赛全流程.sh [--dry-run] [--no-sim] [--provider deepseek]
# ============================================================================
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${HERE}/env.sh"

DRY_RUN=false
START_SIM=true
PROVIDER=local
QUESTION=""
for arg in "$@"; do
    case "$arg" in
        --dry-run)  DRY_RUN=true ;;
        --no-sim)   START_SIM=false ;;
        --provider=*) PROVIDER="${arg#*=}" ;;
        --question=*) QUESTION="${arg#*=}" ;;
        -h|--help)
            grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    esac
done

LOGDIR="${LLM_COMP_ROOT}/07_日志"
mkdir -p "${LOGDIR}"
PIDS=()

cleanup() {
    echo
    echo ">>> 停止所有比赛进程..."
    for pid in "${PIDS[@]}"; do kill "${pid}" 2>/dev/null || true; done
    wait 2>/dev/null || true
    echo ">>> 已全部停止"
}
trap cleanup EXIT INT TERM

# --------------------------- 1. 本地大模型 --------------------------------- #
if [ "${PROVIDER}" = "local" ]; then
    if curl -s --max-time 2 "${LLAMA_SERVER_URL}/models" >/dev/null 2>&1; then
        echo "[1/8] llama-server 已在运行，跳过"
    else
        echo "[1/8] 启动本地大模型 llama-server ..."
        "${HERE}/01_启动本地大模型.sh" > "${LOGDIR}/llama-server.log" 2>&1 &
        PIDS+=($!)
        sleep 6
    fi
else
    echo "[1/8] 使用云端大模型: ${PROVIDER}（请确认已设置 API Key）"
fi

# --------------------------- 2. Gazebo ------------------------------------- #
if [ "${START_SIM}" = true ]; then
    echo "[2/8] 启动 Gazebo 仿真 ..."
    ros2 launch llm_comp competition_world.launch.py > "${LOGDIR}/gazebo.log" 2>&1 &
    PIDS+=($!)
    sleep 12
    echo "[3/8] 启动 Nav2 导航 ..."
    ros2 launch bot_navigation nav_bringup_gazebo.launch.py > "${LOGDIR}/nav2.log" 2>&1 &
    PIDS+=($!)
    sleep 8
else
    echo "[2/8] 跳过仿真（--no-sim）"
fi

# --------------------------- 3. 可视化数据源 ------------------------------- #
echo "[4/8] 启动 rosbridge ..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > "${LOGDIR}/rosbridge.log" 2>&1 &
PIDS+=($!)
sleep 3

# --------------------------- 4. 比赛功能节点 ------------------------------- #
echo "[5/8] 启动大模型任务解析 ..."
PARSER_ARGS=(-p provider:="${PROVIDER}" -p use_llm:=true
             -p config_file:="${LLM_COMP_ROOT}/01_dev_ws/dev_ws/src/llm_comp/config/llm_config.yaml")
# 注意: -p question:= 传空串会让 rcl 直接报错，所以只在有题目时才加这个参数
if [ -n "${QUESTION}" ]; then
    PARSER_ARGS+=(-p question:="${QUESTION}")
fi
ros2 run llm_comp llm_task_parser --ros-args "${PARSER_ARGS[@]}" \
    > "${LOGDIR}/parser.log" 2>&1 &
PIDS+=($!)

echo "[6/8] 启动视觉识别 + 状态聚合 ..."
ros2 run llm_comp object_detector --ros-args \
    -p config_file:="${ROS_WS}/src/llm_comp/config/competition.yaml" \
    > "${LOGDIR}/detector.log" 2>&1 &
PIDS+=($!)
ros2 run llm_comp panel_node > "${LOGDIR}/panel.log" 2>&1 &
PIDS+=($!)

echo "[7/8] 启动任务调度 + 动态障碍物 ..."
ros2 run llm_comp task_manager --ros-args -p dry_run:="${DRY_RUN}" \
    -p config_file:="${ROS_WS}/src/llm_comp/config/competition.yaml" \
    > "${LOGDIR}/manager.log" 2>&1 &
PIDS+=($!)
ros2 run llm_comp dynamic_obstacles > "${LOGDIR}/obstacles.log" 2>&1 &
PIDS+=($!)
sleep 2

# --------------------------- 5. 出题 --------------------------------------- #
echo "[8/8] 发布比赛题目 ..."
ros2 run llm_comp question_generator --ros-args -p count:=1 > "${LOGDIR}/generator.log" 2>&1 &
PIDS+=($!)

cat <<EOF

============================================================
  比赛环境已启动 (dry_run=${DRY_RUN}, provider=${PROVIDER})
------------------------------------------------------------
  可视化面板: ws://127.0.0.1:9090  (Foxglove / coStudio)
  关键话题:
     /competition/panel          面板总数据(JSON)
     /competition/panel_text     面板文本
     /competition/task_plan      结构化指令 [red,4,A;blue,1,C]
     /competition/task_status    当前状态
     /competition/nav_path       规划路径
     /competition/robot_path     实际路径
  日志目录: ${LOGDIR}
  按 Ctrl+C 停止全部进程
============================================================
EOF

wait
