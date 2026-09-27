#!/usr/bin/env bash
# ============================================================================
#  比赛环境变量总入口 —— 所有其它脚本都会 source 这个文件
#  用法:  source /home/robo/llm_comp/06_脚本/env.sh
#  说明:  只影响当前终端，不修改 ~/.bashrc 等任何现有配置。
#
#  注意:  本机 ~/.bashrc 里已经 source 了系统 ROS2(/opt/ros/humble, Python3.10)。
#         本项目使用独立 conda 环境(ROS2 Humble, Python3.12)，两者混用会出问题，
#         所以下面会先把系统 ROS 的环境变量清干净，再激活独立环境。
#         想用系统 ROS2 时: export USE_SYSTEM_ROS=1 后再 source 本文件。
# ============================================================================

export LLM_COMP_ROOT="/home/robo/llm_comp"
export ROS_WS="${LLM_COMP_ROOT}/01_dev_ws/dev_ws"
export LLAMA_DIR="${LLM_COMP_ROOT}/02_llama.cpp/llama.cpp"
export LLAMA_MODEL="${LLAMA_DIR}/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf"
export CONDA_ROS_ENV="${LLM_COMP_ROOT}/09_ros2_env/ros2humble"

# --------------------------- 0. 清理外部 ROS 环境 --------------------------- #
_strip_ros_paths() {
    local var_name="$1" out="" part
    local value="${!var_name}"
    [ -z "${value}" ] && return
    local IFS=':'
    for part in ${value}; do
        case "${part}" in
            /opt/ros/*|/opt/ros) ;;                       # 丢掉系统 ROS 路径
            *) out="${out:+${out}:}${part}" ;;
        esac
    done
    export "${var_name}=${out}"
}
if [ "${USE_SYSTEM_ROS:-0}" != "1" ]; then
    unset ROS_VERSION ROS_PYTHON_VERSION ROS_DISTRO AMENT_PREFIX_PATH \
          COLCON_PREFIX_PATH ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE 2>/dev/null
    _strip_ros_paths LD_LIBRARY_PATH
    _strip_ros_paths PYTHONPATH
    _strip_ros_paths CMAKE_PREFIX_PATH
fi
unset -f _strip_ros_paths 2>/dev/null

# --------------------------- 1. ROS2 环境 ---------------------------------- #
if [ "${USE_SYSTEM_ROS:-0}" = "1" ]; then
    # shellcheck disable=SC1091
    source "/opt/ros/humble/setup.bash"
    echo "[env] 使用系统 ROS2: /opt/ros/humble (Python3.10)"
elif [ -f "${CONDA_ROS_ENV}/setup.bash" ]; then
    # 独立 conda 环境（无需 sudo，装在本目录内，方便整包删除）
    if [ -f "/home/robo/miniconda3/etc/profile.d/conda.sh" ]; then
        # shellcheck disable=SC1091
        source "/home/robo/miniconda3/etc/profile.d/conda.sh"
        conda activate "${CONDA_ROS_ENV}" >/dev/null 2>&1
    fi
    export ROS_DISTRO="${ROS_DISTRO:-humble}"
    echo "[env] 独立 conda ROS2 环境: ${CONDA_ROS_ENV} (ROS_DISTRO=${ROS_DISTRO}, $(python3 -V 2>&1))"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "/opt/ros/humble/setup.bash"
    echo "[env] 回退到系统 ROS2: /opt/ros/humble"
else
    echo "[env] 警告: 未找到任何 ROS2 环境！"
fi

# --------------------------- 2. 工作空间 ----------------------------------- #
if [ -f "${ROS_WS}/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${ROS_WS}/install/setup.bash" >/dev/null 2>&1
    echo "[env] 已加载工作空间: ${ROS_WS}/install"
else
    echo "[env] 提示: 工作空间尚未编译，请先运行 06_脚本/00_编译工作空间.sh"
fi

# --------------------------- 2.5 隔离 ROS 域 --------------------------------- #
# 本机同时还装着系统 ROS2(Python3.10)，两边共用 default 域会互相干扰：
#   - ros2cli 的 daemon 会串台，报 xmlrpc Fault: !rclpy.ok()
#   - 你和别人的 ros2 节点会互相看到对方的话题
# 给比赛环境单独一个域号即可彻底隔离。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY=0

# --------------------------- 3. Gazebo 资源路径 ---------------------------- #
# 关键: 关掉 Gazebo 在线模型库查询。不关的话 gzserver 会反复尝试连接
# models.gazebosim.org（连不上），导致场景加载要等好几分钟。
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH="${ROS_WS}/src/yzbot/mybot_description/worlds:${HOME}/.gazebo/models:${GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="${ROS_WS}/src/yzbot/mybot_description:${GAZEBO_RESOURCE_PATH}"
export SVGA_VGPU10=0

# --------------------------- 4. 本地大模型 --------------------------------- #
export LLAMA_SERVER_URL="http://127.0.0.1:8081/v1"
# 需要云端大模型时，把 Key 写进下面这个文件（该文件不会被提交）
if [ -f "${LLM_COMP_ROOT}/06_脚本/api_keys.sh" ]; then
    # shellcheck disable=SC1091
    source "${LLM_COMP_ROOT}/06_脚本/api_keys.sh"
fi

# --------------------------- 5. 日志格式 ----------------------------------- #
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=1
