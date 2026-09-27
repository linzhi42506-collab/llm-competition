#!/usr/bin/env bash
# 启动可视化面板数据源：rosbridge_websocket (9090)
# 对应评分项 2「指令解析」、6「状态反馈」的显示通道
# 之后用 Foxglove Studio 或 coStudio 连接 ws://<本机IP>:9090
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${HERE}/env.sh"

echo ">>> 启动 rosbridge（WebSocket 端口 9090）"
echo "    Foxglove:  Open connection → Rosbridge → ws://127.0.0.1:9090"
echo "    coStudio:  设备 IP 填本机 IP，端口 9090"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
