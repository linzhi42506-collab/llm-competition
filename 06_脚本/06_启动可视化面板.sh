#!/usr/bin/env bash
# 启动 Foxglove Studio（免安装版：deb 已解包到本目录，不需要 sudo）
# 连接方式: Open connection → Rosbridge → ws://127.0.0.1:9090
set -e
ROOT="/home/robo/llm_comp"
APP="${ROOT}/08_可视化/foxglove/extracted/opt/Foxglove/foxglove-studio"

if [ ! -x "${APP}" ]; then
    echo "未找到 Foxglove，请先执行:"
    echo "  cd ${ROOT}/08_可视化/foxglove && dpkg -x foxglove-studio.deb extracted/"
    exit 1
fi

echo ">>> 启动 Foxglove Studio 3.2.1"
echo "    连接: Open connection → Rosbridge → ws://127.0.0.1:9090"
echo "    布局: 导入 ${ROOT}/08_可视化/foxglove/competition_layout.json"
exec "${APP}" "$@"
