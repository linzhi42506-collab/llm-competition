#!/usr/bin/env bash
# 启动本地大模型服务（llama.cpp llama-server），供 llm_task_parser 调用
# 监听: http://127.0.0.1:8081  （OpenAI 兼容 /v1/chat/completions）
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "${HERE}")"
LLAMA_DIR="${ROOT}/02_llama.cpp/llama.cpp"
MODEL="${LLAMA_DIR}/qwen2.5-coder-0.5b-instruct-q4_k_m.gguf"
PORT="${1:-8081}"

if [ ! -x "${LLAMA_DIR}/build/bin/llama-server" ]; then
    echo "错误: 未找到 ${LLAMA_DIR}/build/bin/llama-server"
    exit 1
fi
if [ ! -f "${MODEL}" ]; then
    echo "错误: 未找到模型 ${MODEL}"
    exit 1
fi

echo ">>> 启动 llama-server: 端口 ${PORT}, 模型 $(basename "${MODEL}")"
exec env LD_LIBRARY_PATH="${LLAMA_DIR}/build/bin:${LD_LIBRARY_PATH}" \
    "${LLAMA_DIR}/build/bin/llama-server" \
    -m "${MODEL}" \
    -c 4096 \
    -t "$(nproc)" \
    --host 127.0.0.1 --port "${PORT}"
