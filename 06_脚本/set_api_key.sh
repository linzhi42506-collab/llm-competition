#!/usr/bin/env bash
# 配置云端大模型 API Key（写入本目录的 api_keys.sh，该文件不会被提交）
# 用法:  ./set_api_key.sh deepseek sk-xxxxxxxx
#        ./set_api_key.sh zhipu  xxxxxxxx
#        ./set_api_key.sh volcengine xxxxxxxx
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FILE="${HERE}/api_keys.sh"

PROVIDER="${1:-}"
KEY="${2:-}"

declare -A ENVNAME=(
    [deepseek]=DEEPSEEK_API_KEY
    [zhipu]=ZHIPUAI_API_KEY
    [volcengine]=ARK_API_KEY
    [dashscope]=DASHSCOPE_API_KEY
    [siliconflow]=SILICONFLOW_API_KEY
    [openai]=OPENAI_API_KEY
)

if [ -z "${PROVIDER}" ] || [ -z "${KEY}" ]; then
    echo "用法: $0 <provider> <api_key>"
    echo "支持的 provider: ${!ENVNAME[@]}"
    echo
    if [ -f "${FILE}" ]; then
        echo "当前已配置的环境变量名:"; grep -o '^export [A-Z_]*' "${FILE}" || true
    fi
    exit 1
fi

VAR="${ENVNAME[${PROVIDER}]}"
if [ -z "${VAR}" ]; then
    echo "未知 provider: ${PROVIDER}；支持: ${!ENVNAME[@]}"; exit 1
fi

touch "${FILE}"
chmod 600 "${FILE}"
# 覆盖同名变量
grep -v "^export ${VAR}=" "${FILE}" > "${FILE}.tmp" 2>/dev/null || true
mv "${FILE}.tmp" "${FILE}"
echo "export ${VAR}=\"${KEY}\"" >> "${FILE}"
echo "已写入 ${VAR} → ${FILE} (权限 600)"
echo "重新 source 环境即可生效: source ${HERE}/env.sh"
echo
echo "别忘了把 llm_config.yaml 里的 provider 改成 ${PROVIDER}"
