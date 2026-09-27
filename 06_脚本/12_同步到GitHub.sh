#!/usr/bin/env bash
# ============================================================================
#  日常同步到 GitHub —— 改完代码后一条命令推上去
#
#  用法:
#      ./12_同步到GitHub.sh <你的Token> ["本次改了什么"]
#
#      例: ./12_同步到GitHub.sh ghp_xxxx "修好了导航参数"
#
#  如果嫌每次输 Token 麻烦，可以在当前终端先 export：
#      export GITHUB_TOKEN=ghp_xxxx
#      ./12_同步到GitHub.sh "" "改了什么"
#
#  ⚠ Token 只是本次使用，不会写入仓库或任何文件。
#    比赛结束后建议到 https://github.com/settings/tokens 撤销它。
# ============================================================================
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "${HERE}")"
cd "${ROOT}"

USER_NAME="linzhi42506-collab"
REPO_NAME="llm-competition"
TOKEN="${1:-${GITHUB_TOKEN:-}}"
MESSAGE="${2:-update: $(date '+%Y-%m-%d %H:%M')}"

if [ ! -d .git ]; then
    echo "错误：当前目录不是 git 仓库"
    exit 1
fi

echo ">>> 1/4 检查改动 ..."
if [ -z "$(git status --porcelain)" ] && git diff --quiet HEAD 2>/dev/null; then
    echo "    没有新改动，检查是否本地领先远程 ..."
else
    echo ">>> 2/4 提交改动 ..."
    git add -A
    git commit -q -m "${MESSAGE}" && echo "    已提交: ${MESSAGE}"
fi

if [ -z "${TOKEN}" ]; then
    echo
    echo ">>> 没有提供 Token，只做了本地提交。"
    echo "    要推送请执行:"
    echo "      ./12_同步到GitHub.sh ghp_你的token \"说明\""
    exit 0
fi

echo ">>> 3/4 推送到 GitHub ..."
git push "https://${TOKEN}@github.com/${USER_NAME}/${REPO_NAME}.git" main 2>&1 | tail -4

echo ">>> 4/4 确认同步 ..."
git fetch "https://${TOKEN}@github.com/${USER_NAME}/${REPO_NAME}.git" main 2>/dev/null
LOCAL=$(git rev-parse HEAD)
REMOTE=$(git rev-parse FETCH_HEAD)
if [ "${LOCAL}" = "${REMOTE}" ]; then
    echo "    ✓ 本地与 GitHub 已完全同步"
    echo "    仓库地址: https://github.com/${USER_NAME}/${REPO_NAME}"
else
    echo "    ✗ 仍未同步，请检查网络或 Token"
    exit 1
fi
