#!/usr/bin/env bash
# ============================================================================
#  把本仓库推送到 GitHub（一次性完成：建仓库 + 推送）
#
#  用法:
#      # 方式一：直接传 Token（推荐，Token 不会被写进任何文件）
#      ./11_推送到GitHub.sh <你的PersonalAccessToken> [仓库名]
#
#      # 方式二：用环境变量
#      export GITHUB_TOKEN=ghp_xxxx
#      ./11_推送到GitHub.sh
#
#  怎么拿 Token（1 分钟）:
#      1. 浏览器打开 https://github.com/settings/tokens
#      2. 右上角 Generate new token → Generate new token (classic)
#      3. Note 随便填（如 robot-competition），Expiration 选 90 天
#      4. 勾选权限：☑ repo   （要建仓库必须勾这个）
#      5. 拉到底点 Generate token，复制那串 ghp_xxxx
#
#  ⚠ 安全提醒：
#      - 密码不能用于 git 推送，GitHub 2021 年就停用了密码认证
#      - Token 只在本次命令里用一次，不会存进仓库或任何文件
#      - 推送完成后可以到上面那个页面把 Token 撤销（Revoke）
# ============================================================================
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "${HERE}")"
cd "${ROOT}"

TOKEN="${1:-${GITHUB_TOKEN:-}}"
REPO_NAME="${2:-llm-competition}"

if [ -z "${TOKEN}" ]; then
    echo "错误：没有提供 GitHub Token"
    echo
    grep '^#' "$0" | sed -n '3,25p' | sed 's/^# \{0,1\}//'
    exit 1
fi

echo ">>> 1/4 校验 Token ..."
USER_JSON=$(curl -s -H "Authorization: token ${TOKEN}" https://api.github.com/user)
LOGIN=$(echo "${USER_JSON}" | python3 -c "import sys,json; print(json.load(sys.stdin).get('login',''))" 2>/dev/null || echo "")
if [ -z "${LOGIN}" ]; then
    echo "✗ Token 无效或权限不足。返回："
    echo "${USER_JSON}" | head -5
    exit 1
fi
echo "    已登录为：${LOGIN}"

echo ">>> 2/4 创建仓库 ${REPO_NAME}（private，避免其他参赛队看到方案）..."
CREATE=$(curl -s -X POST \
    -H "Authorization: token ${TOKEN}" \
    -H "Accept: application/vnd.github+json" \
    https://api.github.com/user/repos \
    -d "{\"name\":\"${REPO_NAME}\",\"private\":true,\"description\":\"大模型技术创新赛参赛作品 ROS2+Gazebo+Nav2+MoveIt2+LLM\"}")
if echo "${CREATE}" | grep -q '"full_name"'; then
    echo "    ✓ 仓库已创建"
elif echo "${CREATE}" | grep -q "already exists"; then
    echo "    ✓ 仓库已存在，直接推送"
else
    echo "    ! 创建返回：$(echo "${CREATE}" | head -3)"
fi

echo ">>> 3/4 配置 remote 并推送 ..."
git remote remove origin 2>/dev/null || true
git remote add origin "https://${TOKEN}@github.com/${LOGIN}/${REPO_NAME}.git"
git branch -M main
git push -u origin main 2>&1 | tail -6

echo ">>> 4/4 清理（把带 Token 的 remote 换成不含 Token 的地址）..."
git remote set-url origin "https://github.com/${LOGIN}/${REPO_NAME}.git"

cat <<EOF

============================================================
 推送完成 🎉
   仓库地址: https://github.com/${LOGIN}/${REPO_NAME}
   （已设为 private，如需公开请到仓库 Settings → Danger Zone 修改）

 建议现在去 https://github.com/settings/tokens 把这个 Token 撤销
============================================================
EOF
