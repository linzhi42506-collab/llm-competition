#!/usr/bin/env bash
# ============================================================================
#  添加队友为协作者（加完他们就能直接 git push 到这个仓库）
#
#  用法:
#      ./13_添加队友到仓库.sh <你的Token> <队友GitHub用户名> [队友2] [队友3] ...
#
#  例:
#      ./13_添加队友到仓库.sh ghp_xxxx zhangsan lisi wangwu
#
#  说明:
#      - 加完后队友会收到 GitHub 邀请邮件，**他必须点接受**才能推送
#      - 权限级别默认给 push（可推送代码，但不能删仓库/改设置）
#      - 队友的 GitHub 用户名就是 github.com/ 后面那段，不是邮箱
#
#  如果队友不想被加协作者，也可以走 Fork + Pull Request 流程（见
#  05_文档/11_队友协作与同步.md），不需要本脚本。
# ============================================================================
set -e
OWNER="linzhi42506-collab"
REPO="llm-competition"
TOKEN="${1:-${GITHUB_TOKEN:-}}"
shift || true

if [ -z "${TOKEN}" ] || [ $# -eq 0 ]; then
    echo "用法: $0 <你的Token> <队友GitHub用户名> [更多用户名...]"
    echo "例:   $0 ghp_xxxx zhangsan lisi"
    exit 1
fi

for USER in "$@"; do
    printf ">>> 添加 %-24s " "${USER}"
    RESP=$(curl -s -X PUT \
        -H "Authorization: token ${TOKEN}" \
        -H "Accept: application/vnd.github+json" \
        "https://api.github.com/repos/${OWNER}/${REPO}/collaborators/${USER}" \
        -d '{"permission":"push"}')
    if echo "${RESP}" | grep -q '"invitation"\|"permissions"'; then
        echo "✓ 已发出邀请（等他点接受）"
    elif echo "${RESP}" | grep -q "Not Found"; then
        echo "✗ 找不到这个用户名（拼写错？或该账号不存在）"
    else
        echo "✗ $(echo "${RESP}" | head -c 120)"
    fi
done

echo
echo "=== 当前协作者列表 ==="
curl -s -H "Authorization: token ${TOKEN}" \
    "https://api.github.com/repos/${OWNER}/${REPO}/collaborators" \
    | python3 -c "
import sys,json
for u in json.load(sys.stdin):
    p=u.get('permissions',{})
    role='admin(管理员)' if p.get('admin') else ('push(可推送)' if p.get('push') else 'pull(只读)')
    print(f\"  {u['login']:24s} {role}\")
"
echo
echo "提醒队友："
echo "  1. 去邮箱点 GitHub 发来的邀请链接（或到 github.com/notifications 接受）"
echo "  2. 接受后执行:"
echo "     git clone https://github.com/${OWNER}/${REPO}.git"
