#!/usr/bin/env bash
# install.sh — 把 dobot-suction-grasp 安装到本机，Claude Code 与 opencode 都能发现
#
# 用法:
#   bash install.sh              # 默认用符号链接（推荐，仓库即源）
#   bash install.sh --copy       # 复制一份（仓库移动后仍可用）
#   bash install.sh --force      # 覆盖已存在的非本仓库安装
#   bash install.sh --uninstall  # 删除本仓库创建的链接/副本
#
# 安装位置:
#   ~/.claude/skills/dobot-suction-grasp            (Claude Code)
#   ~/.config/opencode/skills/dobot-suction-grasp   (opencode)
#
# opencode 原生也会扫描 ~/.claude/skills/，两处都装是为了不依赖版本差异。

set -eu

NAME="dobot-suction-grasp"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$HERE/skills/$NAME"
CLAUDE_DIR="$HOME/.claude/skills"
OPENCODE_DIR="$HOME/.config/opencode/skills"

MODE="link"
FORCE=0
UNINSTALL=0

while [ $# -gt 0 ]; do
  case "$1" in
    --copy) MODE="copy"; shift ;;
    --force) FORCE=1; shift ;;
    --uninstall) UNINSTALL=1; shift ;;
    -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
    *) echo "未知参数: $1" >&2; exit 2 ;;
  esac
done

if [ ! -f "$SRC/SKILL.md" ]; then
  echo "❌ 找不到 $SRC/SKILL.md" >&2
  exit 2
fi

# frontmatter 自检：name 必须与目录名一致，description 不能为空
python3 - "$SRC/SKILL.md" "$NAME" <<'PY'
import re, sys
from pathlib import Path
p = Path(sys.argv[1]); name = sys.argv[2]
text = p.read_text(encoding="utf-8")
m = re.match(r"^---\n(.*?)\n---\n", text, re.S)
if not m:
    raise SystemExit("❌ SKILL.md 缺少 YAML frontmatter")
fm = m.group(1)
n = re.search(r"^name:\s*(.+)$", fm, re.M)
d = re.search(r"^description:\s*(.+)$", fm, re.M)
if not n or n.group(1).strip() != name:
    raise SystemExit(f"❌ frontmatter name 必须是 {name!r}")
if not d or len(d.group(1).strip()) < 20:
    raise SystemExit("❌ frontmatter description 缺失或过短")
if not re.fullmatch(r"[a-z0-9]+(-[a-z0-9]+)*", name):
    raise SystemExit("❌ name 必须是小写 kebab-case")
print(f"✅ frontmatter 自检通过: {name}")
PY

is_ours() {
  local dst="$1"
  if [ -L "$dst" ]; then
    [ "$(readlink -f "$dst")" = "$(readlink -f "$SRC")" ] && return 0
  fi
  if [ -f "$dst/.installed-from" ]; then
    [ "$(cat "$dst/.installed-from")" = "$SRC" ] && return 0
  fi
  return 1
}

remove_target() {
  local dst="$1"
  if [ -L "$dst" ] || [ -e "$dst" ]; then
    if is_ours "$dst"; then
      rm -rf "$dst"
      echo "   已移除旧安装 $dst"
    elif [ "$UNINSTALL" -eq 1 ]; then
      echo "   ⚠ $dst 不是本仓库安装，未删除（请手动确认）"
    elif [ "$FORCE" -eq 1 ]; then
      rm -rf "$dst"
      echo "   已覆盖 $dst"
    else
      echo "❌ 目标已存在且不是本仓库安装: $dst" >&2
      echo "   用 --force 覆盖，或先手动移走" >&2
      exit 1
    fi
  fi
}

install_to() {
  local base="$1"
  local dst="$base/$NAME"
  mkdir -p "$base"
  remove_target "$dst"
  if [ "$UNINSTALL" -eq 1 ]; then
    return 0
  fi
  if [ "$MODE" = "copy" ]; then
    cp -a "$SRC" "$dst"
    printf '%s\n' "$SRC" > "$dst/.installed-from"
    echo "   已复制 → $dst"
  else
    ln -s "$SRC" "$dst"
    echo "   已链接 → $dst"
  fi
}

echo "== 安装 $NAME =="
echo "源: $SRC"
install_to "$CLAUDE_DIR"
install_to "$OPENCODE_DIR"

if [ "$UNINSTALL" -eq 1 ]; then
  echo "✅ 已卸载 $NAME"
  exit 0
fi

echo
echo "✅ 安装完成"
echo "   Claude Code: 新开会话后应能看到 $NAME skill"
echo "   opencode:    同上（两处都已安装）"
echo
echo "   验证: 在 Claude Code 里输入 /skills 或直接说「抓取物体」"
echo "   预检: bash \"$SRC/scripts/preflight.sh\""
