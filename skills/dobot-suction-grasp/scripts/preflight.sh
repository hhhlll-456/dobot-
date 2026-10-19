#!/usr/bin/env bash
# preflight.sh — 抓取前的一键预检（只读，不动臂、不开吸盘）
#
# 用法:
#   bash preflight.sh [--config PATH] [--arm arm_b] [--ws /home/hanli/fyp_ws]
#
# 检查项:
#   1. ROS 环境与工作区
#   2. 配置文件存在且 JSON 合法、关键字段齐全
#   3. 工具偏移与当前 J 是否一致
#   4. 相机设备存在、标定会话与原图存在
#   5. 驱动服务在线、/arm_b/state 有消息（best_effort）
#   6. 吸盘当前状态、报警
#
# 退出码: 0 全通过; 1 有失败项; 2 参数/环境错误

set -u

CFG="/home/hanli/fyp_ws/src/fyp_bringup/config/s1_pnp.json"
ARM="arm_b"
WS="/home/hanli/fyp_ws"

while [ $# -gt 0 ]; do
  case "$1" in
    --config) CFG="${2:-}"; shift 2 ;;
    --arm)    ARM="${2:-}";  shift 2 ;;
    --ws)     WS="${2:-}";   shift 2 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "未知参数: $1" >&2; exit 2 ;;
  esac
done

FAIL=0
WARN=0
ok()   { printf '  [OK]   %s\n' "$*"; }
warn() { printf '  [WARN] %s\n' "$*"; WARN=$((WARN+1)); }
bad()  { printf '  [FAIL] %s\n' "$*"; FAIL=$((FAIL+1)); }
head2() { printf '\n== %s ==\n' "$*"; }

# ── 1. 环境 ────────────────────────────────────────────────────────────
head2 "1. ROS 环境"
if [ -n "${ROS_DISTRO:-}" ]; then
  ok "ROS_DISTRO=$ROS_DISTRO"
else
  warn "ROS_DISTRO 为空；请先 source /opt/ros/jazzy/setup.bash"
fi
if [ -f "$WS/install/setup.bash" ]; then
  ok "工作区 install 存在: $WS/install/setup.bash"
  # ROS 的 setup.bash 里有未定义变量，set -u 下 source 会直接退出
  set +u
  # shellcheck disable=SC1090
  source "$WS/install/setup.bash" 2>/dev/null
  _src_rc=$?
  set -u
  [ "$_src_rc" -eq 0 ] || warn "source $WS/install/setup.bash 失败"
else
  bad "找不到 $WS/install/setup.bash（先 colcon build）"
fi
if command -v ros2 >/dev/null 2>&1; then
  ok "ros2: $(command -v ros2)"
else
  bad "ros2 不在 PATH（source 环境后再试）"
fi

# ── 2. 配置 ────────────────────────────────────────────────────────────
head2 "2. 配置 $CFG"
if [ ! -f "$CFG" ]; then
  bad "配置文件不存在"
else
  ok "配置文件存在"
  CFG_OUT="$(python3 - "$CFG" <<'PY'
import json, sys
from pathlib import Path
p = Path(sys.argv[1])
try:
    c = json.loads(p.read_text(encoding="utf-8"))
except Exception as e:
    print("PARSE_ERROR", e)
    raise SystemExit(0)
m = c.get("motion", {})
cam = c.get("camera", {})
obj = c.get("object", {})
req = {
    "camera.dev": cam.get("dev"),
    "camera.label": cam.get("label"),
    "camera.intrinsics_file": cam.get("intrinsics_file"),
    "camera.calib_file": cam.get("calib_file"),
    "motion.j_world": m.get("j_world"),
    "motion.tool_offset_arm_mm": m.get("tool_offset_arm_mm"),
    "motion.tool_offset_world_mm": m.get("tool_offset_world_mm"),
    "motion.code_above_tip_mm": m.get("code_above_tip_mm"),
    "motion.lift_mm": m.get("lift_mm"),
    "motion.max_hover_z_mm": m.get("max_hover_z_mm"),
    "object.height_mm": obj.get("height_mm"),
    "object.size_mm": obj.get("size_mm"),
}
missing = [k for k, v in req.items() if v is None]
print("MISSING", ",".join(missing) if missing else "-")
for k in ("intrinsics_file", "calib_file"):
    v = cam.get(k)
    print(("EXISTS" if v and Path(v).is_file() else "NOFILE"), k, v)
print("DEV", cam.get("dev"))
print("LABEL", cam.get("label"))
print("LIFT", m.get("lift_mm"), "HOVER_MAX", m.get("max_hover_z_mm"))
print("J", json.dumps(m.get("j_world")))
print("OFF_ARM", json.dumps(m.get("tool_offset_arm_mm")))
print("OFF_WORLD", json.dumps(m.get("tool_offset_world_mm")))
print("TRIM", json.dumps(m.get("grasp_trim_arm_mm")))
# 标定会话结构
cal = cam.get("calib_file")
if cal:
    session = Path(cal).parent.parent
    rounds = session / "标定轮次"
    print(("EXISTS" if rounds.is_dir() else "NOROUNDS"), "rounds", rounds)
    if rounds.is_dir():
        dev, label = cam.get("dev"), cam.get("label")
        n = 0
        try:
            data = json.loads(Path(cal).read_text(encoding="utf-8"))
            for rnd, b in data.get("boards", {}).items():
                img = rounds / rnd / f"video{dev}_1600x1200.png"
                if label in b.get("seen_by", []) and img.is_file():
                    n += 1
        except Exception:
            pass
        print("ROUNDS_FOR_CAM", n)
PY
)"
  if printf '%s\n' "$CFG_OUT" | grep -q '^PARSE_ERROR'; then
    bad "JSON 解析失败: $(printf '%s\n' "$CFG_OUT" | head -1)"
  else
    MISS="$(printf '%s\n' "$CFG_OUT" | awk '$1=="MISSING"{print $2}')"
    if [ "$MISS" = "-" ] || [ -z "$MISS" ]; then ok "关键字段齐全"; else bad "缺字段: $MISS"; fi
    printf '%s\n' "$CFG_OUT" | awk '$1=="NOFILE"{printf "  [FAIL] %s 指向的文件不存在: %s\n", $2, $3}'
    printf '%s\n' "$CFG_OUT" | awk '$1=="NOFILE"{c++} END{exit (c>0)?0:1}' && FAIL=$((FAIL+1))
    ROUNDS="$(printf '%s\n' "$CFG_OUT" | awk '$1=="ROUNDS_FOR_CAM"{print $2}')"
    if [ "${ROUNDS:-0}" -ge 1 ]; then
      ok "本相机可用棋盘轮次: $ROUNDS"
    else
      bad "本相机没有可用棋盘轮次（位姿会退回 homography/cached）"
    fi
    TRIM="$(printf '%s\n' "$CFG_OUT" | sed -n 's/^TRIM //p' | tr -d ' ')"
    if [ "$TRIM" = "[0,0,0]" ] || [ "$TRIM" = "[0.0,0.0,0.0]" ]; then
      ok "grasp_trim_arm_mm 为 0（符合禁止常数修正）"
    else
      warn "grasp_trim_arm_mm=$TRIM 非零（代码会忽略，但应改回 0）"
    fi
  fi
fi

# ── 3. 相机设备 ────────────────────────────────────────────────────────
head2 "3. 相机设备"
DEV="$(printf '%s\n' "${CFG_OUT:-}" | awk '$1=="DEV"{print $2}')"
if [ -n "${DEV:-}" ] && [ -e "/dev/video$DEV" ]; then
  ok "/dev/video$DEV 存在"
  if command -v fuser >/dev/null 2>&1; then
    if fuser -v "/dev/video$DEV" >/dev/null 2>&1; then
      warn "/dev/video$DEV 正被进程占用（抓取前必须释放）"
    else
      ok "/dev/video$DEV 未被占用"
    fi
  else
    warn "fuser 不可用，跳过占用检查"
  fi
else
  bad "/dev/video${DEV:-?} 不存在（设备号可能已变）"
  echo "         现有设备:"
  for d in /dev/video*; do
    [ -e "$d" ] || continue
    model="$(udevadm info -q property -n "$d" 2>/dev/null | grep -m1 '^ID_MODEL=' | cut -d= -f2)"
    echo "           $d  ${model:-unknown}"
  done
fi

# ── 4. 驱动与状态 ──────────────────────────────────────────────────────
head2 "4. 驱动 /arm_b"
if command -v ros2 >/dev/null 2>&1; then
  SVCS="$(timeout 10 ros2 service list 2>/dev/null | grep -E "^/$ARM/(home|suction|clear_alarms)$" || true)"
  if printf '%s\n' "$SVCS" | grep -q "/$ARM/home"; then
    ok "服务在线: $(printf '%s' "$SVCS" | tr '\n' ' ')"
  else
    bad "找不到 /$ARM/home 等服务（驱动没起？命名空间错了？）"
  fi

  STATE="$(timeout 8 ros2 topic echo "/$ARM/state" --once \
    --qos-reliability best_effort 2>/dev/null || true)"
  if [ -z "$STATE" ]; then
    bad "/$ARM/state 5–8s 内没有消息"
    echo "         查: 驱动进程 fd 是否有 (deleted) tty; 串口是否用 by-id; 是否起了两个驱动"
  else
    ok "/$ARM/state 有消息"
    printf '%s\n' "$STATE" | grep -E '^(x_mm|y_mm|z_mm|r_deg|suction_on|connected|estop):' \
      | sed 's/^/         /'
    SUP="$(printf '%s\n' "$STATE" | awk '/^suction_on:/{print $2}')"
    ESTOP="$(printf '%s\n' "$STATE" | awk '/^estop:/{print $2}')"
    CONN="$(printf '%s\n' "$STATE" | awk '/^connected:/{print $2}')"
    [ "${CONN:-false}" = "true" ] && ok "connected=true" || bad "connected=$CONN"
    if [ "${ESTOP:-false}" = "true" ]; then bad "急停锁定中"; else ok "estop=false"; fi
    if [ "${SUP:-false}" = "true" ]; then
      warn "吸盘当前是开的（脚本启动时会自动关；若吸着物体请先人工取下）"
    else
      ok "吸盘当前关闭"
    fi
    ALARM="$(printf '%s\n' "$STATE" | awk '/^alarm_codes:/{print}')"
    if printf '%s\n' "$ALARM" | grep -qE '^alarm_codes: \[\]|^alarm_codes: \[\]$|^alarm_codes:$'; then
      ok "无报警"
    else
      warn "报警列表非空: $ALARM（先排查再 clear_alarms）"
    fi
  fi
else
  warn "跳过驱动检查（ros2 不可用）"
fi

# ── 5. 工具偏移一致性 ──────────────────────────────────────────────────
head2 "5. 工具偏移 / J"
if [ -f "$CFG" ] && command -v python3 >/dev/null 2>&1; then
  python3 - "$CFG" <<'PY'
import json, sys
import numpy as np
c = json.loads(open(sys.argv[1], encoding="utf-8").read())
m = c.get("motion", {})
try:
    J = np.asarray(m["j_world"], float)
    u = np.asarray(m["tool_offset_arm_mm"], float)
    w_stored = np.asarray(m.get("tool_offset_world_mm", [0, 0]), float)
    w_calc = J @ u
    d = float(np.linalg.norm(w_calc - w_stored))
    print(f"  [{'OK' if d <= 0.5 else 'FAIL'}]  臂坐标偏移 {u.round(3).tolist()} "
          f"→ 当前 J 表达 {w_calc.round(2).tolist()} mm；"
          f"配置存 {w_stored.round(2).tolist()} mm；偏差 {d:.2f} mm")
    if d > 0.5:
        print("         用 scripts/tool_offset.py --write 重表达，或重做触摸标定")
except Exception as e:
    print(f"  [FAIL]  工具偏移/J 计算失败: {e}")
PY
else
  warn "跳过工具偏移检查"
fi

# ── 汇总 ───────────────────────────────────────────────────────────────
head2 "汇总"
printf '  失败 %d 项，警告 %d 项\n' "$FAIL" "$WARN"
if [ "$FAIL" -gt 0 ]; then
  echo "  ❌ 预检未通过，先修失败项再动臂"
  exit 1
fi
if [ "$WARN" -gt 0 ]; then
  echo "  ⚠ 预检通过，但有警告；确认无害后再继续"
else
  echo "  ✅ 预检通过"
fi
exit 0
