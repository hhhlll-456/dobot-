# dobot-

Dobot 机械臂**吸盘视觉抓取**的可复用 skill。把一次成功抓取里真正起作用的
流程、标定关系和踩过的坑沉淀下来：换相机、换机械臂、挪物体位置，仍按同一
套流程重新标定并抓取。

本 skill 对应的就是 S1：**机械臂 B 从小车 A 上取物体**。项目内的实施记录
在 `ROS2_ws/S1_取件流程.md` 第 0.1 节和 `S1-S4_全流程.md` 的 S1 段。

- 兼容 **Claude Code** 与 **opencode**（同一份 skill，安装在两边的标准目录）
- 遵循 Anthropic Agent Skills 格式：`SKILL.md` + `references/` + `scripts/`
- 面向 ROS 2 Jazzy / Dobot Magician / OpenCV / ArUco

## 这个 skill 解决什么

不是「摆好位置直接吸」的开环脚本，而是一条闭环：

```
标准位拍照
  → 码 22（吸盘）+ 码 34（小车）+ 物体识别
  → J_world：臂坐标 mm ↔ 世界 mm
  → 工具偏移：码心 ≠ 吸嘴
  → 竖直抬到悬停高度（z ≤ 70mm）
  → 悬停闭环修正 xy
  → 降到抓取高度 +12mm，再闭环一次
  → 竖直下降到抓取 z
  → 同一姿态 5 帧中位，校验「码 22 相对码 34」
  → 通过才开吸盘 → 竖直抬 → L 形返回标准位
```

核心原则：

1. **几何标定只用相机内参 + 棋盘格，绝不用 A 码。**
   码 22/34 只做识别与物理基准，不参与相机标定解算。
2. **禁止常数 x/y/z 修正。** 误差随位姿变化，必须靠两级闭环。
3. **悬停 z ≤ 70mm，lift_mm=40，全程 L 形。**
4. **每个动作都用外部观测验收。** 吸盘服务返回只是指令回显。
5. **看不清就不动臂；确认不通过就不吸。**

## 目录结构

```
skills/dobot-suction-grasp/
  SKILL.md                      # 主流程（先读这个）
  references/
    calibration.md              # 内参、棋盘世界系、J_world、工具偏移、抓取 z
    troubleshooting.md          # 症状 → 根因 → 处理
    command-reference.md        # 命令、服务、配置键、产物
  scripts/
    preflight.sh                # 抓取前一键预检（只读）
    tool_offset.py              # 工具偏移核对 / 换世界系重表达
install.sh                      # 安装到 Claude Code + opencode
```

## 安装

```bash
git clone https://github.com/hhhlll-456/dobot-.git
cd dobot-
bash install.sh
```

默认用符号链接安装到：

```
~/.claude/skills/dobot-suction-grasp
~/.config/opencode/skills/dobot-suction-grasp
```

opencode 原生也会扫描 `~/.claude/skills/`，两处都装是为了不依赖版本差异。

```bash
bash install.sh --copy       # 复制安装（仓库可移动）
bash install.sh --uninstall  # 卸载
```

安装后新开一个 Claude Code / opencode 会话，skill 就能被发现。
在 Claude Code 里可以直接说「抓取物体」，或查看 `/skills`。

## 使用

```bash
# 预检（驱动、相机、配置、J、工具偏移、状态、吸盘）
bash ~/.claude/skills/dobot-suction-grasp/scripts/preflight.sh

# 只识别、不动臂
cd /home/hanli/fyp_ws
python3 src/fyp_bringup/scripts/pick_cycle_pnp.py --dry-run

# 正式抓取
python3 src/fyp_bringup/scripts/pick_cycle_pnp.py

# 已回零/已在标准位，跳过回零
python3 src/fyp_bringup/scripts/pick_cycle_pnp.py --no-home
```

完整流程、标定顺序、换硬件后的检查单见
[`SKILL.md`](skills/dobot-suction-grasp/SKILL.md)。

## 依赖与本机路径

本 skill 记录的是本机验证过的实现，路径写死在示例里：

| 项 | 路径 |
|---|---|
| ROS 工作区 | `/home/hanli/fyp_ws` |
| 抓取脚本 | `src/fyp_bringup/scripts/pick_cycle_pnp.py` |
| 配置 | `src/fyp_bringup/config/s1_pnp.json` |
| 驱动配置 | `src/fyp_bringup/config/arm_b_local.yaml` |
| 相机标定根目录 | `/home/hanli/ros2机械臂项目/摄像头畸变矫正` |
| 内参工具 | `~/.agents/skills/plane-metric-calibration/scripts/chess_intrinsics.py` |

换机器/换工程路径时，按 `references/command-reference.md` 改配置与命令；
算法与判据不用改。

## 已验证

2026-10-19 在重新调整过的 c2（video6）相机上连续验证：

| 轮次 | 悬停残差 | 近抓取残差 | 抓取位确认 | 结果 |
|---|---|---|---|---|
| 1730 | 15.3 → 4.35 → 3.17mm | 1.92mm | 2.41mm | 成功 |
| 1755 | — | — | 0.33mm | 成功 |
| 1820 | — | — | 0.53mm | 成功 |
| 1835 | — | — | 1.20mm | 成功 |

中途两次 5.48/4.22mm 的「确认拒绝」暴露了单帧判据的噪声，之后改为
同一姿态 5 帧中位，并把容差从 4mm 放宽到物理推导的 7mm。

## 换硬件时先读

- 换相机 → `SKILL.md` 第 2 节 + `references/calibration.md`
- 换机械臂/底座 → 重示教标准位、重标 J、重做触摸标定
- 换物体 → 改 `object.height_mm / size_mm / color`，容差按
  「物体半宽 − 吸盘半径」重新推导

## 安全

- 悬停 z 硬上限 70mm；不要为了「看得更清楚」抬高。
- 先竖直抬升，再水平移动；低 y 平移先增大 x。
- 抓取位确认不通过绝不吸。
- 任何碰撞、异响、报警无法清除、状态中断，立即停机。
