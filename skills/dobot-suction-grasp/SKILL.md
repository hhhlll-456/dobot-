---
name: dobot-suction-grasp
description: 用 ROS 2 + MoveToPose 驱动 Dobot 机械臂，通过吸盘从桌面或小车上抓取物体：标准位拍照 → J_world 像素-毫米标定 → 小车码相对跟踪 → 悬停/近抓取两级视觉闭环 → 抓取位两码相对位置校验 → 开吸盘抬升。适用于换相机、换机械臂、挪动物体位置后，仍按同一套流程重新标定并复跑。当用户说「抓取物体/吸取/吸起」「抓不到/吸不上/没开吸盘」「换相机或换臂后还要抓」「标定 J_world、工具偏移、抓取高度」，或提到 Dobot suction grasp、visual servoing、pick and place 时使用；只要任务涉及本项目的 S1 吸盘抓取，即使没说“skill”也应主动使用。不用于纯图像识别或四相机融合标定本身。
compatibility: Linux + ROS 2 Jazzy + /home/hanli/fyp_ws；Claude Code 与 opencode 都可读
---

# Dobot 吸盘视觉抓取（S1）

把「相机看到物体」变成「吸嘴落到物体上、吸住、抬走」的完整闭环。
**本 skill 就是 S1「机械臂 B 从小车 A 取物体」的操作版**；项目内的实施记录见
`/home/hanli/ros2机械臂项目/ROS2_ws/S1_取件流程.md` 第 0.1 节与
`S1-S4_全流程.md` 的 S1 段。
本 skill 的命令默认在本机工作区 `/home/hanli/fyp_ws` 执行，脚本是
`src/fyp_bringup/scripts/pick_cycle_pnp.py`，配置是
`src/fyp_bringup/config/s1_pnp.json`。

**这不是「摆好位置直接吸」的开环脚本。** 全部精度来自：
标定关系（内参 + 棋盘 + J_world + 工具偏移）+ 两级视觉闭环 + 抓取前外部观测校验。

---

## 硬约束（违反任何一条就停下来）

1. **几何标定与融合只用相机内参和棋盘格，绝不用 A 码。**
   A 码（桌面 0–21）只做验收/诊断；码 22（吸盘）和码 34（小车）只做
   识别与物理基准，不参与相机标定解算。
2. **禁止常数 x/y/z 修正。** 同一目标在不同位姿需要的修正量不同，
   常数 trim 只是碰巧对某一次有效。误差必须靠「悬停闭环 + 近抓取高度
   再闭环」消掉。`grasp_trim_arm_mm` 必须保持 `[0,0,0]`（代码会忽略非零值）。
3. **悬停高度硬上限 z ≤ 70mm。** `lift_mm=40`（标准位 z=21.44 → 悬停 61.44）。
   抬到 111mm 时臂接近全伸奇异位形，水平 MOVL 会报 `0x22`。
4. **先竖直抬升，再水平移动，最后竖直下降（L 形）。** 禁止从低位斜线
   飞过小车。低 y（<40mm）平移必须先增大 x 再走 y（`safe_goto`）。
5. **每个动作都要外部观测验收。** 吸盘服务返回只是指令回显，不代表吸住；
   抓取位确认不通过就**不吸**，抬回悬停位、回标准位重试。
6. **看不清码或物体就停，不动臂。** 每次运行都可以 Ctrl-C；异常分支会
   尝试安全回标准位。吸盘启动时必须关闭（脚本会先发 `suction=False`）。

---

## 0. 每次开工的预检

```bash
source /opt/ros/jazzy/setup.bash
source /home/hanli/fyp_ws/install/setup.bash

# 0.1 驱动必须只有一个实例，串口用 by-id（换 USB 口后 ttyUSBn 会变）
ros2 launch fyp_bringup arm.launch.py arm_name:=arm_b \
  params_file:=/home/hanli/fyp_ws/install/fyp_bringup/share/fyp_bringup/config/arm_b_local.yaml

# 0.2 状态话题是 best_effort QoS，必须显式指定
ros2 topic echo /arm_b/state --once --qos-reliability best_effort

# 0.3 吸盘关、必要时清报警
ros2 service call /arm_b/suction fyp_interfaces/srv/Suction \
  "{on: false, wait_done: true}"
ros2 service call /arm_b/clear_alarms std_srvs/srv/Trigger "{}"
```

一键检查（配置路径、设备、节点、状态、吸盘）：

```bash
bash <skill>/scripts/preflight.sh
```

通过信号：打印 state 的 x/y/z；相机设备存在；配置里 `j_world`、
`tool_offset_*`、`code_above_tip_mm` 都在；没有残留报警。

**启动后 20s 仍无 `/arm_b/state`**：先查驱动进程的 fd 是否攥着已删除的
tty（`ls -l /proc/<pid>/fd | grep deleted`），再确认串口 by-id 路径。
换相机时 USB 重新枚举是常见触发点。

---

## 1. 先判断这次要不要重标定

| 变化 | 必须做什么 |
|---|---|
| 只挪了物体位置（车/桌面） | 不用标定。直接 `--dry-run` 确认看得见，再正式抓 |
| 换了相机、改了相机位置/角度/焦距 | 内参（若镜头/分辨率变）→ 重拍棋盘轮次 → 更新 `camera.calib_file` → 重标 J → 重表达工具偏移 → 重新确认抓取 z |
| 机械臂底座挪了 / 换了臂 | 重新示教标准位 → 重标 J → 触摸标定工具偏移与 `code_above_tip_mm` → 重新锚定抓取 z |
| 只换了物体（尺寸/高度/颜色） | 改 `object.size_mm / height_mm / color`；按 `height_mm` 重算抓取 z；容差按物体半宽重新推 |
| 只换吸盘/码 22 安装位置 | 重做触摸标定（offset + `code_above_tip_mm`）；J 不用重标 |

**不要每次抓取都跑 `--calibrate-j`。** J 探针会在标准位附近移动
±20/±30mm，可能把小车碰走；配置里已有同一套底座/相机标定的 J 就直接用。

---

## 2. 换相机后的标定顺序（最容易出错的一段）

详细步骤、公式与验收阈值见
`references/calibration.md`；这里只给顺序和关键事实。

1. **内参**：1600×1200、`CAP_V4L2 + MJPG`；RMS ≤ 0.6px；输出
   `camN/calib_intrinsics_1600.json`。四台 USB2 相机**不能同时打开**，
   必须逐台采集。
2. **棋盘轮次 + 世界系**：用 `four-camera-seam-fusion` skill 重拍接缝
   棋盘并解算，得到新的 `<session>/检查/棋盘标定.json`。世界系的参考棋盘
   一变，旧 `j_world` 立即失效。S1 相机必须至少在一轮里看到棋盘，且该轮
   出现在 JSON 的 `boards[round].seen_by` 中。
3. **改配置**：`camera.dev`、`camera.label`、`intrinsics_file`、`calib_file`
   指向新会话；保持 `refresh_pose: true`。
4. **自检**：`--dry-run`。看打印的位姿来源必须是 `chess_rounds`、RMS
   合理（约 0.3–1.5px）；如果是 `homography` 或 `cached`，说明原图没找到，
   精度不够，先修标定。
5. **重标 J**：`pick_cycle_pnp.py --calibrate-j`。验收：尺度 0.85–1.15
   （目标≈1.0）、轴夹角 ≤15°（目标≈90°）、`kz` 0.9–1.1。结果自动写回配置。
   **J 是「臂坐标 mm → 世界 mm」，det<0 正常。**
6. **重表达工具偏移**（见下一节）。世界系变了以后，绝不能沿用旧的
   `tool_offset_world_mm` 数值。
7. **重新确认抓取 z**（见第 4 节），先 `--dry-run` 看自动 z 是否落在
   合理范围（约 15–35mm），再用 `--grasp-z` 或接触标定兜底。

> 只换一台相机也要重跑整套接缝标定：世界系由参考棋盘定义，单换相机
> 位姿而不重解会得到「新相机 + 旧世界」的错配。

---

## 3. 工具偏移：码心不是吸嘴

触摸标定实测：码 22 中心相对吸嘴有约 12mm 水平偏移，且码心比吸嘴尖高
86.12mm。代码里：

```
期望码心位置 = 物体目标位置 + tool_offset_world_mm
tool_offset_world_mm = J_current · tool_offset_arm_mm
```

- **工具偏移是刚体量，存臂坐标** `tool_offset_arm_mm`（本机
  `(-5.633, -10.984)`）。
- 换世界系 / 换 J 后，用**新 J** 乘这个臂坐标值重新表达，得到新的
  `tool_offset_world_mm`（本机新系为 `(-10.90, -5.20)`，模长 12.07mm）。
- 换吸盘、换码 22 支架或码贴纸位置后，必须重做触摸标定，不能只改常数。
- 检查：

```bash
python3 <skill>/scripts/tool_offset.py \
  --config /home/hanli/fyp_ws/src/fyp_bringup/config/s1_pnp.json
```

输出会同时给出「臂坐标值」「当前 J 表达的世界值」「配置里存的世界值」，
三者不一致就说明配置没跟上 J。

---

## 4. 抓取高度 z：优先自动，偏置时用物理锚点

自动公式（`auto_grasp_z=true`）：

```
吸嘴当前高 tip_h = 码高 H_code - code_above_tip_mm
目标高 desired  = object.height_mm + grasp_clearance_mm
Δz_arm          = (desired - tip_h) / kz
抓取 z          = base.z + Δz_arm + grasp_z_calib_mm
```

要点：

- **只用 `kz`，不用高度对臂 x/y 的梯度**。`gx,gy` 主要来自小码深度
  测量偏差，计入会把 z 带偏约 20mm。
- 码高用**同一姿态 5 帧中位数**（单帧噪声约 ±1.2mm）。
- 倾斜的小码会让码高读数偏高（码倾角越大偏得越多）。若自动 z 明显
  偏高/偏低（例如算出 −32.8mm），**不要硬跑**：先用物理锚点确认
  「标准位 z 时吸嘴尖实际离桌面多高」，用 `--grasp-z <值>` 跑一轮，
  再决定是否更新 `tilt/grasp_z_calib_mm`。
- 近抓取高度会按当地码高再重算一次 z（限幅
  `approach_z_max_corr_mm=±15mm`），之后统一加
  `grasp_z_calib_mm`（本机 −7.0，仅对自动 z 生效）。
- `--grasp-z` 优先级最高，且不叠加 `grasp_z_calib_mm`。
- 每次重试失败后 z 再降 `z_retry_step_mm=5mm`，最多 `max_attempts` 次。

---

## 5. 正式抓取流程（脚本内部顺序）

1. 吸盘关 → 回零（`--no-home` 可跳过）→ 到标准位。
2. 标准位拍照（此时无遮挡）：码 22、物体、小车码 34 都要看到。
3. 记录 `q_obj_cart = P_obj - P_34`（物体在小车坐标系里的位置）。
4. 算臂目标：`V = (P_obj + tool_offset) - P_22`，`Δ = J⁻¹ · V`。
5. **竖直抬到悬停 z**（标准位正上方）。
6. **悬停闭环**：每帧目标 = `P_34_now + q_obj_cart + tool_offset`，
   水平修正到 ≤ `hover_tol_mm=4mm`（最多 2 次）。
7. **降到抓取 z + 12mm，再闭环一次**（≤2mm，步长 ≤6mm），并用当地
   码高重算最终 z。这一步消除「悬停对齐 ≠ 抓取处对齐」的位姿相关误差。
8. **竖直下降到抓取 z**（速度 8%），打印实际到位与指令差。
9. **抓取位确认（吸之前）**：同一姿态连抓 5 帧，取
   `q = P_22 - P_34` 的中位数；与 `q_obj_cart + tool_offset` 比较。
   差 ≤ `cart_verify_tol_mm=7mm` → 开吸盘；否则**不吸**，抬回、回标准位重试。
10. 开吸盘 → 竖直抬到悬停 z → `safe_goto` 回标准位上方 → 下降回标准位。
11. 成功后吸盘保持开启（物体在吸嘴上）；要放回加 `--place-back`。
12. 失败最多 `max_attempts=2` 次；仍失败就停并报人工，不继续瞎试。

**为什么用码 22 相对码 34 判成功**：下降后机械臂会挡住物体，颜色块
不可靠；而码 22 和码 34 在抓取高度都可见，相对位置是唯一可靠判据。

**为什么容差是 7mm**：物体半宽 12.5mm、吸盘半径约 5mm，吸嘴中心偏 7mm
仍基本在顶面上。4mm 会误拒肉眼已在物体上的抓取。若换更小的物体，
按「半宽 − 吸盘半径」重新推导，不要照抄 7。

---

## 6. 常用命令

```bash
cd /home/hanli/fyp_ws

# 只识别不动臂（臂已在标准位）
python3 src/fyp_bringup/scripts/pick_cycle_pnp.py --dry-run

# 正式抓取（回零 → 标准位 → 识别 → 闭环 → 吸 → 抬 → 验证）
python3 src/fyp_bringup/scripts/pick_cycle_pnp.py

# 已经回零/已在标准位时，跳过回零再抓
python3 src/fyp_bringup/scripts/pick_cycle_pnp.py --no-home

# 指定抓取高度（自动 z 有偏置时）
python3 src/fyp_bringup/scripts/pick_cycle_pnp.py --grasp-z 25.3

# 连抓 2 轮；抓完放回
python3 src/fyp_bringup/scripts/pick_cycle_pnp.py --repeat 2
python3 src/fyp_bringup/scripts/pick_cycle_pnp.py --place-back

# 重标 J（只在世界系/底座变化后）
python3 src/fyp_bringup/scripts/pick_cycle_pnp.py --calibrate-j

# 重新示教标准位（交互式）
python3 src/fyp_bringup/scripts/jog_arm.py --arm arm_b --save-name b_standard
```

产物默认写到 `src/fyp_bringup/runs/<时间戳>/`：每阶段的标注图
`r*_base.jpg`、`r*a*_hover.jpg`、`r*a*_approach.jpg`、`r*a*_graspcheck.jpg`，
以及对应 JSON（含每帧残差、码高、两码相对位置、`d_mm`、`aligned`）。
**先看 JSON 再看图**，数值比肉眼快。

---

## 7. 失败时先看什么

完整「现象 → 原因 → 处理」表见 `references/troubleshooting.md`。最常见的六类：

| 现象 | 首先检查 |
|---|---|
| 启动后无 state / `wait_state` 超时 | 串口 by-id、驱动是否攥着已删除的 tty、是否起了两个驱动 |
| 动作报 `0x22` | 悬停 z 是否 >70、是否从低位斜线平移、低 y 是否没先增大 x |
| 吸嘴在物体上但没吸住 | 抓取位确认是否 `aligned=false`（脚本故意不吸）、z 是否高了几毫米 |
| 确认差 4–7mm 被拒 | 是否单帧判据、码 34 姿态跳支；看 `q_std`，用 5 帧中位 |
| 码高/抓取 z 离谱 | 码 22 倾角、重投影 RMS；改用物理锚点 + `--grasp-z` |
| 坐标整体偏了 | 相机/世界系是否变过；`--dry-run` 的位姿来源是不是 `chess_rounds` |

---

## 8. 参考文件

- `references/calibration.md` —— 内参、棋盘轮次、J 标定、触摸标定、
  工具偏移重表达、抓取 z 锚定的完整公式与验收阈值。
- `references/troubleshooting.md` —— 症状、根因、修复命令；
  含串口/USB、QoS、码检测、标定错配、报警处理。
- `references/command-reference.md` —— 全部命令、服务、消息字段、
  配置键、产物与日志位置。

---

## 9. 每次运行后的汇报模板

```
配置: s1_pnp.json（相机 video6/c2，位姿 chess_rounds RMS x.xx px）
J: [..]（taught 时间；scale x.xx，skew xx.x°，kz x.xxx）
工具偏移: world (x.xx, y.xx) / arm (x.xx, y.xx)
抓取 z: xx.xx（来源：自动 / --grasp-z / 接触标定）
悬停残差: x.xx → x.xx mm；近抓取残差: x.xx mm
抓取位确认: d=x.xx mm（5 帧，std x.xx）→ aligned true/false
结果: 成功 / 失败（第几轮第几次、停在哪一步）
产物: src/fyp_bringup/runs/<时间戳>/
物理复核: 物体是否离开原位（看抓后标准位照片/人工确认）
```

只报数值和结论，不要只说「成功了」。
