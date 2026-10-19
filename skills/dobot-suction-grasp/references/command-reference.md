# 命令与接口速查

**目录**

0. 环境
1. 驱动
2. 状态与手动动作（服务、MoveToPose）
3. 抓取脚本 flags
4. 配置键
5. 标定相关命令
6. 产物与日志
7. 快速判断表

工作区：`/home/hanli/fyp_ws`
配置：`src/fyp_bringup/config/s1_pnp.json`
抓取脚本：`src/fyp_bringup/scripts/pick_cycle_pnp.py`
示教点：`src/fyp_bringup/config/taught_points.json`（`standard_poses.b_standard`）

---

## 0. 环境

```bash
source /opt/ros/jazzy/setup.bash
source /home/hanli/fyp_ws/install/setup.bash
```

所有 ROS 命令都依赖这两个 source。新开终端必须重做。

---

## 1. 驱动

```bash
ros2 launch fyp_bringup arm.launch.py arm_name:=arm_b \
  params_file:=/home/hanli/fyp_ws/install/fyp_bringup/share/fyp_bringup/config/arm_b_local.yaml
```

- `install/.../arm_b_local.yaml` 是指向 `src/...` 的符号链接，改源码配置
  立即生效，不用重装。
- 只允许一个驱动实例。
- 串口必须是 `/dev/serial/by-id/...`，不能是 `/dev/ttyUSBn`。

停止：前台 `Ctrl-C`；后台任务用进程管理停掉，不要直接拔 USB。

---

## 2. 状态与手动动作

### 状态

```bash
ros2 topic echo /arm_b/state --once --qos-reliability best_effort
```

字段（`fyp_interfaces/msg/DeviceState`，设备原生单位 mm/度）：

```
device, connected, estop
x_mm, y_mm, z_mm, r_deg
joint_deg[4]
suction_on          # 指令状态，不是真空回读
alarm_codes[], alarm_text
```

### 回零

```bash
ros2 service call /arm_b/home std_srvs/srv/Trigger "{}"
```

### 清报警

```bash
ros2 service call /arm_b/clear_alarms std_srvs/srv/Trigger "{}"
```

丢步报警必须先回零，不能只清报警。

### 吸盘

```bash
# 开
ros2 service call /arm_b/suction fyp_interfaces/srv/Suction \
  "{on: true, wait_done: true}"

# 关（破真空）
ros2 service call /arm_b/suction fyp_interfaces/srv/Suction \
  "{on: false, wait_done: true}"
```

`Suction.srv`：请求 `bool on, bool wait_done`；响应
`bool ok, string message, bool is_on`。`is_on` 只是指令回显。

### 直接运动（谨慎，仅调试/救援）

MoveToPose 用**米**，姿态只用绕 Z 的 yaw（四元数 x/y 置 0）：

```bash
ros2 action send_goal /arm_b/move_to_pose \
  fyp_interfaces/action/MoveToPose \
  "{mode: 2,
    target: {position: {x: 0.16042, y: 0.13, z: 0.06144},
             orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
    speed_ratio: 15.0}" --feedback
```

- `mode: 2` = MOVL（直线，XY 微调不跑 z）；`mode: 1` = MOVJ。
- 状态码：`0 SUCCESS`、`1 REJECTED_LIMIT`、`2 REJECTED_ALARM`、
  `3 REJECTED_ESTOP`、`4 NOT_CONNECTED`、`5 EXEC_TIMEOUT`、
  `6 ARRIVAL_FAILED`。
- 手动动臂前先确认吸盘状态、周围无人、目标在软限位内。

---

## 3. 抓取脚本 flags

```bash
cd /home/hanli/fyp_ws
python3 src/fyp_bringup/scripts/pick_cycle_pnp.py [flags]
```

| flag | 作用 | 何时用 |
|---|---|---|
| `--dry-run` | 只识别、算目标，不动臂、不回零 | 每次标定/换相机后先跑 |
| `--calibrate-j` | 回零 + 探针 + 拟合 J，写回配置后退出 | 世界系/底座变化后 |
| `--recalibrate-j` | 同上，但继续本次抓取 | 需要边标边抓时 |
| `--no-home` | 跳过回零（仍 `goto(base)`） | 已回零且臂在标准位 |
| `--grasp-z <mm>` | 指定抓取 z，不叠加 `grasp_z_calib_mm` | 自动 z 有已知偏置 |
| `--place-back` | 抓完放回抓取点 | 需要归位 |
| `--repeat <n>` | 连续抓 n 轮 | 批量测试 |
| `--max-attempts <n>` | 每轮最多尝试次数（默认 2） | 物体较小/位置难时 |
| `--color <name>` | 指定物体颜色 | 背景有同色物体 |
| `--config <path>` | 用别的配置 | 多工位 |
| `--out-dir <path>` | 指定产物目录 | 复现实验 |
| `--home-only` | 回零 + 到标准位后退出 | 只做定位 |
| `--tool-x/--tool-y` | **已废弃，忽略** | 不要用 |

### 典型命令

```bash
# 只识别
python3 src/fyp_bringup/scripts/pick_cycle_pnp.py --dry-run

# 正式抓取
python3 src/fyp_bringup/scripts/pick_cycle_pnp.py

# 不重拍/不回零，连抓
python3 src/fyp_bringup/scripts/pick_cycle_pnp.py --no-home --repeat 2

# 指定 z
python3 src/fyp_bringup/scripts/pick_cycle_pnp.py --grasp-z 25.3

# 抓完放回
python3 src/fyp_bringup/scripts/pick_cycle_pnp.py --place-back
```

### 退出码

- `0`：全部轮次成功（或 `--dry-run`/`--calibrate-j` 正常结束）
- `1`：抓取失败、中断、或某轮未成功

---

## 4. 配置键（`s1_pnp.json`）

### camera

| 键 | 含义 |
|---|---|
| `dev` | `/dev/videoN` 的 N |
| `label` | 标定 JSON 里的相机名（`c1`–`c4`） |
| `intrinsics_file` | 1600×1200 内参 JSON |
| `calib_file` | `<session>/检查/棋盘标定.json` |
| `refresh_pose` | 每次从标定原图重算位姿（保持 true） |
| `pose` | 可选缓存；有它且 refresh 失败时兜底 |

### suction / object

| 键 | 本机值 | 含义 |
|---|---|---|
| `suction.id` | 22 | 吸盘码 |
| `suction.marker_mm` | 30.0 | 码边长 |
| `suction.max_tilt_deg` | 45.0 | 倾角上限 |
| `object.color` | auto | 颜色筛选 |
| `object.height_mm` | 105.0 | 物体高度（影响 z 与反投影） |
| `object.size_mm` | 25.0 | 正方形边长 |
| `object.max_distance_from_suction_mm` | 320.0 | 误检过滤 |

### motion（核心）

| 键 | 本机值 | 含义 |
|---|---|---|
| `arm` | arm_b | ROS 命名空间 |
| `standard_point` | b_standard | 示教点名 |
| `speed_ratio` | 15.0 | 平移速度 % |
| `j_world` | 2×2 | 臂 mm → 世界 mm |
| `j_meta.kz` | ~1.0166 | d(高度)/d(臂 z) |
| `lift_mm` | 40.0 | 悬停抬升 |
| `max_hover_z_mm` | 70.0 | 悬停硬上限 |
| `tool_offset_world_mm` | [-10.9,-5.2] | 当前 J 表达的码心偏移 |
| `tool_offset_arm_mm` | [-5.633,-10.984] | 刚体偏移（换系基准） |
| `code_above_tip_mm` | 86.12 | 码心高于吸嘴尖 |
| `auto_grasp_z` | true | 自动算 z |
| `grasp_z_mm` | 20.0 | auto=false 时用 |
| `grasp_z_calib_mm` | -7.0 | 自动 z 的接触偏置 |
| `grasp_clearance_mm` | 0.0 | 抓取面额外间隙 |
| `approach_above_grasp_mm` | 12.0 | 近抓取闭环高度 |
| `approach_tol_mm` | 2.0 | 近抓取闭环容差 |
| `max_approach_corrections` | 2 | 近抓取修正次数 |
| `approach_z_max_corr_mm` | 15.0 | 近处重算 z 的限幅 |
| `hover_tol_mm` | 4.0 | 悬停闭环容差 |
| `max_hover_corrections` | 2 | 悬停修正次数 |
| `cart_marker_id` | 34 | 小车码 |
| `cart_verify_tol_mm` | 7.0 | 抓取位确认容差 |
| `z_retry_step_mm` | 5.0 | 失败后降 z |
| `reach_x_mm` | [60,370] | x 工作区 |
| `reach_y_mm` | [-300,300] | y 工作区 |
| `reach_radius_mm` | 300.0 | 半径限制 |
| `soft_z_mm` | [-110,210] | z 软限位 |

`grasp_trim_arm_mm` 必须为 `[0,0,0]`，非零会被忽略并告警。

---

## 5. 标定相关命令

### 内参

```bash
cd /home/hanli/ros2机械臂项目/摄像头畸变矫正

python3 c1/capture_c1.py --device /dev/video6 \
  --out cam6/photos1600 --tag c2 --target 18 \
  --width 1600 --height 1200 --fourcc MJPG

python3 ~/.agents/skills/plane-metric-calibration/scripts/chess_intrinsics.py \
  --dir cam6/photos1600 --pattern 9x6 --square-mm 25 \
  --out cam6/calib_intrinsics_1600.json
```

### 接缝棋盘 / 世界系

用 `four-camera-seam-fusion` skill（同一台电脑已安装），流程：

```bash
cd /home/hanli/ros2机械臂项目/摄像头畸变矫正
ROOT=数据/<新会话>
# 四条接缝逐条采集
python3 融合/capture_fusion.py --dir $ROOT/标定轮次/01_video6_video2 \
  --check-board 9x6 --require-devs 6,2 --note "上接缝"
# ... 02/03/04 同理
python3 融合/solve_chess.py --rounds $ROOT/标定轮次 \
  --pattern 9x6 --square-mm 25 --out $ROOT/检查/棋盘标定.json
```

### J 标定

```bash
cd /home/hanli/fyp_ws
python3 src/fyp_bringup/scripts/pick_cycle_pnp.py --calibrate-j
```

### 触摸标定

```bash
S=src/fyp_bringup/scripts/measure_touch.py
python3 $S --tag ref --frames 5
# 人工把吸嘴放到物体顶面中心后
python3 $S --tag touch --frames 5 \
  --ref-json src/fyp_bringup/runs/<时间>/ref.json
```

### 工具偏移核对

```bash
python3 <skill>/scripts/tool_offset.py \
  --config src/fyp_bringup/config/s1_pnp.json
```

### 示教标准位

```bash
python3 src/fyp_bringup/scripts/jog_arm.py --arm arm_b --save-name b_standard
```

---

## 6. 产物与日志

默认目录：

```
src/fyp_bringup/runs/<YYYYmmdd_HHMMSS>/
```

| 文件 | 内容 |
|---|---|
| `dry_base.jpg/json` | 标准位识别 |
| `r<N>_base.jpg/json` | 每轮标准位（码 22/34、物体） |
| `r<N>a<M>_hover.jpg/json` | 悬停闭环每帧 |
| `r<N>a<M>_approach.jpg/json` | 近抓取闭环每帧 |
| `r<N>a<M>_graspcheck.jpg/json` | 抓取位 5 帧中位与 `d_mm` |
| `r<N>_attempt<M>_graspcheck.json` | 含 `aligned`、`q_now`、期望值 |
| `r<N>_base_*.json` | 5 帧码高、残差 |

`graspcheck.json` 的关键字段：

```json
{
  "q_obj_cart": [..],
  "tool_offset_world_mm": [..],
  "q_now": [..],
  "q_code_expected": [..],
  "d_mm": 2.41,
  "aligned": true,
  "graspcheck_sampling": {"n": 5, "q_std": [..], "q_frames": [..]}
}
```

---

## 7. 快速判断表

| 输出 | 含义 |
|---|---|
| `相机位姿来源 chess_rounds` | 正常 |
| `RMS 0.3–1.5px` | 正常 |
| `RMS >2px` | 棋盘/内参/原图有问题 |
| `源 homography/cached` | 精度不足，修标定 |
| `scale 0.85–1.15` | J 正常 |
| `axis_skew ≤15°` | J 正常 |
| `kz 0.9–1.1` | 高度换算正常 |
| `悬停残差 ≤4mm` | 正常 |
| `近抓取残差 ≤2mm` | 正常 |
| `d_mm ≤7mm` | 允许吸 |
| `q_std ≈0.3mm` | 多帧稳定 |
| `q_std >2mm` | 码姿态跳支/检测不稳 |
