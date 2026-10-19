# 标定手册：内参、棋盘世界系、J_world、工具偏移、抓取 z

**目录**

1. 相机内参（采集、拟合、验收）
2. 棋盘世界系与相机位姿（会话结构、重算逻辑、换相机）
3. J_world（含义、重标时机、验收、高度梯度）
4. 工具偏移（触摸标定、刚体量存储、重表达）
5. 抓取高度 z（自动公式、物理锚点、接触标定、重试）
6. 物体参数与容差
7. 标定完成检查单

本文件是 `SKILL.md` 第 2–4 节的展开。所有命令默认在
`/home/hanli/fyp_ws` 或注明的目录执行。

> **总原则：标定的产物必须是「关系」，不是「坐标」。**
> 相机内参、棋盘平面、J_world、工具偏移都是刚体量；换世界系时要么
> 重新测量，要么用刚体变换重新表达，绝不能把旧世界系的数值抄过去。

---

## 1. 相机内参（每台相机、每次改镜头/分辨率）

### 1.1 采集

四台 USB2 相机**不能同时开流**，必须逐台采集。分辨率固定
1600×1200、MJPG，与运行时 `grab_frame()` 完全一致。

```bash
cd /home/hanli/ros2机械臂项目/摄像头畸变矫正

# 设备号每次插拔都可能变，先确认
for d in /dev/video*; do
  echo -n "$d  "
  udevadm info -q property -n "$d" 2>/dev/null | grep -m1 '^ID_MODEL='
done

python3 c1/capture_c1.py \
  --device /dev/video6 \
  --out cam6/photos1600 --tag c2 \
  --target 18 --width 1600 --height 1200 --fourcc MJPG
```

拍摄要求：棋盘 9×6 内角点、25mm 格；至少 12–18 张，覆盖画面中心、
四角、远近、倾斜 ±30° 以内；避免反光和运动模糊。

### 1.2 拟合

```bash
python3 ~/.agents/skills/plane-metric-calibration/scripts/chess_intrinsics.py \
  --dir cam6/photos1600 --pattern 9x6 --square-mm 25 \
  --out cam6/calib_intrinsics_1600.json
```

验收：

- 拟合 RMS ≤ 0.6px，LOO-CV 与 RMS 接近（差值 < 0.1px）；
- fx ≈ fy（同型号相机差异 < 2%）；
- 主点贴近画面中心（偏差 < 20px）；
- 边缘径向畸变 < 4%，且四台同型号参数同号同量级。

输出 JSON 里必须有 `K`（3×3）和 `dist`（5 元素）。`pnp_vision.load_intrinsics`
直接读这两个字段。

**换相机后不要复用旧内参**，即使型号相同：镜头个体差异和装配角度
会让 fx/主点差几个像素，直接进入毫米级误差。

---

## 2. 棋盘世界系与相机位姿

### 2.1 会话目录结构

`PnpVision` 由 `camera.calib_file` 反推整个标定会话：

```
<session>/
  标定轮次/
    <round>/video6_1600x1200.png     # 文件名必须是 video<dev>_1600x1200.png
    ...
  检查/
    棋盘标定.json                     # calib_file 指向这里
```

会话根 = `calib_file.parent.parent`。**移动/改名 JSON 会让原图找不到，
相机位姿悄悄退回单应兜底。**

### 2.2 位姿是怎么算的

`camera_pose_from_rounds()`：

1. 遍历 `棋盘标定.json` 的 `boards`；
2. 只取 `seen_by` 包含本相机 label（如 `c2`）的轮次；
3. 读 `标定轮次/<round>/video<dev>_1600x1200.png`，重新检测 54 个角点；
4. 用该轮的 `T`（棋盘 mm → 世界 mm）把角点变到世界平面；
5. 所有轮次的点一起 `solvePnP`，得到世界→相机位姿，并报 RMS。

因此：

- **S1 相机必须至少在一轮棋盘里被看到**，且该轮要写进
  `boards[round].seen_by`、有对应原图；
- 多轮、棋盘铺满工作区，会显著降低边缘外推误差；
- 世界系由参考棋盘的 `T` 定义（`T=I` 的那块）。参考棋盘动了，
  世界系就动了，J 必须重标；
- `refresh_pose: true` 时每次运行都从原图重算，相机被碰一下也能反映；
  但仍要求原图存在。

### 2.3 换相机/换角度的标准动作

用 `four-camera-seam-fusion` skill 重拍四条接缝并解算，输出新的
`<session>/检查/棋盘标定.json`，然后改 `s1_pnp.json`：

```json
"camera": {
  "dev": 6,
  "label": "c2",
  "intrinsics_file": ".../cam6/calib_intrinsics_1600.json",
  "calib_file": ".../数据/<新会话>/检查/棋盘标定.json",
  "refresh_pose": true
}
```

改完先跑 `--dry-run`，确认打印：

```
相机位姿来源 chess_rounds  RMS 0.xx px  高度 xxxx.x mm
```

- `chess_rounds` + RMS 0.3–1.5px：正常；
- `homography`：没找到原图，精度较低，修 `标定轮次` 路径；
- `cached`：配置里缓存了旧位姿，删掉 `camera.pose` 或修标定。

---

## 3. J_world：臂坐标 → 世界坐标

### 3.1 它是什么

`J_world` 是 2×2 矩阵，把**臂坐标的水平增量 mm** 映射到**世界系水平
增量 mm**：

```
world_delta ≈ J · arm_delta
arm_delta   ≈ J⁻¹ · world_delta
```

它不是「像素到毫米」，也不是相机外参。它吸收：臂底座相对桌面的旋转、
两轴尺度差异、两轴不垂直。`det<0` 正常（世界 Z 朝下，臂 Z 朝上）。

### 3.2 什么时候重标

- 世界系变了（重拍棋盘、换参考棋盘）；
- 臂底座挪动、换臂、重新回零后零点变化；
- `--dry-run` 算出的目标系统性偏一个方向且闭环也拉不回。

**不要每次抓取都标**：探针会在标准位附近 ±20/±30mm 移动，可能碰走
小车，也拖慢流程。

### 3.3 命令与验收

```bash
cd /home/hanli/fyp_ws
python3 src/fyp_bringup/scripts/pick_cycle_pnp.py --calibrate-j
```

脚本会：回零 → 到标准位 → 在 base、x±20、y±20、z±30 各停一次，
每处 8 帧取中位 → 拟合 J、`kz`、`height_grad_arm` → 写回
`s1_pnp.json` 的 `motion.j_world` 与 `motion.j_meta`。

验收（`j_meta`）：

| 量 | 合格范围 | 本机实测 |
|---|---|---|
| `scale` | 0.85–1.15 | 0.995 |
| `axis_skew_deg` | ≤15°（目标 90°） | 91.28° |
| `kz` | 0.9–1.1 | 1.0166 |
| `det` | 非零（负值正常） | −0.9904 |

超范围先查：码 22 是否倾斜/遮挡、相机位姿 RMS、探针期间小车是否被碰、
标准位是否真的到达。

### 3.4 高度梯度

`j_meta.height_grad_arm = [gx, gy, kz]` 表示「臂 x/y/z 各走 1mm，码高
变化多少」。**抓取 z 只用 `kz`**：

- `gx, gy` 主要来自 30mm 小码的深度测量偏差，不是真实几何；
- 把 `gx, gy` 计入会把抓取高度带偏约 20mm（2026-10-19 实测）；
- 代码里 `compute_grasp_z()` 显式只用 `kz`。

---

## 4. 工具偏移：码 22 中心 ≠ 吸嘴

### 4.1 概念

吸盘码 22 贴在吸盘支架上，码中心与吸嘴尖不重合。抓取时相机只能看到
码，所以：

```
期望码心 = 物体目标 + tool_offset_world_mm
实际吸嘴 = 码心 − tool_offset_world_mm
```

本机触摸标定：码心相对吸嘴约 `(+11.4, +6.2)mm`（旧世界系），模长约
12mm；码心比吸嘴尖高 `code_above_tip_mm = 86.12mm`。

### 4.2 触摸标定步骤

`measure_touch.py` 只拍照、不动臂，需要人工把吸嘴放到物体顶面中心：

```bash
cd /home/hanli/fyp_ws
S=src/fyp_bringup/scripts/measure_touch.py

# 1) 臂不挡物体时拍参考（标准位通常可以）
python3 $S --tag ref --frames 5

# 2) 关吸盘，人工 jog 使吸嘴轻放在物体顶面中心，停稳
python3 $S --tag touch --frames 5 --ref-json src/fyp_bringup/runs/<时间>/ref.json
```

输出：

- `offset_world_xy = P22 − P_obj`（码心相对吸嘴，世界 mm）；
- `code_above_tip = H22 − object.height_mm`。

### 4.3 存成刚体量（关键）

世界系会变，但工具相对臂的几何不变。**存臂坐标**：

```python
tool_offset_arm_mm = inv(J_old) @ offset_world_old
```

配置里同时存：

```json
"tool_offset_arm_mm":  [-5.633, -10.984],
"tool_offset_world_mm": [-10.90, -5.20]
```

换 J 后重新表达：

```python
tool_offset_world_new = J_new @ tool_offset_arm_mm
```

本机的来历：旧世界 `(11.40, 6.22)` → `J_old⁻¹` → 臂坐标
`(-5.633, -10.984)` → 新 J → 新世界 `(-10.90, -5.20)`。

检查脚本：

```bash
python3 <skill>/scripts/tool_offset.py \
  --config src/fyp_bringup/config/s1_pnp.json
```

它会打印臂坐标值、当前 J 表达值、配置存的世界值，并给出偏差。
偏差 > 0.5mm 就说明配置没跟上 J。

### 4.4 换吸盘/换码支架后

必须重做 4.2 的触摸标定，同时更新：

- `motion.tool_offset_world_mm`
- `motion.tool_offset_arm_mm`
- `motion.code_above_tip_mm`

不要用「上次抓取偏了几毫米」去猜偏移——那是把位姿相关误差当成常数。

---

## 5. 抓取高度 z 的确定

### 5.1 自动公式

```python
tip_h  = H_code - code_above_tip_mm          # 吸嘴尖当前离桌面高度
desired = object.height_mm + grasp_clearance_mm
dz_arm = (desired - tip_h) / kz              # 只用 kz
gz     = base_z + dz_arm + grasp_z_calib_mm  # calib 仅对自动 z 生效
```

其中 `H_code` 取同一姿态 5 帧中位（`suction_height_median`）。

### 5.2 什么时候自动 z 不可信

- 码 22 明显倾斜：平面 pose 有歧义，码高会偏高，倾角越大偏得越多
  （本机新相机下曾偏高约 50mm，自动 z 算出 −32.8mm）；
- 重投影 RMS > 1.5px；
- 抓取高度落在软限位边缘（`soft_z_mm = [-110, 210]`）。

此时**先做物理锚定**，不要硬跑：

1. 在标准位 `--dry-run`，记下吸嘴尖推算高度；
2. 用一次低风险试跑来确认：把 `--grasp-z` 设为一个保守值（离物体顶
   还有几毫米），观察吸嘴与物体的间隙；
3. 按「间隙 + 当前 z」推正确 z，例如实测 z=32.3 时还差约 7mm →
   正确值约 25.3；
4. 用 `--grasp-z 25.3` 跑正式抓取；验证成功后再决定是否把偏置写进
   `grasp_z_calib_mm` 或重做触摸标定。

### 5.3 接触标定 `grasp_z_calib_mm`

本机为 −7.0：近抓取高度重算 z 时会因码高偏置抬高约 6–7mm，所以自动 z
统一减 7mm。**它只对自动 z 生效；显式 `--grasp-z` 不叠加。**

换相机/重做触摸标定后，这个值应重新实测，不要长期沿用。

### 5.4 重试降 z

每次抓取尝试失败后，`z_retry_step_mm=5`：第 2 次尝试比第 1 次低 5mm。
最多 `max_attempts=2` 次（可通过参数加大），且每次重试都会加水平
螺旋偏移并重新闭环、重新确认。仍失败就停，不无限试。

---

## 6. 物体参数与容差

```json
"object": {
  "color": "auto",
  "height_mm": 105.0,
  "size_mm": 25.0,
  "min_area_px": 150.0,
  "min_fill": 0.55,
  "max_side_err_ratio": 0.35,
  "max_distance_from_suction_mm": 320.0
}
```

- `height_mm` 同时用于抓取 z 与物体世界坐标反投影，换物体必须改；
- `size_mm` 是正方形边长，用于色块形状筛选；
- `color: auto` 会在候选颜色里选最像正方形的一个；背景有同色物体时
  显式指定 `--color green/red/...`；
- 抓取确认容差按物理推导：

```
tol ≈ 物体半宽 − 吸盘半径
25mm 物体、半径约 5mm 的吸盘 → 12.5 − 5 ≈ 7mm
```

换 15mm 物体时约 2.5mm，此时应同时提高标定精度、减小闭环容差，
否则会频繁误拒或误吸。

---

## 7. 标定完成检查单

- [ ] 内参 RMS ≤0.6px，`K/dist` 已写入 `camN/calib_intrinsics_1600.json`
- [ ] 新会话 `标定轮次/` 原图齐全，S1 相机出现在 `seen_by`
- [ ] `calib_file` 路径正确，`refresh_pose=true`
- [ ] `--dry-run` 位姿来源 `chess_rounds`，RMS 合理
- [ ] J 尺度/轴夹角/kz 在范围内，`j_meta.taught` 是本次时间
- [ ] `tool_offset_arm_mm` 已用新 J 重表达，脚本核对偏差 <0.5mm
- [ ] `code_above_tip_mm` 与当前吸盘/码支架一致
- [ ] 自动 z 或 `--grasp-z` 落在 15–35mm 的合理区间
- [ ] 标准位能同时看到码 22、码 34、物体
