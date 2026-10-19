#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""工具偏移核对/重表达：码 22 中心相对吸嘴的刚体偏移。

模式
----
1) 核对配置（默认）:
     python3 tool_offset.py --config .../s1_pnp.json

   计算 world = J · tool_offset_arm_mm，与配置里的 tool_offset_world_mm
   比较。偏差 >0.5mm 说明配置没跟上当前 J。

2) 用当前 J 重写世界偏移:
     python3 tool_offset.py --config .../s1_pnp.json --write

   只改 motion.tool_offset_world_mm，其余字段原样保留。

3) 已知旧世界系的偏移，换到新世界系:
     python3 tool_offset.py \
       --old-world 11.40 6.22 \
       --j-old 0.02,0.98,1.01,-0.04 \
       --j-new 0.0225,0.9804,1.0092,-0.0437

   步骤: u = J_old⁻¹ · old_world; new_world = J_new · u。
   输出的 u 就是应该存进 tool_offset_arm_mm 的刚体量。

矩阵参数接受 4 个数（a,b,c,d 按行）或 2×2 JSON。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

DEFAULT_CFG = Path("/home/hanli/fyp_ws/src/fyp_bringup/config/s1_pnp.json")


def parse_matrix(s: str) -> np.ndarray:
    s = s.strip()
    if s.startswith("["):
        a = np.asarray(json.loads(s), float)
    else:
        vals = [float(x) for x in s.replace(";", ",").split(",") if x.strip()]
        if len(vals) != 4:
            raise ValueError(f"需要 4 个数，收到 {len(vals)} 个: {s!r}")
        a = np.asarray(vals, float).reshape(2, 2)
    if a.shape != (2, 2):
        raise ValueError(f"矩阵必须是 2x2，收到 {a.shape}")
    return a


def describe(J: np.ndarray, u: np.ndarray, old_world: np.ndarray) -> None:
    new_world = J @ u
    print(f"  臂坐标偏移 u          = ({u[0]:+.3f}, {u[1]:+.3f}) mm")
    print(f"  旧世界偏移            = ({old_world[0]:+.3f}, {old_world[1]:+.3f}) mm")
    print(f"  新 J 表达的世界偏移   = ({new_world[0]:+.3f}, {new_world[1]:+.3f}) mm")
    print(f"  模长                  = {np.linalg.norm(new_world):.3f} mm")
    # 角度变化只作为诊断：刚体偏移本身不随世界系改变长度
    a0 = np.degrees(np.arctan2(old_world[1], old_world[0]))
    a1 = np.degrees(np.arctan2(new_world[1], new_world[0]))
    print(f"  方向                  = {a0:+.1f}° → {a1:+.1f}°（差 {a1-a0:+.1f}°）")
    scale = np.linalg.norm(new_world) / max(np.linalg.norm(old_world), 1e-9)
    print(f"  模长比                = {scale:.4f}（应≈1，偏差大说明 J 不一致）")


def mode_reproject(args) -> int:
    old_world = np.asarray(args.old_world, float)
    J_old = parse_matrix(args.j_old)
    J_new = parse_matrix(args.j_new)
    u = np.linalg.solve(J_old, old_world)
    print("旧世界 → 臂坐标 → 新世界：")
    describe(J_new, u, old_world)
    print("\n把下面两个值写进配置 motion：")
    print(f'  "tool_offset_arm_mm":   [{u[0]:.3f}, {u[1]:.3f}],')
    w = J_new @ u
    print(f'  "tool_offset_world_mm": [{w[0]:.2f}, {w[1]:.2f}]')
    return 0


def load_cfg(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def mode_config(args) -> int:
    path = Path(args.config)
    if not path.is_file():
        print(f"❌ 配置不存在: {path}", file=sys.stderr)
        return 2
    cfg = load_cfg(path)
    m = cfg.get("motion", {})
    try:
        J = np.asarray(m["j_world"], float)
        u = np.asarray(m["tool_offset_arm_mm"], float)
    except KeyError as e:
        print(f"❌ 配置缺字段: {e}", file=sys.stderr)
        return 2
    w_calc = J @ u
    w_stored = np.asarray(m.get("tool_offset_world_mm", [0.0, 0.0]), float)
    d = float(np.linalg.norm(w_calc - w_stored))

    print(f"配置: {path}")
    print(f"  J                     = {J.round(6).tolist()}")
    print(f"  臂坐标偏移 u          = ({u[0]:+.3f}, {u[1]:+.3f}) mm")
    print(f"  当前 J 表达           = ({w_calc[0]:+.3f}, {w_calc[1]:+.3f}) mm")
    print(f"  配置存的世界偏移      = ({w_stored[0]:+.3f}, {w_stored[1]:+.3f}) mm")
    print(f"  偏差                  = {d:.3f} mm")
    print(f"  模长                  = {np.linalg.norm(w_calc):.3f} mm")

    if d <= 0.5:
        print("  ✅ 一致（偏差 ≤0.5mm）")
    else:
        print("  ❌ 不一致：配置没有跟上当前 J")
        print("     修法: 先确认 tool_offset_arm_mm 正确，再 --write；"
              "若换过吸盘/码支架，应重做触摸标定")

    if args.write:
        if d <= 0.5 and not args.force:
            print("  无需写入（--force 可强制重写）")
            return 0
        m["tool_offset_world_mm"] = [round(float(w_calc[0]), 2),
                                     round(float(w_calc[1]), 2)]
        path.write_text(json.dumps(cfg, ensure_ascii=False, indent=2) + "\n",
                        encoding="utf-8")
        print(f"  ✅ 已写回 tool_offset_world_mm = "
              f"{m['tool_offset_world_mm']}（其余字段未改）")
    return 0 if d <= 0.5 else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="核对/重表达码 22 相对吸嘴的工具偏移")
    ap.add_argument("--config", default=str(DEFAULT_CFG))
    ap.add_argument("--write", action="store_true",
                    help="把 J·tool_offset_arm_mm 写回配置")
    ap.add_argument("--force", action="store_true",
                    help="即使当前一致也强制写")
    ap.add_argument("--old-world", nargs=2, type=float, metavar=("X", "Y"),
                    help="旧世界系下的偏移，进入重表达模式")
    ap.add_argument("--j-old", help="旧 J: 4 个数 a,b,c,d 或 2x2 JSON")
    ap.add_argument("--j-new", help="新 J: 4 个数 a,b,c,d 或 2x2 JSON")
    args = ap.parse_args(argv)

    if args.old_world is not None:
        if not args.j_old or not args.j_new:
            ap.error("--old-world 需要同时给 --j-old 和 --j-new")
        return mode_reproject(args)
    return mode_config(args)


if __name__ == "__main__":
    sys.exit(main())
