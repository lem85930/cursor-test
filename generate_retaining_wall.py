#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Retaining wall cross-section generator.

- Supports four types: 仰斜式、直立式、俯斜式、衡重式
- Inputs parameters: H, b, m1, bj, hj, hn, Bd, n, m2, bt, h1, h2
- Produces a 2D closed polyline at 1:1 scale in DXF format, and
  will attempt to convert to DWF if a supported converter is detected.

Notes on geometry conventions used here
- Coordinate system: X to the right, Y upwards.
- Origin (0, 0) is at the front toe bottom-left corner.
- Bottom underside is a straight line from (0, 0) to (Bd_adj, hn).
- "1:m" for faces is interpreted as vertical:horizontal; when moving up by Δy,
  X shifts to the left by m*Δy (leaning towards negative X).
- Top width b connects the free face top-left to the back face top-right.
- For 衡重式, lower back slope is fixed to 1:0.25, the upper back slope is 1:m2,
  with an intermediate platform width bt, heights h2 (lower) and h1 (upper),
  where H ≈ h1 + h2. If not, the script adjusts h2 to H - h1.

Important
- DWF is a proprietary format. This script always writes DXF.
  If an external converter is found (environment variable DWF_CONVERTER_CMD, or
  commands known to the script), it will be called to convert the DXF to DWF.
  Otherwise, you will get only the DXF output.

Run
- Interactive: python generate_retaining_wall.py
- Non-interactive: provide CLI args, e.g.
  python generate_retaining_wall.py --type 直立式 --H 6 --b 0.8 --m1 0.3 \
    --bj 0.4 --hj 0.3 --hn 0.2 --Bd 3.5 --n 8 --m2 0.2 --bt 0.5 --h1 3 --h2 3 \
    --out /workspace/output
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from typing import List, Sequence, Tuple

try:
    import ezdxf  # type: ignore
except Exception as exc:  # pragma: no cover - dependency error is fatal at runtime
    print("[ERROR] Missing dependency 'ezdxf'. Install with: pip install ezdxf", file=sys.stderr)
    raise


Point = Tuple[float, float]


@dataclass
class WallParams:
    H: float
    b: float
    m1: float
    bj: float
    hj: float
    hn: float
    Bd: float
    n: float
    m2: float
    bt: float
    h1: float
    h2: float


WALL_TYPES = ["仰斜式", "直立式", "俯斜式", "衡重式"]

# Back-face slopes for non-gravity walls (1:mb)
NON_GRAVITY_BACK_SLOPE = {
    "仰斜式": 0.25,
    "直立式": 0.0,
    "俯斜式": 0.05,
}

LOWER_BACK_SLOPE_HENGZHONG = 0.25  # fixed 1:0.25 for lower back in gravity type


def _safe_float(value: str, default: float = 0.0) -> float:
    try:
        return float(str(value).strip())
    except Exception:
        return default


def prompt_interactive() -> Tuple[str, WallParams, str]:
    print("请选择挡墙类型：仰斜式 / 直立式 / 俯斜式 / 衡重式")
    wall_type = input("类型: ").strip()
    if wall_type not in WALL_TYPES:
        print(f"未识别的类型，默认使用 直立式")
        wall_type = "直立式"

    # Unit note
    print("注意：所有输入均使用同一长度单位（例如 m 或 mm），脚本按 1:1 输出，不做单位换算。")

    def ask(name: str) -> float:
        return _safe_float(input(f"请输入 {name}: "))

    H = ask("H 挡墙总高")
    b = ask("b 墙顶宽度")
    m1 = ask("m1 临空面斜率(1:m1)")
    bj = ask("bj 墙趾顶宽")
    hj = ask("hj 墙趾厚度")
    hn = ask("hn 墙踵底与墙趾底高差")
    Bd = ask("Bd 墙底总宽")
    n = ask("n 墙底底面斜率(n:1)中的 n")
    m2 = ask("m2 衡重式上阶墙背斜率(1:m2)，非衡重式可填0")
    bt = ask("bt 衡重式上阶平台宽度，非衡重式可填0")
    h1 = ask("h1 衡重式上阶高度，非衡重式可填0")
    h2 = ask("h2 衡重式下阶高度，非衡重式可填0")

    out_dir = input("输出目录(回车默认 /workspace/output): ").strip() or "/workspace/output"

    return wall_type, WallParams(H, b, m1, bj, hj, hn, Bd, n, m2, bt, h1, h2), out_dir


def build_outline_points(wall_type: str, p: WallParams) -> Tuple[List[Point], float]:
    """Return (points, Bd_adjusted).

    The polygon is ordered counter-clockwise starting at the toe bottom-left (0,0).
    """
    # Vertical top elevation relative to origin (toe bottom)
    y_top = p.hn + p.H

    # Free face top-left coordinate determined by slope from (bj, hj)
    x_top_left = p.bj - p.m1 * (y_top - p.hj)
    x_top_right = x_top_left + p.b

    if wall_type == "衡重式":
        # Harmonize H with h1 + h2 if needed
        if abs((p.h1 + p.h2) - p.H) > 1e-6:
            # Prefer keeping h1, adjust h2
            p.h2 = max(0.0, p.H - p.h1)
        # Compute required heel bottom X so that back-face segments close
        x_after_upper = x_top_right + p.m2 * p.h1  # descending along upper slope
        x_after_platform = x_after_upper + p.bt
        bd_required = x_after_platform + LOWER_BACK_SLOPE_HENGZHONG * p.h2
        Bd_adj = bd_required

        # Bottom and back-face points
        a = (0.0, 0.0)
        btm = (Bd_adj, p.hn)
        lower_top = (Bd_adj - LOWER_BACK_SLOPE_HENGZHONG * p.h2, p.hn + p.h2)
        step_inner = (lower_top[0] - p.bt, lower_top[1])
        back_top = (x_top_right, y_top)
        free_top_left = (x_top_left, y_top)
        toe_top_right = (p.bj, p.hj)
        toe_top_left = (0.0, p.hj)

        points: List[Point] = [a, btm, lower_top, step_inner, back_top, free_top_left, toe_top_right, toe_top_left]
        return points, Bd_adj

    # Non-gravity types: determine required Bd from back slope and top-right
    mb = NON_GRAVITY_BACK_SLOPE.get(wall_type, 0.0)
    Bd_adj = x_top_right + mb * p.H

    a = (0.0, 0.0)
    btm = (Bd_adj, p.hn)
    back_top = (x_top_right, y_top)
    free_top_left = (x_top_left, y_top)
    toe_top_right = (p.bj, p.hj)
    toe_top_left = (0.0, p.hj)

    points = [a, btm, back_top, free_top_left, toe_top_right, toe_top_left]
    return points, Bd_adj


def write_dxf(points: Sequence[Point], out_path: str) -> None:
    doc = ezdxf.new(setup=True)
    # Units: draw in raw units; 1:1 by construction. Optionally set INSUNITS to meters.
    # Not setting units avoids unintended scaling by downstream tools.
    msp = doc.modelspace()

    # Add polyline
    msp.add_lwpolyline(points, dxfattribs={"closed": True})

    # Add a layer and axis for reference (optional)
    try:
        doc.layers.new(name="_AXIS", dxfattribs={"color": 8})
        msp.add_line((0, 0), (max(p[0] for p in points) * 1.05, 0), dxfattribs={"layer": "_AXIS"})
        msp.add_line((0, 0), (0, max(p[1] for p in points) * 1.05), dxfattribs={"layer": "_AXIS"})
    except Exception:
        pass

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    doc.saveas(out_path)


def find_dwf_converter_cmd() -> str | None:
    """Try to locate an external DWF converter.

    Recognized options (return the executable path if found):
    - Env var DWF_CONVERTER_CMD explicitly points to a command.
    - Known commands in PATH (best-effort): dwfconvert, dwf_publish, odapublish, odadwf, acad_dwf_export
    """
    env_cmd = os.environ.get("DWF_CONVERTER_CMD")
    if env_cmd:
        return env_cmd

    probes = [
        "dwfconvert",
        "dwf_publish",
        "odapublish",
        "odadwf",
        "acad_dwf_export",
    ]
    for name in probes:
        path = shutil.which(name)
        if path:
            return path
    return None


def try_convert_to_dwf(dxf_path: str, dwf_path: str) -> bool:
    """Attempt to convert DXF to DWF using an external tool if available.

    Returns True on success, False otherwise.
    """
    cmd = find_dwf_converter_cmd()
    if not cmd:
        return False

    # Heuristic CLI invocation; real tools differ. We try a couple of common patterns.
    attempts = [
        [cmd, dxf_path, dwf_path],
        [cmd, "--input", dxf_path, "--output", dwf_path],
        [cmd, "-i", dxf_path, "-o", dwf_path],
    ]
    for argv in attempts:
        try:
            subprocess.run(argv, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            return os.path.exists(dwf_path) and os.path.getsize(dwf_path) > 0
        except Exception:
            continue
    return False


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate retaining wall cross-section (DXF/DWF)")
    parser.add_argument("--type", dest="wall_type", choices=WALL_TYPES, help="挡墙类型")
    parser.add_argument("--H", type=float, help="挡墙总高")
    parser.add_argument("--b", type=float, help="墙顶宽度")
    parser.add_argument("--m1", type=float, help="临空面斜率(1:m1)")
    parser.add_argument("--bj", type=float, help="墙趾顶宽")
    parser.add_argument("--hj", type=float, help="墙趾厚度")
    parser.add_argument("--hn", type=float, help="踵底与趾底高差")
    parser.add_argument("--Bd", type=float, help="墙底总宽")
    parser.add_argument("--n", type=float, default=0.0, help="底面斜率 n:1 中的 n（仅用于记录）")
    parser.add_argument("--m2", type=float, default=0.0, help="衡重式上阶斜率(1:m2)")
    parser.add_argument("--bt", type=float, default=0.0, help="衡重式上阶平台宽度")
    parser.add_argument("--h1", type=float, default=0.0, help="衡重式上阶高度")
    parser.add_argument("--h2", type=float, default=0.0, help="衡重式下阶高度")
    parser.add_argument("--out", dest="out_dir", default="/workspace/output", help="输出目录")
    parser.add_argument("--basename", default="retaining_wall", help="输出文件基础名")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    if argv is None:
        argv = sys.argv[1:]

    if argv:
        ns = parse_args(argv)
        if not all(
            getattr(ns, k) is not None
            for k in ["wall_type", "H", "b", "m1", "bj", "hj", "hn", "Bd"]
        ):
            print("参数不全，改用交互式输入。\n")
            wall_type, params, out_dir = prompt_interactive()
        else:
            wall_type = ns.wall_type
            params = WallParams(
                ns.H,
                ns.b,
                ns.m1,
                ns.bj,
                ns.hj,
                ns.hn,
                ns.Bd,
                ns.n,
                ns.m2,
                ns.bt,
                ns.h1,
                ns.h2,
            )
            out_dir = ns.out_dir
            basename = ns.basename
    else:
        wall_type, params, out_dir = prompt_interactive()
        basename = "retaining_wall"

    points, Bd_adj = build_outline_points(wall_type, params)

    # Inform if Bd was adjusted for geometric closure
    if abs(Bd_adj - params.Bd) > 1e-6:
        print(f"[提示] 为保证几何闭合，已按输入的斜率与尺寸调整 Bd: {params.Bd:.6g} -> {Bd_adj:.6g}")

    os.makedirs(out_dir, exist_ok=True)
    dxf_path = os.path.join(out_dir, f"{basename}_{wall_type}.dxf")
    write_dxf(points, dxf_path)
    print(f"DXF 已生成: {dxf_path}")

    # Try to produce DWF
    dwf_path = os.path.join(out_dir, f"{basename}_{wall_type}.dwf")
    if try_convert_to_dwf(dxf_path, dwf_path):
        print(f"DWF 已生成: {dwf_path}")
    else:
        print(
            "[注意] 未检测到可用的 DWF 转换器。已输出 DXF。\n"
            "您可以在安装了 AutoCAD/ODA Publish 的环境中将 DXF 发布为 DWF，"
            "或设置环境变量 DWF_CONVERTER_CMD 指向可用的命令行转换器。"
        )

    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())