#!/usr/bin/env python3
"""地图清洗：用 3x3 多数表决（中值滤波）去掉散点噪声。

**为什么需要**

用真值位姿扫描生成的地图虽然覆盖完整，但会有零星噪点
（机械臂自身、掠射角回波等）。这些孤立小点在 Nav2 里会被
``inflation_radius`` 膨胀成半米宽的"假墙"，把本来能走的路堵死，
表现为"规划不出路径 / 走到一半卡住"。

多数表决的规则很简单：一个格子只有在它 3x3 邻域里**多数**是障碍时才保留为障碍。
1 像素宽的细线和孤立点会输掉表决被删掉，而连续的墙体（≥2 像素厚）会保留。

用法::

    python3 06_脚本/09_清洗地图.py                      # 清洗 competition_map
    python3 06_脚本/09_清洗地图.py --threshold 6        # 更激进（默认 5/9）
    python3 06_脚本/09_清洗地图.py --in a.pgm --out b   # 指定文件
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_MAP = os.path.join(
    ROOT, "01_dev_ws/dev_ws/src/yzbot/bot_navigation/maps/competition_map")

OCC, FREE, UNKNOWN = 0, 254, 205


def read_pgm(path: str):
    with open(path, "rb") as fp:
        data = fp.read()
    m = re.match(rb"P5\s+(?:#.*\n)?\s*(\d+)\s+(\d+)\s+(\d+)\s", data)
    if not m:
        raise SystemExit(f"不是合法的 P5 PGM: {path}")
    w, h = int(m.group(1)), int(m.group(2))
    return w, h, bytearray(data[m.end():m.end() + w * h])


def write_pgm(path: str, w: int, h: int, pixels: bytearray) -> None:
    with open(path, "wb") as fp:
        fp.write(b"P5\n%d %d\n255\n" % (w, h))
        fp.write(bytes(pixels))


def clean(w: int, h: int, grid: bytearray, threshold: int):
    """3x3 多数表决：邻域里障碍数 >= threshold 才保留为障碍。"""
    out = bytearray(grid)
    removed = 0
    for y in range(h):
        for x in range(w):
            idx = y * w + x
            if grid[idx] != OCC:
                continue
            count = 0
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < w and 0 <= ny < h and grid[ny * w + nx] == OCC:
                        count += 1
            if count < threshold:
                out[idx] = FREE
                removed += 1
    return out, removed


def main() -> None:
    ap = argparse.ArgumentParser(description="地图散点清洗")
    ap.add_argument("--in", dest="src", default=DEFAULT_MAP + ".pgm")
    ap.add_argument("--out", dest="dst", default=DEFAULT_MAP)
    ap.add_argument("--threshold", type=int, default=5,
                    help="3x3 邻域里至少几个障碍才保留（默认 5，越大越激进）")
    ap.add_argument("--no-backup", action="store_true")
    args = ap.parse_args()

    src = args.src if args.src.endswith(".pgm") else args.src + ".pgm"
    if not os.path.isfile(src):
        raise SystemExit(f"找不到 {src}")

    w, h, grid = read_pgm(src)
    before = sum(1 for v in grid if v == OCC)
    cleaned, removed = clean(w, h, grid, args.threshold)
    after = sum(1 for v in cleaned if v == OCC)

    if not args.no_backup:
        backup = src.replace(".pgm", "_raw.pgm")
        shutil.copyfile(src, backup)
        print(f"原图已备份: {backup}")

    out_pgm = args.dst + ".pgm"
    write_pgm(out_pgm, w, h, cleaned)
    print(f"已写出: {out_pgm}")
    print(f"  障碍像素: {before} → {after}（删除 {removed} 个散点，占 {removed/max(before,1)*100:.0f}%）")
    print(f"  尺寸 {w}x{h}")

    # 同步 yaml（如果原 yaml 存在就照抄，只改 image 名）
    src_yaml = src.replace(".pgm", ".yaml")
    dst_yaml = args.dst + ".yaml"
    if os.path.isfile(src_yaml):
        text = open(src_yaml, encoding="utf-8").read()
        text = re.sub(r"^image:.*$", f"image: {os.path.basename(out_pgm)}",
                      text, flags=re.M)
        with open(dst_yaml, "w", encoding="utf-8") as fp:
            fp.write(text)
        print(f"已写出: {dst_yaml}")
    else:
        print(f"⚠ 未找到 {src_yaml}，请确认 yaml 与 pgm 文件名匹配", file=sys.stderr)


if __name__ == "__main__":
    main()
