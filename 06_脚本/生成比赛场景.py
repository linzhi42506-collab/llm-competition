#!/usr/bin/env python3
"""比赛场景生成器 —— 一键生成满足「场景设计分」要求的 competition.world

评分项 7「场景设计」（10 分）要求：
  ① 货物在场景中不同位置，彼此距离不小于 2 m；
  ② 放置区在场景中不同位置，至少间隔一面墙；
  ③ 存在至少两个有效障碍物（位于机器人前往货物/放置区的路线附近）。

规则同时明确：**场景中现有元素不能删减，但目标货物位置、放置区域位置可以自行
设计，并且可以增加元素**。因此本脚本的做法是：

  * 以官方 room.world 为底稿（所有原有元素原样保留）；
  * 按 ``scene_layout.yaml`` 重新摆放货物与放置区；
  * 给每个货物加一个支撑台（保证货物 z=0.75 有支撑，不会掉到地上）；
  * 在放置区之间**增加**隔墙（满足"至少隔一面墙"）；
  * **增加**两个可移动障碍物模型（供 dynamic_obstacles 节点驱动）。

输出：mybot_description/worlds/competition.world （不覆盖任何原始文件）

用法::

    python3 06_脚本/生成比赛场景.py                    # 用默认布局
    python3 06_脚本/生成比赛场景.py --layout my.yaml    # 用自定义布局
    python3 06_脚本/生成比赛场景.py --check            # 只做评分项校验
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
from typing import Dict, List, Tuple

try:
    import yaml
except ImportError:
    print("需要 pyyaml: pip3 install pyyaml", file=sys.stderr)
    raise

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_WORLD = os.path.join(
    ROOT, "01_dev_ws/dev_ws/src/yzbot/mybot_description/worlds/room.world")
DST_WORLD = os.path.join(
    ROOT, "01_dev_ws/dev_ws/src/yzbot/mybot_description/worlds/competition.world")
LAYOUT_FILE = os.path.join(ROOT, "01_dev_ws/dev_ws/src/llm_comp/config/scene_layout.yaml")

# --------------------------------------------------------------------------- #
#  默认布局：2.2 m 网格，保证任意两个货物间距 >= 2 m
#  ⚠ 这些坐标必须对着 Gazebo 实际场景目视确认一次（可能撞到原有家具/墙体），
#    确认后直接改本文件或 --layout 指定的 YAML 即可。
# --------------------------------------------------------------------------- #
DEFAULT_LAYOUT: Dict = {
    "grid_step": 2.2,
    "support_height": 0.75,          # 货物放置高度 = 支撑台高度
    "support_size": [0.16, 0.16],    # 细支柱：足迹小，机器人可以停得很近又不会卡住
    "cubes": {
        "red_cube_1":  [-4.4, 2.2],
        "red_cube_2":  [-2.2, 2.2],
        "red_cube_3":  [0.0, 2.2],
        "red_cube_4":  [2.2, 2.2],
        "red_cube_5":  [4.4, 2.2],
        "blue_cube_1": [-4.4, -2.2],
        "blue_cube_2": [-2.2, -2.2],
        "blue_cube_3": [0.0, -2.2],
        "blue_cube_4": [2.2, -2.2],
        "blue_cube_5": [4.4, -2.2],
    },
    "zones": {
        # 三个区域分处三块由隔墙分开的位置（x 方向被墙隔断）
        "A": [-2.2, -4.6],
        "B": [0.0, -4.6],
        "C": [2.2, -4.6],
    },
    # ⚠ 隔墙是"场景设计分"第②条（放置区至少隔一面墙）用的，但它同时是
    #   导航最大的障碍源——实测加上去以后通道变窄、机器人容易卡住。
    #   官方讲解明确说"三点满足两点即得 10 分"，所以默认**不加墙**：
    #   保留"货物间距≥2m"+"2个移动障碍物"两条即满足 10 分要求。
    #   想加回来把关掉下面这行注释即可（或改 --walls）。
    "zone_walls": {},
    # ⚠ 障碍物轨迹的三条硬性约束（踩过坑，务必遵守）:
    #   1) 两条轨迹**不能相交**，否则两个障碍物会撞在一起；
    #   2) 轨迹**不能穿过机器人出生点 (0,0)**，否则开机瞬间就把车撞飞；
    #   3) 轨迹**不能经过机器人停车作业点**（货物前方 0.55m 处），否则抓取时被撞。
    #   下面两条是平行的水平巡逻线（y=±1.0），互相平行不相交，
    #   离出生点 1.0m，且在"货物行(y=±2.2) ↔ 放置区(y=-4.6)"的必经之路上。
    "obstacles": {
        # 尺寸刻意做小做矮：太高/太大会把机器人顶翻（实测踩过坑）
        "obstacle_1": {"size": [0.24, 0.24, 0.45], "color": [0.90, 0.50, 0.10, 1.0],
                       "waypoints": [[-2.6, 1.0], [2.6, 1.0]]},
        "obstacle_2": {"size": [0.24, 0.24, 0.45], "color": [0.60, 0.20, 0.80, 1.0],
                       "waypoints": [[-2.6, -1.0], [2.6, -1.0]]},
    },
    "spawn_robot": [0.0, 0.0, 0.2],
}


# --------------------------------------------------------------------------- #
def load_layout(path: str | None) -> Dict:
    if path and os.path.isfile(path):
        with open(path, "r", encoding="utf-8") as fp:
            data = yaml.safe_load(fp) or {}
        layout = {**DEFAULT_LAYOUT, **data}
        print(f"[场景] 使用布局文件 {path}")
        return layout
    if os.path.isfile(LAYOUT_FILE):
        with open(LAYOUT_FILE, "r", encoding="utf-8") as fp:
            data = yaml.safe_load(fp) or {}
        layout = {**DEFAULT_LAYOUT, **data}
        print(f"[场景] 使用布局文件 {LAYOUT_FILE}")
        return layout
    print("[场景] 使用内置默认布局")
    return DEFAULT_LAYOUT


def set_pose(world: str, model: str, x: float, y: float, z: float, yaw: float = 0.0) -> str:
    """替换某个 model 的 <pose>。"""
    pattern = re.compile(r"(<model name='%s'>\s*<pose>)([^<]*)(</pose>)" % re.escape(model))
    if not pattern.search(world):
        # 该模型没有 <pose> 或名字不匹配
        pattern = re.compile(r"(<model name='%s'>)" % re.escape(model))

        def _insert(match: re.Match) -> str:
            return f"{match.group(1)}\n      <pose>{x} {y} {z} 0 0 {yaw}</pose>"

        world, count = pattern.subn(_insert, world, count=1)
        if count == 0:
            print(f"  ! 未找到模型 {model}，跳过")
        return world
    return pattern.sub(
        lambda m: f"{m.group(1)}{x} {y} {z} 0 0 {yaw}{m.group(3)}", world, count=1)


def make_support(name: str, x: float, y: float, layout: Dict) -> str:
    """货物支撑台（一个 0.75 m 高的方柱），保证货物不会掉到地上。"""
    sx, sy = layout["support_size"]
    h = layout["support_height"]
    return f"""
    <model name='support_{name}'>
      <static>true</static>
      <pose>{x} {y} {h / 2.0} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>{sx} {sy} {h}</size></box></geometry>
        </collision>
        <visual name='visual'>
          <geometry><box><size>{sx} {sy} {h}</size></box></geometry>
          <material><ambient>0.35 0.35 0.4 1</ambient><diffuse>0.35 0.35 0.4 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def make_wall(name: str, start: List[float], end: List[float], height: float = 1.6, thick: float = 0.12) -> str:
    """两个放置区之间的隔墙。"""
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = (dx * dx + dy * dy) ** 0.5
    cx, cy = (start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0
    yaw = 0.0
    return f"""
    <model name='{name}'>
      <static>true</static>
      <pose>{cx} {cy} {height / 2.0} 0 0 {yaw}</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><box><size>{length} {thick} {height}</size></box></geometry>
        </collision>
        <visual name='visual'>
          <geometry><box><size>{length} {thick} {height}</size></box></geometry>
          <material><ambient>0.8 0.8 0.75 1</ambient><diffuse>0.8 0.8 0.75 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def make_obstacle(name: str, spec: Dict) -> str:
    """可移动障碍物：非 static，供 dynamic_obstacles 节点用 set_entity_state 驱动。"""
    sx, sy, sz = spec.get("size", [0.4, 0.4, 0.7])
    r, g, b, a = spec.get("color", [0.9, 0.5, 0.1, 1.0])
    x, y = spec.get("waypoints", [[0, 0], [0, 0]])[0]
    return f"""
    <model name='{name}'>
      <pose>{x} {y} {sz / 2.0 + 0.01} 0 0 0</pose>
      <link name='link'>
        <!-- kinematic: 障碍物只按指令移动，不会被物理推挤，
             配合高频率小步长移动，避免"瞬移"给机器人巨大冲击把它顶翻 -->
        <kinematic>true</kinematic>
        <inertial><mass>1.0</mass></inertial>
        <collision name='collision'>
          <geometry><cylinder><radius>{sx / 2.0}</radius><length>{sz}</length></cylinder></geometry>
        </collision>
        <visual name='visual'>
          <geometry><cylinder><radius>{sx / 2.0}</radius><length>{sz}</length></cylinder></geometry>
          <material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse></material>
        </visual>
      </link>
    </model>
"""


ROS_STATE_PLUGIN = """
    <!-- gazebo_ros_state: 提供 /gazebo/model_states 与 /gazebo/set_entity_state。
         视觉识别真值模式、坐标记录工具、动态障碍物节点都依赖它。
         注意: 它是 world 插件，必须写在世界文件里，不能用 gzserver -s 加载。 -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <!-- 不要加 <ros><namespace>/</namespace></ros>，否则话题会变成 /model_states，
           默认（不带 ros 标签）才是 /gazebo/model_states -->
      <update_rate>30.0</update_rate>
    </plugin>
"""


def generate(layout: Dict) -> str:
    if not os.path.isfile(SRC_WORLD):
        raise SystemExit(f"找不到原始场景文件: {SRC_WORLD}")
    with open(SRC_WORLD, "r", encoding="utf-8") as fp:
        world = fp.read()

    print("[场景] 重新摆放货物 ...")
    supports = []
    for name, (x, y) in layout["cubes"].items():
        world = set_pose(world, name, x, y, layout["support_height"])
        supports.append(make_support(name, x, y, layout))
        print(f"  {name:12s} → ({x:.2f}, {y:.2f})")

    print("[场景] 重新摆放放置区 ...")
    for zone, (x, y) in layout["zones"].items():
        world = set_pose(world, f"zone_{zone.lower()}", x, y + 0.0, 0.01)
        print(f"  {zone} 区        → ({x:.2f}, {y:.2f})")

    print("[场景] 增加放置区隔墙 ...")
    walls = [make_wall(name, seg[0], seg[1]) for name, seg in (layout.get("zone_walls") or {}).items()]

    print("[场景] 增加可移动障碍物 ...")
    obstacles = [make_obstacle(name, spec) for name, spec in (layout.get("obstacles") or {}).items()]

    extra = ROS_STATE_PLUGIN + "".join(supports + walls + obstacles)
    world = world.replace("</world>", extra + "\n  </world>")

    os.makedirs(os.path.dirname(DST_WORLD), exist_ok=True)
    with open(DST_WORLD, "w", encoding="utf-8") as fp:
        fp.write(world)
    print(f"\n[场景] 已生成: {DST_WORLD}")
    return DST_WORLD


# --------------------------------------------------------------------------- #
def check(layout: Dict) -> bool:
    """校验场景设计三项要求。"""
    ok = True
    print("=" * 64)
    print("场景设计分（10 分）自查")
    print("=" * 64)

    # ① 货物间距 >= 2 m
    print("\n① 货物两两距离 >= 2 m")
    names = list(layout["cubes"].keys())
    bad = []
    min_d = 1e9
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            ax, ay = layout["cubes"][names[i]]
            bx, by = layout["cubes"][names[j]]
            d = ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5
            min_d = min(min_d, d)
            if d < 2.0:
                bad.append((names[i], names[j], d))
    if bad:
        ok = False
        for a, b, d in bad[:10]:
            print(f"   ✗ {a} ↔ {b} = {d:.2f} m (< 2 m)")
    else:
        print(f"   ✓ 全部满足，最小间距 {min_d:.2f} m")

    # ② 放置区至少隔一面墙
    print("\n② 放置区至少隔一面墙")
    walls = layout.get("zone_walls") or {}
    if not walls:
        ok = False
        print("   ✗ 未定义任何隔墙（zone_walls）")
    else:
        # 判断：每个区域对之间，连线是否被某面墙的线段相交
        zones = layout["zones"]
        zn = list(zones.keys())
        for i in range(len(zn)):
            for j in range(i + 1, len(zn)):
                a, b = zones[zn[i]], zones[zn[j]]
                sep = any(_segments_intersect(a, b, seg[0], seg[1]) for seg in walls.values())
                mark = "✓" if sep else "✗"
                if not sep:
                    ok = False
                print(f"   {mark} {zn[i]} 区 ↔ {zn[j]} 区  {'被墙隔开' if sep else '没有墙隔开'}")
        print(f"   隔墙数量: {len(walls)}")

    # ③ 至少两个有效障碍物
    print("\n③ 至少 2 个有效障碍物（位于任务路线上）")
    obstacles = layout.get("obstacles") or {}
    if len(obstacles) < 2:
        ok = False
        print(f"   ✗ 只有 {len(obstacles)} 个障碍物")
    else:
        print(f"   ✓ 共 {len(obstacles)} 个")
        for name, spec in obstacles.items():
            wps = spec.get("waypoints", [])
            print(f"     {name}: " + " ↔ ".join(f"({w[0]:.1f},{w[1]:.1f})" for w in wps))
        # --- 安全性自检：轨迹互不相交、不压出生点、不压停车作业点 ---
        spawn = tuple(layout.get("spawn_robot", [0.0, 0.0, 0.2])[:2])
        approach = 0.55
        park_pts = []
        for name, (cx, cy) in layout["cubes"].items():
            yaw = 1.5708 if cy > 0 else -1.5708
            park_pts.append((name, cx - approach * math.cos(yaw), cy - approach * math.sin(yaw)))
        for zn, (zx, zy) in layout["zones"].items():
            park_pts.append((f"{zn}区", zx, zy + approach))

        segs = {n: (tuple(sp["waypoints"][0]), tuple(sp["waypoints"][1]))
                for n, sp in obstacles.items() if len(sp.get("waypoints", [])) >= 2}
        names = list(segs)
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                if _segments_intersect(segs[names[i]][0], segs[names[i]][1],
                                       segs[names[j]][0], segs[names[j]][1]):
                    ok = False
                    print(f"   ✗ {names[i]} 与 {names[j]} 的轨迹**相交**，两个障碍物会互撞！")

        def dist_to_seg(pt, a, b):
            ax, ay = a; bx, by = b; px, py = pt
            dx, dy = bx - ax, by - ay
            denom = dx * dx + dy * dy
            t = 0.0 if denom == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
            return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

        for nm, (a, b) in segs.items():
            if dist_to_seg(spawn, a, b) < 0.8:
                ok = False
                print(f"   ✗ {nm} 的轨迹离机器人出生点 {spawn} 太近"
                      f"({dist_to_seg(spawn, a, b):.2f} m < 0.8 m)，开机就会把车撞飞！")
            for pname, px, py in park_pts:
                if dist_to_seg((px, py), a, b) < 0.5:
                    ok = False
                    print(f"   ✗ {nm} 的轨迹会撞到 {pname} 的停车作业点 "
                          f"({px:.2f},{py:.2f})，距离仅 {dist_to_seg((px, py), a, b):.2f} m")
        if ok:
            print("   ✓ 轨迹互不相交、不压出生点、不压停车作业点（安全）")
        print("   ⚠ 仍需确认障碍物位于「机器人→货物→放置区」的路线附近，")
        print("     否则规则判定为无效障碍物，不给分。")

    # 官方讲解："三点满足两点即得 10 分"（规则原文写的是每点 5 分共 10 分，两处有差异）
    passed = 0
    passed += 1 if not bad else 0                                  # ① 货物间距
    passed += 1 if (layout.get("zone_walls") or {}) else 0          # ② 隔墙
    passed += 1 if len(obstacles) >= 2 else 0                       # ③ 有效障碍物
    print("\n" + "=" * 64)
    print(f"通过项数: {passed}/3")
    if passed >= 2:
        print("结论: ✓ 满足官方讲解的『三点中满足两点即得 10 分』→ 场景设计分可拿满")
        if passed == 2 and not (layout.get("zone_walls") or {}):
            print("       （未加隔墙：这是刻意的取舍——隔墙会显著增加导航难度，")
            print("         而按官方讲解两点即可得满分。想要三点全满可加 --walls）")
    else:
        print("结论: ✗ 不足两点，场景设计分会丢分")
    print("=" * 64)
    return passed >= 2


def _segments_intersect(p1, p2, p3, p4) -> bool:
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    d1, d2 = cross(p3, p4, p1), cross(p3, p4, p2)
    d3, d4 = cross(p1, p2, p3), cross(p1, p2, p4)
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    return False


def main() -> None:
    parser = argparse.ArgumentParser(description="比赛场景生成 / 校验")
    parser.add_argument("--layout", default=None, help="布局 YAML 文件")
    parser.add_argument("--check", action="store_true", help="只校验，不生成")
    parser.add_argument("--dump-layout", action="store_true", help="导出默认布局 YAML 供修改")
    parser.add_argument("--walls", action="store_true",
                        help="加上放置区隔墙（满足场景设计分第②条，但会明显增加导航难度）")
    args = parser.parse_args()

    layout = load_layout(args.layout)
    if args.walls and not layout.get("zone_walls"):
        layout["zone_walls"] = {
            "wall_1": [[-1.1, -6.0], [-1.1, -3.4]],
            "wall_2": [[1.1, -6.0], [1.1, -3.4]],
        }
        print("[场景] 已按 --walls 加上隔墙")

    if args.dump_layout:
        target = LAYOUT_FILE
        os.makedirs(os.path.dirname(target), exist_ok=True)
        with open(target, "w", encoding="utf-8") as fp:
            yaml.safe_dump(layout, fp, allow_unicode=True, sort_keys=False)
        print(f"默认布局已导出: {target}")
        return

    if not args.check:
        generate(layout)
        print()
    sys.exit(0 if check(layout) else 1)


if __name__ == "__main__":
    main()
