#!/usr/bin/env python3
"""用 Gazebo 真值位姿生成**完整准确**的栅格地图（不需要 SLAM 探索）。

原理
----
仿真里我们能拿到机器人的精确位姿（`/gazebo/set_entity_state` 设过去，
读回来的就是真值，没有里程计漂移）。于是：

  1. 把机器人**瞬移**到覆盖整个作业区域的网格点上（含 4 个朝向）；
  2. 每到一个点等一帧新的 `/scan`；
  3. 用已知位姿把每一条激光射线**反投影**到全局栅格：
     射线路径上标记"可通行"，命中点标记"障碍"；
  4. 汇总成 PGM + YAML，格式和 `map_server` 完全兼容。

比让机器人自己跑 SLAM 快得多、也完整得多（几秒 vs 几分钟，且不留未知区域），
非常适合仿真比赛用来"重新建图"。

用法::

    # 前置：Gazebo 已经在跑（competition_world.launch.py）
    python3 06_脚本/08_生成真值地图.py
    # 或指定范围/输出
    python3 06_脚本/08_生成真值地图.py --xmin -8 --xmax 8 --ymin -9 --ymax 5 \
        --out <maps>/competition_map
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    from gazebo_msgs.msg import EntityState, ModelStates
    from gazebo_msgs.srv import SetEntityState
    from geometry_msgs.msg import Pose, Twist

    _OK = True
except Exception as exc:  # pragma: no cover
    print(f"缺少 gazebo_msgs: {exc}", file=sys.stderr)
    _OK = False

ROBOT = "six_arm"
FREE, OCC, UNKNOWN = 254, 0, 205


class TruthMapper(Node):
    def __init__(self, args) -> None:
        super().__init__("truth_mapper")
        self.args = args
        self.scan = None
        self.scan_seq = 0
        self.pose = (0.0, 0.0, 0.0)

        self.res = args.resolution
        self.xmin, self.xmax = args.xmin, args.xmax
        self.ymin, self.ymax = args.ymin, args.ymax
        self.w = int((self.xmax - self.xmin) / self.res)
        self.h = int((self.ymax - self.ymin) / self.res)
        # 0=障碍 254=空闲 205=未知
        self.grid = bytearray([UNKNOWN]) * (self.w * self.h)

        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        # 关键: 瞬移后机器人会因物理沉降/碰撞偏离设定位置，
        # 必须用 Gazebo 回报的**真实位姿**作为射线原点，否则地图会出现放射状噪声。
        try:
            from gazebo_msgs.msg import ModelStates
            self.create_subscription(ModelStates, "/gazebo/model_states", self.on_models, 10)
            self.create_subscription(ModelStates, "/model_states", self.on_models, 10)
            self.true_pose = None
        except Exception:
            self.true_pose = None
        self.state_clients = [
            self.create_client(SetEntityState, "/gazebo/set_entity_state"),
            self.create_client(SetEntityState, "/set_entity_state"),
        ]
        self.get_logger().info(
            f"栅格 {self.w}x{self.h}，范围 x[{self.xmin},{self.xmax}] y[{self.ymin},{self.ymax}]，"
            f"分辨率 {self.res} m")

    # ------------------------------------------------------------------ #
    def on_scan(self, msg: LaserScan) -> None:
        self.scan = msg
        self.scan_seq += 1

    def on_models(self, msg) -> None:
        try:
            i = list(msg.name).index(ROBOT)
        except ValueError:
            return
        p = msg.pose[i]
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.true_pose = (p.position.x, p.position.y, yaw)

    def _client(self, timeout: float = 0.5):
        deadline = time.time() + timeout
        while time.time() < deadline:
            for c in self.state_clients:
                if c.service_is_ready():
                    return c
            # 给服务发现留时间：必须 spin，否则图变化看不到
            rclpy.spin_once(self, timeout_sec=0.05)
        for c in self.state_clients:
            if c.wait_for_service(timeout_sec=0.1):
                return c
        return None

    def teleport(self, x: float, y: float, yaw: float) -> bool:
        client = self._client()
        if client is None:
            return False
        req = SetEntityState.Request()
        st = EntityState()
        st.name = ROBOT
        st.pose = Pose()
        st.pose.position.x = x
        st.pose.position.y = y
        st.pose.position.z = 0.05
        st.pose.orientation.z = math.sin(yaw / 2.0)
        st.pose.orientation.w = math.cos(yaw / 2.0)
        st.twist = Twist()
        st.reference_frame = "world"
        req.state = st
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        self.pose = (x, y, yaw)
        return True

    # ------------------------------------------------------------------ #
    def _to_cell(self, x: float, y: float):
        cx = int((x - self.xmin) / self.res)
        cy = int((y - self.ymin) / self.res)
        if 0 <= cx < self.w and 0 <= cy < self.h:
            return cx, cy
        return None

    def _mark(self, cx: int, cy: int, value: int) -> None:
        if 0 <= cx < self.w and 0 <= cy < self.h:
            idx = cy * self.w + cx
            cur = self.grid[idx]
            if value == OCC:
                self.grid[idx] = OCC
            elif cur == UNKNOWN:
                self.grid[idx] = FREE

    def integrate(self) -> None:
        """把当前这一帧 scan 按已知位姿打进栅格。"""
        if self.scan is None:
            return
        s = self.scan
        x0, y0, yaw = self.true_pose if self.true_pose else self.pose
        angle = s.angle_min
        for r in s.ranges:
            a = yaw + angle
            angle += s.angle_increment
            if r is None or math.isnan(r) or math.isinf(r) or r <= 0.0:
                continue
            if r < max(s.range_min, self.args.min_range) or r > s.range_max:
                continue
            steps = max(2, int(r / (self.res * 0.5)))
            for k in range(steps + 1):
                d = r * k / steps
                px, py = x0 + d * math.cos(a), y0 + d * math.sin(a)
                cell = self._to_cell(px, py)
                if cell is None:
                    continue
                self._mark(cell[0], cell[1], OCC if k == steps else FREE)

    # ------------------------------------------------------------------ #
    def run(self) -> None:
        step = self.args.step
        xs = []
        v = self.xmin + step / 2
        while v < self.xmax:
            xs.append(v)
            v += step
        ys = []
        v = self.ymin + step / 2
        while v < self.ymax:
            ys.append(v)
            v += step

        poses = [(x, y, yaw) for y in ys for x in xs for yaw in (0.0, math.pi / 2)]
        # 先等服务发现完成（初始几秒服务可能还看不到）
        if self._client(timeout=10.0) is None:
            print("找不到 set_entity_state 服务；确认 Gazebo 已启动且 world 里加载了 "
                  "libgazebo_ros_state.so 插件", file=sys.stderr)
            return
        print(f"共 {len(poses)} 个扫描位姿，开始 ...")
        ok = 0
        for i, (x, y, yaw) in enumerate(poses, 1):
            if not self.teleport(x, y, yaw):
                print("无法调用 set_entity_state，退出", file=sys.stderr)
                return
            # 等机器人停稳（物理沉降/碰撞需要时间）
            settle = time.time() + self.args.settle
            while time.time() < settle:
                rclpy.spin_once(self, timeout_sec=0.02)
            # 丢掉可能还是旧位姿的帧，再等一帧新的
            before = self.scan_seq
            deadline = time.time() + 1.5
            while self.scan_seq == before and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.02)
            for _ in range(3):
                rclpy.spin_once(self, timeout_sec=0.02)
            self.integrate()
            ok += 1
            if i % 40 == 0 or i == len(poses):
                print(f"  {i}/{len(poses)}")
        print(f"扫描完成（{ok} 个位姿）")

    def clean(self) -> None:
        """形态学开运算：腐蚀掉 1 像素宽的放射状噪声线，保留真正的墙体。

        真值扫描难免扫到机械臂自身和掠射角，会产生细线状伪障碍；
        这些细线在 Nav2 里会被膨胀成半米宽的"假墙"，必须清掉。
        """
        w, h, g = self.w, self.h, self.grid

        def neighbors_occupied(idx, need):
            count = 0
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    x = idx % w + dx
                    y = idx // w + dy
                    if 0 <= x < w and 0 <= y < h and g[y * w + x] == OCC:
                        count += 1
                        if count >= need:
                            return True
            return False

        # 腐蚀：孤立/细线状障碍点去掉（要求至少 3 个障碍邻居）
        eroded = bytearray(g)
        removed = 0
        need = self.args.min_neighbors
        for idx in range(w * h):
            if g[idx] == OCC and not neighbors_occupied(idx, need):
                eroded[idx] = FREE
                removed += 1
        final = eroded  # 只腐蚀不膨胀：1像素细线消失，厚墙保留
        # 连通域滤波：只保留面积足够大的障碍块（墙体），删掉零星小点。
        # Nav2 的膨胀半径会把每个小点膨胀成 ~1m 的禁区，
        # 散点一多整张图就没法规划了。
        self.grid = self._filter_components(final, self.args.min_component)
        print(f"  去噪：移除 {removed} 个细线噪声像素（阈值 {need} 邻居）")

    def _filter_components(self, grid: bytearray, min_size: int) -> bytearray:
        """删除面积小于 min_size 的障碍连通域。"""
        w, h = self.w, self.h
        visited = bytearray(w * h)
        out = bytearray(grid)
        removed_cells = 0
        removed_groups = 0
        for start in range(w * h):
            if grid[start] != OCC or visited[start]:
                continue
            stack = [start]
            visited[start] = 1
            group = []
            while stack:
                idx = stack.pop()
                group.append(idx)
                x, y = idx % w, idx // w
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < w and 0 <= ny < h:
                            nidx = ny * w + nx
                            if grid[nidx] == OCC and not visited[nidx]:
                                visited[nidx] = 1
                                stack.append(nidx)
            if len(group) < min_size:
                for idx in group:
                    out[idx] = FREE
                removed_cells += len(group)
                removed_groups += 1
        print(f"  连通域滤波：删除 {removed_groups} 个小障碍块，共 {removed_cells} 像素")
        return out

    def save(self, path: str) -> None:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path + ".pgm", "wb") as fp:
            fp.write(b"P5\n%d %d\n255\n" % (self.w, self.h))
            # PGM 第一行是最上方，对应 y 最大值 → 上下翻转
            for row in range(self.h - 1, -1, -1):
                fp.write(bytes(self.grid[row * self.w:(row + 1) * self.w]))
        with open(path + ".yaml", "w", encoding="utf-8") as fp:
            fp.write(
                f"image: {os.path.basename(path)}.pgm\n"
                f"mode: trinary\n"
                f"resolution: {self.res}\n"
                f"origin: [{self.xmin:.3f}, {self.ymin:.3f}, 0.0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.25\n"
            )
        occ = sum(1 for v in self.grid if v == OCC)
        free = sum(1 for v in self.grid if v == FREE)
        unk = self.w * self.h - occ - free
        print(f"已保存 {path}.pgm / .yaml")
        print(f"  尺寸 {self.w}x{self.h}  障碍 {occ}  可通行 {free}  未知 {unk}")
        print(f"  实际范围 {(self.xmax - self.xmin):.1f} m x {(self.ymax - self.ymin):.1f} m")


def main() -> None:
    if not _OK:
        sys.exit(1)
    ap = argparse.ArgumentParser(description="用真值位姿生成栅格地图")
    ap.add_argument("--out", default="/home/robo/llm_comp/01_dev_ws/dev_ws/src/yzbot/"
                                    "bot_navigation/maps/competition_map")
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--xmin", type=float, default=-8.0)
    ap.add_argument("--xmax", type=float, default=8.0)
    ap.add_argument("--ymin", type=float, default=-9.0)
    ap.add_argument("--ymax", type=float, default=5.0)
    ap.add_argument("--step", type=float, default=1.5, help="扫描网格间距(米)")
    ap.add_argument("--settle", type=float, default=0.45, help="每次瞬移后等待停稳的秒数")
    ap.add_argument("--min_range", type=float, default=0.6,
                    help="忽略小于该距离的回波（过滤机器人自身结构）")
    ap.add_argument("--min_component", type=int, default=30,
                    help="连通域滤波：障碍块小于该像素数就删掉（30像素≈0.075m²）")
    ap.add_argument("--min_neighbors", type=int, default=4,
                    help="去噪：障碍点至少要有几个障碍邻居才保留（细线会被删掉）")
    args = ap.parse_args()

    rclpy.init()
    node = TruthMapper(args)
    try:
        node.run()
        node.clean()
        node.save(args.out)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
