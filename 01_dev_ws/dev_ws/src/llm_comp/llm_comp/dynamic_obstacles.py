"""动态（可移动）障碍物节点 —— 对应评分项 5「机器人避障」10 分。

评分要点：除机器人模型和货物模型外，场景中存在**其他可移动物体**，并且机器人
成功避障；每增加 1 个移动物体并成功实现避障得 5 分，满分 10 分。
障碍物还必须位于机器人前往货物/放置区的**路线附近**，否则视为无效障碍。

实现方式（官方讲解给出的思路：不断修改模型坐标实现轨迹运动）：
通过 Gazebo 的 ``set_entity_state`` 服务周期性修改障碍物模型的位姿，
让它在两个航点之间往复运动，形成"会动的障碍物"。

⚠ 三条安全约束（第一版没注意，导致两个圆柱互撞并把机器人撞飞，已修正）：
  1. 两条轨迹**不能相交**（否则两个障碍物自己撞在一起）；
  2. 轨迹**不能穿过机器人出生点**（否则开机瞬间把车撞飞）；
  3. 轨迹**不能经过机器人停车作业点**（否则抓取时被撞）。
  ``06_脚本/生成比赛场景.py --check`` 会自动校验这三条。

障碍物轨迹从 ``config/competition.yaml`` 的 ``obstacles:`` 段读取，
格式 ``"名称:Ax,Ay:Bx,By"``，也可以用同名 ROS 参数覆盖。
"""

from __future__ import annotations

import math
import os
from typing import Dict, List, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

try:
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import SetEntityState

    _HAS_GAZEBO = True
except Exception:  # pragma: no cover
    _HAS_GAZEBO = False

# 默认航点（与 competition.yaml 保持一致；两条轨迹平行、不相交、避开出生点）
DEFAULTS = {
    "obstacle_1": "obstacle_1:-2.6,1.0:2.6,1.0",
    "obstacle_2": "obstacle_2:-2.6,-1.0:2.6,-1.0",
    "obstacle_3": "",
}


class DynamicObstacles(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_obstacles")

        # 仿真环境必须用仿真时钟，否则消息时间戳会和 TF 对不上
        # （踩过坑：时间戳是墙上时钟 → AMCL 无法定位 → 导航全废）
        try:
            from rclpy.parameter import Parameter
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        except Exception:
            pass

        self.declare_parameter("config_file", "")
        self.declare_parameter("speed", 0.20)         # m/s，太快会把机器人撞翻
        self.declare_parameter("update_rate", 30.0)   # Hz，越高每步位移越小、冲击越轻
        self.declare_parameter("z", 0.24)
        self.declare_parameter("pause_radius", 0.75)  # 机器人静止且这么近时，障碍物停让
        self.declare_parameter("pause_speed", 0.03)   # 机器人线速度低于此值视为"停着"
        self.declare_parameter("start_delay", 0.0)    # 秒，赛前准备阶段先别动
        self.declare_parameter("enabled", True)
        for key, value in DEFAULTS.items():
            self.declare_parameter(key, value)

        self.speed = float(self.get_parameter("speed").value)
        self.z = float(self.get_parameter("z").value)
        self.pause_radius = float(self.get_parameter("pause_radius").value)
        self.pause_speed = float(self.get_parameter("pause_speed").value)
        self.robot_xy = None
        self.robot_v = 0.0
        self.robot_still_since = 0.0
        self.start_delay = float(self.get_parameter("start_delay").value)
        self.enabled = bool(self.get_parameter("enabled").value)
        rate = float(self.get_parameter("update_rate").value)

        self.obstacles: List[Dict] = []
        for spec in self._collect_specs():
            parsed = self._parse_spec(spec)
            if parsed:
                self.obstacles.append(parsed)

        self.state_clients = []
        if _HAS_GAZEBO:
            # 兼容 /gazebo/set_entity_state 与 /set_entity_state 两种命名
            self.state_clients.append(self.create_client(SetEntityState, "/gazebo/set_entity_state"))
            self.state_clients.append(self.create_client(SetEntityState, "/set_entity_state"))
        else:
            self.get_logger().error("未安装 gazebo_msgs，动态障碍物无法运行")

        try:
            from nav_msgs.msg import Odometry
            self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        except Exception:
            pass

        self.elapsed = 0.0
        self.dt = 1.0 / max(rate, 1.0)
        self.create_timer(self.dt, self.tick)

        if not self.obstacles:
            self.get_logger().warn("未配置任何动态障碍物（enabled=false 或 obstacles 为空）")
        else:
            self.get_logger().info(
                f"动态障碍物已启用: {[o['name'] for o in self.obstacles]} "
                f"速度 {self.speed} m/s，启动延迟 {self.start_delay}s")
            self._warn_if_unsafe()

    # ------------------------------------------------------------------ #
    def _collect_specs(self) -> List[str]:
        """优先从 competition.yaml 读取，其次用 ROS 参数。"""
        specs: List[str] = []
        table = self._load_config().get("obstacles") or {}
        if isinstance(table, dict):
            for name, spec in table.items():
                if isinstance(spec, str) and spec:
                    specs.append(spec if ":" in spec else f"{name}:{spec}")
        elif isinstance(table, list):
            specs.extend(str(x) for x in table if x)

        for key, default in DEFAULTS.items():
            value = str(self.get_parameter(key).value or "")
            if value:
                specs.append(value)
            elif default and not specs:
                specs.append(default)

        seen, unique = set(), []
        for spec in specs:
            name = spec.split(":", 1)[0].strip()
            if name and name not in seen:
                seen.add(name)
                unique.append(spec)
        return unique

    def _load_config(self) -> dict:
        path = str(self.get_parameter("config_file").value or "")
        if not path:
            try:
                path = os.path.join(
                    get_package_share_directory("llm_comp"), "config", "competition.yaml")
            except Exception:
                return {}
        try:
            with open(path, "r", encoding="utf-8") as fp:
                return yaml.safe_load(fp) or {}
        except Exception:
            return {}

    # ------------------------------------------------------------------ #
    @staticmethod
    def _parse_spec(spec: str) -> Optional[Dict]:
        try:
            name, rest = spec.split(":", 1)
            a_str, b_str = rest.split(":")
            ax, ay = [float(v) for v in a_str.split(",")]
            bx, by = [float(v) for v in b_str.split(",")]
            return {"name": name.strip(), "a": (ax, ay), "b": (bx, by),
                    "t": 0.0, "dir": 1, "len": max(math.hypot(bx - ax, by - ay), 1e-3)}
        except Exception:
            return None

    def _warn_if_unsafe(self) -> None:
        """运行时再自查一次，避免 world 里的位置与配置不一致。"""
        for obs in self.obstacles:
            if self._dist_to_seg((0.0, 0.0), obs["a"], obs["b"]) < 0.8:
                self.get_logger().error(
                    f"{obs['name']} 的轨迹距机器人出生点过近，可能把机器人撞飞！"
                    "请修改 competition.yaml 的 obstacles 段")
        for i in range(len(self.obstacles)):
            for j in range(i + 1, len(self.obstacles)):
                if self._segments_intersect(self.obstacles[i]["a"], self.obstacles[i]["b"],
                                            self.obstacles[j]["a"], self.obstacles[j]["b"]):
                    self.get_logger().error(
                        f"{self.obstacles[i]['name']} 与 {self.obstacles[j]['name']} 的轨迹相交，"
                        "两个障碍物会互撞！请修改 competition.yaml")

    @staticmethod
    def _dist_to_seg(pt, a, b) -> float:
        (px, py), (ax, ay), (bx, by) = pt, a, b
        dx, dy = bx - ax, by - ay
        denom = dx * dx + dy * dy
        t = 0.0 if denom == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
        return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

    @staticmethod
    def _segments_intersect(p1, p2, p3, p4) -> bool:
        def cross(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

        d1, d2 = cross(p3, p4, p1), cross(p3, p4, p2)
        d3, d4 = cross(p1, p2, p3), cross(p1, p2, p4)
        return ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
               ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0))

    def on_odom(self, msg) -> None:
        p = msg.pose.pose.position
        self.robot_xy = (p.x, p.y)
        v = msg.twist.twist.linear
        self.robot_v = math.hypot(v.x, v.y)

    def _should_pause(self, obs) -> bool:
        """安全停让：机器人停在障碍物路线上时，别去撞它。

        这是保护措施——机器人停着不动（比如正在抓取）时被障碍物顶一下最容易翻车。
        比赛时机器人一直在移动，几乎不会触发。
        """
        if self.robot_xy is None:
            return False
        ax, ay = obs["a"]
        bx, by = obs["b"]
        if self._dist_to_seg(self.robot_xy, (ax, ay), (bx, by)) > self.pause_radius:
            return False
        return self.robot_v < self.pause_speed

    # ------------------------------------------------------------------ #
    def tick(self) -> None:
        self.elapsed += self.dt
        if not self.enabled or not self.state_clients or not self.obstacles:
            return
        if self.elapsed < self.start_delay:
            return
        for obs in self.obstacles:
            if self._should_pause(obs):
                continue          # 机器人停在我前面，先等着
            obs["t"] += obs["dir"] * self.speed * self.dt / obs["len"]
            if obs["t"] >= 1.0:
                obs["t"], obs["dir"] = 1.0, -1
            elif obs["t"] <= 0.0:
                obs["t"], obs["dir"] = 0.0, 1
            ax, ay = obs["a"]
            bx, by = obs["b"]
            self._set_state(obs["name"], ax + (bx - ax) * obs["t"], ay + (by - ay) * obs["t"],
                            math.atan2(by - ay, bx - ax))

    def _set_state(self, name: str, x: float, y: float, yaw: float) -> None:
        client = next((c for c in self.state_clients if c.service_is_ready()), None)
        if client is None:
            for candidate in self.state_clients:
                if candidate.wait_for_service(timeout_sec=0.05):
                    client = candidate
                    break
        if client is None:
            return
        request = SetEntityState.Request()
        state = EntityState()
        state.name = name
        state.pose = Pose()
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = self.z
        state.pose.orientation.z = math.sin(yaw / 2.0)
        state.pose.orientation.w = math.cos(yaw / 2.0)
        state.twist = Twist()
        state.reference_frame = "world"
        request.state = state
        client.call_async(request)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicObstacles()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
