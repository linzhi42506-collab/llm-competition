"""场景坐标记录工具。

改造完比赛场景后，用它把 Gazebo 里货物 / 放置区的真实坐标一键导出成
``competition.yaml`` 能直接使用的 YAML 片段：

    ros2 run llm_comp record_positions                 # 打印到屏幕
    ros2 run llm_comp record_positions --ros-args -p output:=/tmp/pose.yaml

为什么需要它：官方讲解说明"场景中的物块位置和放置区域的位置自行设计，可以提前
记录这些坐标"。把设计好的坐标固化进配置，比赛时就无需现场视觉标定，速度更快、
更稳。
"""

from __future__ import annotations

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

try:
    from gazebo_msgs.msg import ModelStates

    _HAS_GAZEBO = True
except Exception:  # pragma: no cover
    _HAS_GAZEBO = False


class RecordPositions(Node):
    def __init__(self) -> None:
        super().__init__("record_positions")

        # 仿真环境必须用仿真时钟，否则消息时间戳会和 TF 对不上
        # （踩过坑：时间戳是墙上时钟 → AMCL 无法定位 → 导航全废）
        try:
            from rclpy.parameter import Parameter
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        except Exception:
            pass
        self.declare_parameter("output", "")
        self.declare_parameter("wait", 5.0)

        self.cubes: dict = {}
        self.zones: dict = {}
        self.done = False

        if not _HAS_GAZEBO:
            self.get_logger().error("未安装 gazebo_msgs（apt install ros-humble-gazebo-msgs）")
            self.done = True
            return

        self.create_subscription(ModelStates, "/gazebo/model_states", self.on_states, 10)
        # 兼容部分 gazebo_ros_state 构建把话题放在根命名空间的情况
        self.create_subscription(ModelStates, "/model_states", self.on_states, 10)
        self.create_timer(float(self.get_parameter("wait").value), self.finish)

    def on_states(self, msg) -> None:
        for name, pose in zip(msg.name, msg.pose):
            p = pose.position
            if "cube" in name:
                self.cubes[name] = (p.x, p.y, p.z)
            elif name.startswith("zone_"):
                self.zones[name] = (p.x, p.y, p.z)

    def finish(self) -> None:
        lines = ["# ---- 由 record_positions 自动生成 ----", "zones:"]
        for name in sorted(self.zones):
            x, y, _ = self.zones[name]
            lines.append(f"  {name.replace('zone_', '').upper()}: {{x: {x:.3f}, y: {y:.3f}, yaw: 0.0}}")
        lines.append("")
        lines.append("cubes:")
        for name in sorted(self.cubes):
            x, y, z = self.cubes[name]
            lines.append(f"  {name}: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}, yaw: 3.14159}}")
        text = "\n".join(lines)
        print(text, flush=True)

        output = str(self.get_parameter("output").value or "")
        if output:
            with open(output, "w", encoding="utf-8") as fp:
                fp.write(text + "\n")
            print(f"[record_positions] 已写入 {output}", file=sys.stderr, flush=True)
        self.done = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecordPositions()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
