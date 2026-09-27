"""出题程序（模拟版）。

正式比赛的题目由官方「出题程序」随机生成；本节点用于**赛前自测**：
生成同样规律的题目（x,y >= 1 且 x+y = 5，并在题目里给出映射规则），
发布到 ``/competition/question``，供 :mod:`llm_comp.llm_task_parser` 消费。

题目规律（来自官方讲解）：
* 计算结果 x、y 都大于等于 1，并且 x + y = 5；
* 题目中会给出「计算结果与颜色、区域的映射规则」。

用法::

    ros2 run llm_comp question_generator                # 随机出 1 题并发布
    ros2 run llm_comp question_generator --ros-args -p count:=5 -p interval:=0
"""

from __future__ import annotations

import math
import random
from typing import List, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

COLORS = ["红色", "蓝色"]
ZONES = ["A", "B", "C"]
NAMES = ["小美", "小帅", "小丽", "小娜", "小杰", "小刚", "小雅", "阿伟", "阿珍", "小北", "小林", "阿豪"]
ITEMS = [("T恤", "件"), ("裤子", "条")]


def _distribute(total: int, people: List[str], rand: random.Random) -> List[int]:
    """把 ``total`` 件物品随机分配给若干人，保证每人都 >= 1。"""
    n = len(people)
    if total < n:
        people = people[:total]
        n = total
    shares = [1] * n
    for _ in range(total - n):
        shares[rand.randrange(n)] += 1
    return shares


def make_question(rand: random.Random, capacity: int = 5) -> Tuple[str, int, int, str]:
    """生成一道符合比赛规律的题目，返回 ``(题干, x, y, 映射规则)``。"""
    x = rand.randint(1, 4)
    y = 5 - x

    # 让 T 恤总量落在 ((x-1)*cap, x*cap]，裤子总量落在 ((y-1)*cap, y*cap]
    shirt_total = rand.randint((x - 1) * capacity + 1, x * capacity)
    pants_total = rand.randint((y - 1) * capacity + 1, y * capacity)

    n_people = rand.randint(3, 5)
    people = rand.sample(NAMES, n_people)

    shirt_shares = _distribute(shirt_total, people[:], rand)
    pants_shares = _distribute(pants_total, people[:], rand)

    clauses: List[str] = []
    for idx, name in enumerate(people):
        parts = []
        if idx < len(shirt_shares) and shirt_shares[idx] > 0:
            parts.append(f"{shirt_shares[idx]}件T恤")
        if idx < len(pants_shares) and pants_shares[idx] > 0:
            parts.append(f"{pants_shares[idx]}条裤子")
        if not parts:
            continue
        verb = "需要" if len(parts) > 1 else rand.choice(["需要", "要买", "订购"])
        clauses.append(f"{name}{verb}" + "、".join(parts))

    question = (
        f"每个衣柜有{capacity}件衣物;有T恤、裤子两种衣柜;"
        + ";".join(clauses)
        + "。设T恤衣柜数量为x,裤子衣柜数量为y,计算x、y分别为多少。"
    )

    # 映射规则：x、y 分别代表红/蓝（顺序随机），再给一条阈值分区规则
    x_color, y_color = (COLORS[0], COLORS[1]) if rand.random() < 0.5 else (COLORS[1], COLORS[0])
    threshold = rand.randint(2, 4)
    high_zone, low_zone = rand.sample(ZONES, 2)
    rule = (
        f"x代表{x_color},y代表{y_color},"
        f"数量大于等于{threshold}的去{high_zone}区，其他颜色去{low_zone}区"
    )
    return question, x, y, rule


class QuestionGenerator(Node):
    def __init__(self) -> None:
        super().__init__("question_generator")

        # 仿真环境必须用仿真时钟，否则消息时间戳会和 TF 对不上
        # （踩过坑：时间戳是墙上时钟 → AMCL 无法定位 → 导航全废）
        try:
            from rclpy.parameter import Parameter
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        except Exception:
            pass
        self.declare_parameter("count", 1)
        self.declare_parameter("interval", 0.0)
        self.declare_parameter("capacity", 5)
        self.declare_parameter("seed", 0)

        self.pub = self.create_publisher(String, "/competition/question", 10)
        self.count = int(self.get_parameter("count").value)
        self.interval = float(self.get_parameter("interval").value)
        self.capacity = int(self.get_parameter("capacity").value)
        seed = int(self.get_parameter("seed").value)
        self.rand = random.Random(seed or None)

        self._sent = 0
        self.done = False
        # interval>0：按固定间隔连续出题；interval==0：启动 1.5s 后一次性出题
        period = self.interval if self.interval > 0 else 1.5
        self._timer = self.create_timer(period, self._tick)

    # ------------------------------------------------------------------ #
    def _tick(self) -> None:
        if self._sent >= self.count:
            self._timer.cancel()
            self.get_logger().info(f"出题完毕（共 {self._sent} 道）")
            self.done = True
            return
        self._publish_one()
        if self.interval <= 0 and self._sent >= self.count:
            self._timer.cancel()
            self.get_logger().info(f"出题完毕（共 {self._sent} 道）")
            self.done = True

    def _publish_one(self) -> None:
        question, x, y, rule = make_question(self.rand, self.capacity)
        text = f"题目:{question}映射规则:{rule}"
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self._sent += 1
        self.get_logger().info(
            f"\n===== 第 {self._sent} 题 =====\n{text}\n"
            f"[答案] x={x}, y={y}  (x+y={x + y})"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QuestionGenerator()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
