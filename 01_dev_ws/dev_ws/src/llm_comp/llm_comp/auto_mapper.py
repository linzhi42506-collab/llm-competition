"""自动巡游建图节点：让机器人自己在场景里跑一圈，配合 SLAM 生成地图。

用途：改造场景后，官方那张 `maps/map.yaml` 就和实际场景对不上了，
必须重新建图。本节点负责"自动驾驶"那部分——它会：

* 订阅激光雷达 ``/scan``，前方安全就直行；
* 前方太近就停下来转向，转向方向选**两侧空间更大**的那一边；
* 卡住（连续一段时间没有位移）就后退一下换个方向，避免原地磨墙。

配合 ``slam_toolbox`` 使用::

    # 终端1: 仿真
    ros2 launch llm_comp competition_world.launch.py
    # 终端2: SLAM
    ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
    # 终端3: 自动巡游
    ros2 run llm_comp auto_mapper --ros-args -p duration:=240.0
    # 终端4: 存图
    ros2 run nav2_map_server map_saver_cli -f <workspace>/src/yzbot/bot_navigation/maps/competition_map

也可以直接跑一键脚本 ``06_脚本/07_自动建图.sh``。
"""

from __future__ import annotations

import math
import random
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    from nav_msgs.msg import Odometry

    _HAS_ODOM = True
except Exception:  # pragma: no cover
    _HAS_ODOM = False


class AutoMapper(Node):
    def __init__(self) -> None:
        super().__init__("auto_mapper")

        self.declare_parameter("duration", 240.0)        # 巡游总时长(秒)
        self.declare_parameter("forward_speed", 0.22)    # m/s
        self.declare_parameter("turn_speed", 0.7)        # rad/s
        self.declare_parameter("safety_distance", 0.75)  # 前方多少米有障碍就转
        self.declare_parameter("front_angle", 40.0)      # 前方扇区半角(度)
        self.declare_parameter("stuck_timeout", 6.0)     # 多久没动算卡住
        self.declare_parameter("seed", 0)

        self.duration = float(self.get_parameter("duration").value)
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.safety = float(self.get_parameter("safety_distance").value)
        self.front_angle = math.radians(float(self.get_parameter("front_angle").value))
        self.stuck_timeout = float(self.get_parameter("stuck_timeout").value)
        self.rand = random.Random(int(self.get_parameter("seed").value) or None)

        self.scan: Optional[LaserScan] = None
        self.pos = (0.0, 0.0)
        self.state = "FORWARD"
        self.state_until = 0.0
        self.turn_dir = 1.0
        self.start = time.time()
        self.last_move_check = time.time()
        self.last_pos = (0.0, 0.0)
        self.finished = False

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        if _HAS_ODOM:
            self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_timer(0.1, self.tick)

        self.get_logger().info(
            f"自动巡游建图启动：时长 {self.duration:.0f}s，安全距离 {self.safety} m，"
            f"前进速度 {self.forward_speed} m/s")

    # ------------------------------------------------------------------ #
    def on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pos = (p.x, p.y)

    # ------------------------------------------------------------------ #
    def _sector_min(self, center: float, half: float) -> float:
        """取某个角度扇区内的最小距离。"""
        if self.scan is None:
            return float("inf")
        r = self.scan.ranges
        inc = self.scan.angle_increment
        start = self.scan.angle_min
        half_idx = max(1, int(half / max(inc, 1e-6)))
        center_idx = int((center - start) / max(inc, 1e-6))
        best = float("inf")
        for k in range(-half_idx, half_idx + 1):
            idx = (center_idx + k) % len(r)
            value = r[idx]
            if value is None or math.isnan(value) or value <= 0.0:
                continue
            if value < self.scan.range_min or value > self.scan.range_max:
                continue
            best = min(best, value)
        return best

    def _space(self, center: float, half: float) -> float:
        """扇区内的平均可用空间（越大越空）。"""
        if self.scan is None:
            return 0.0
        r = self.scan.ranges
        inc = self.scan.angle_increment
        start = self.scan.angle_min
        half_idx = max(1, int(half / max(inc, 1e-6)))
        center_idx = int((center - start) / max(inc, 1e-6))
        total, count = 0.0, 0
        for k in range(-half_idx, half_idx + 1):
            idx = (center_idx + k) % len(r)
            value = r[idx]
            if value is None or math.isnan(value) or value <= 0.0:
                continue
            total += min(value, 5.0)
            count += 1
        return total / count if count else 0.0

    # ------------------------------------------------------------------ #
    def tick(self) -> None:
        now = time.time()
        if now - self.start > self.duration:
            self._stop()
            self.get_logger().info("巡游时间到，停止。可以保存地图了。")
            self.finished = True
            return
        if self.scan is None:
            return

        # --- 卡住检测 ---
        if now - self.last_move_check > self.stuck_timeout:
            moved = math.hypot(self.pos[0] - self.last_pos[0], self.pos[1] - self.last_pos[1])
            if moved < 0.10:
                self.get_logger().warn("检测到卡住，后退并换方向")
                self._back_off()
            self.last_pos = self.pos
            self.last_move_check = now

        if now < self.state_until:
            return

        front = self._sector_min(0.0, self.front_angle)
        if front > self.safety:
            self.state = "FORWARD"
            self._drive(self.forward_speed, 0.0)
        else:
            left = self._space(math.pi / 2, math.radians(30))
            right = self._space(-math.pi / 2, math.radians(30))
            self.turn_dir = 1.0 if left >= right else -1.0
            if abs(left - right) < 0.25:
                self.turn_dir = self.rand.choice([-1.0, 1.0])
            self.state = "TURN"
            self.state_until = now + self.rand.uniform(0.8, 1.8)
            self._drive(0.0, self.turn_speed * self.turn_dir)

    def _back_off(self) -> None:
        self._drive(-0.15, 0.0)
        time.sleep(1.0)
        self._drive(0.0, self.turn_speed * self.rand.choice([-1.0, 1.0]))
        time.sleep(1.2)
        self._stop()

    def _drive(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)

    def _stop(self) -> None:
        for _ in range(5):
            self._drive(0.0, 0.0)
            time.sleep(0.05)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoMapper()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node._stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
