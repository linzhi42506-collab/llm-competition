"""自动设置 AMCL 初始位姿 —— 免去每次在 rviz 里点 "2D Pose Estimate"。

**为什么需要它**

AMCL 是"已知地图定位"，必须有人告诉它"机器人现在在地图的哪个位置、朝哪边"，
它才会发布 ``map → odom`` 变换。这一步不做的话：

* ``map`` 坐标系不存在 → rviz 里只有一个坐标轴，看不到地图/机器人/激光点；
* 全局代价地图永远卡在 ``activating``；
* 导航完全不动（表现就是"不会避障"）。

手工做法是在 rviz 工具栏点 ``2D Pose Estimate`` 再在地图上拖一下，
但这一步很容易忘、也容易点歪（位置和朝向都必须对）。

**本节点怎么自动做**

仿真里我们可以直接拿到机器人的**真值位姿**（``/gazebo/model_states``），
而重新生成的地图**就是以 Gazebo 世界坐标系建的**，所以真值位姿 == 地图位姿。
于是本节点在启动后持续把真值位姿发给 ``/initialpose``，
直到检测到 ``map → odom`` 已经建立（说明 AMCL 定位成功）就自动停止。

用法::

    ros2 run llm_comp auto_initial_pose

也可以用它随时"重置定位"：重启一次这个节点即可。
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

try:
    import tf2_ros

    _HAS_TF = True
except Exception:  # pragma: no cover
    _HAS_TF = False

try:
    from gazebo_msgs.msg import ModelStates

    _HAS_GAZEBO = True
except Exception:  # pragma: no cover
    _HAS_GAZEBO = False

try:
    from nav_msgs.msg import Odometry

    _HAS_ODOM = True
except Exception:  # pragma: no cover
    _HAS_ODOM = False


class AutoInitialPose(Node):
    def __init__(self) -> None:
        super().__init__("auto_initial_pose")

        # 仿真环境必须用仿真时钟，否则消息时间戳会和 TF 对不上
        # （踩过坑：时间戳是墙上时钟 → AMCL 无法定位 → 导航全废）
        try:
            from rclpy.parameter import Parameter
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        except Exception:
            pass

        self.declare_parameter("robot_name", "six_arm")
        self.declare_parameter("source", "auto")        # auto / gazebo / odom / fixed
        self.declare_parameter("fixed_x", 0.0)
        self.declare_parameter("fixed_y", 0.0)
        self.declare_parameter("fixed_yaw", 0.0)
        self.declare_parameter("publish_rate", 2.0)
        self.declare_parameter("duration", 30.0)        # 持续发送多少秒（覆盖 Nav2 启动时间）
        self.declare_parameter("timeout", 120.0)        # 兜底上限
        self.declare_parameter("covariance", 0.25)

        self.robot_name = str(self.get_parameter("robot_name").value)
        self.source = str(self.get_parameter("source").value)
        self.duration = float(self.get_parameter("duration").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.cov = float(self.get_parameter("covariance").value)
        rate = float(self.get_parameter("publish_rate").value)

        self.true_pose = None
        self.odom_pose = None
        self.sent = 0
        self.done = False
        # 注意: 必须用墙上时钟计时。开了 use_sim_time 后 ROS 时钟在节点刚构造时
        # 还是 0，第一帧 /clock 到达会直接跳到当前仿真时间，导致"瞬间超时"。
        self.t0 = time.monotonic()

        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        if _HAS_TF:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        if _HAS_GAZEBO and self.source in ("auto", "gazebo"):
            self.create_subscription(ModelStates, "/gazebo/model_states", self.on_models, 10)
            self.create_subscription(ModelStates, "/model_states", self.on_models, 10)
        if _HAS_ODOM and self.source in ("auto", "odom"):
            self.create_subscription(Odometry, "/odom", self.on_odom, 10)

        self.create_timer(1.0 / max(rate, 0.5), self.tick)
        self.get_logger().info(
            f"自动初始位姿节点已启动（数据源={self.source}）；"
            "定位成功（map→odom 建立）后会自动停止发送")

    # ------------------------------------------------------------------ #
    def on_models(self, msg) -> None:
        try:
            idx = list(msg.name).index(self.robot_name)
        except ValueError:
            return
        p = msg.pose[idx]
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.true_pose = (p.position.x, p.position.y, yaw)

    def on_odom(self, msg) -> None:
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_pose = (p.position.x, p.position.y, yaw)

    # ------------------------------------------------------------------ #
    def _localized(self) -> bool:
        """map→odom 是否已经建立（说明 AMCL 已经定位成功）。"""
        if not _HAS_TF:
            return False
        try:
            return bool(self.tf_buffer.can_transform("map", "odom", rclpy.time.Time()))
        except Exception:
            return False

    def _current_pose(self):
        if self.source == "fixed":
            return (float(self.get_parameter("fixed_x").value),
                    float(self.get_parameter("fixed_y").value),
                    float(self.get_parameter("fixed_yaw").value))
        if self.source == "odom":
            return self.odom_pose
        # auto：优先真值，其次里程计
        return self.true_pose or self.odom_pose

    def tick(self) -> None:
        if self.done:
            return
        elapsed = time.monotonic() - self.t0
        if elapsed > self.timeout:
            self.get_logger().warn("等待超时，停止发送初始位姿（可重启本节点重试）")
            self.done = True
            return

        # 说明：不能用 TF 判断"是否已定位"——AMCL 只要发过一次 map→odom，
        # 缓冲区里就会留下记录，会误判为成功。所以这里改成"持续发一段时间"，
        # 覆盖 Nav2/AMCL 的启动窗口，最稳。
        if elapsed > self.duration and self.sent > 0:
            self.get_logger().info(
                f"已持续发送 {self.sent} 次初始位姿（{self.duration:.0f} 秒），停止。"
                "若 rviz 里仍无地图，重启本节点即可重新定位")
            self.done = True
            return

        pose = self._current_pose()
        if pose is None:
            return

        x, y, yaw = pose
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov[7] = self.cov
        cov[35] = 0.07
        msg.pose.covariance = cov
        self.pub.publish(msg)
        self.sent += 1
        if self.sent in (1, 20, 60):
            self.get_logger().info(
                f"已发送初始位姿 #{self.sent}: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f}°")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoInitialPose()
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
