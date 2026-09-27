"""里程计 TF 补丁节点：发布 ``odom → base_footprint`` 变换。

**为什么需要它**

Nav2 的代价地图要求 TF 链完整：``map → odom → base_footprint → laser_link``。
本仿真平台里 ``odom → base_footprint`` 本应由 Gazebo 的差分驱动插件
（``libgazebo_ros_diff_drive.so``，xacro 里已设 ``publish_odom_tf: true``）发布，
实测它**只在机器人运动时偶发地发**，机器人静止时 `/tf` 里完全没有 odom 帧。

后果非常严重：
* 局部/全局代价地图卡在 ``activating`` 状态；
* ``controller_server`` / ``planner_server`` 无法激活；
* 表现就是"导航完全不动、不会避障"。

本节点订阅 ``/odom``，用标准 ``tf2_ros.TransformBroadcaster`` 以固定频率
补发 ``odom → base_footprint``，**不改动官方任何文件**，把这条链补齐。

用法::

    ros2 run llm_comp odom_tf_broadcaster
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__("odom_tf_broadcaster")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_rate", 30.0)

        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        rate = float(self.get_parameter("publish_rate").value)

        self.broadcaster = TransformBroadcaster(self)
        self.latest: Odometry | None = None
        self.count = 0

        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self.on_odom, 10)
        self.create_timer(1.0 / max(rate, 1.0), self.publish)

        self.get_logger().info(
            f"里程计 TF 补丁已启动：{self.odom_frame} → {self.base_frame}，{rate:.0f} Hz")

    def on_odom(self, msg: Odometry) -> None:
        self.latest = msg

    def publish(self) -> None:
        if self.latest is None:
            return
        tf = TransformStamped()
        tf.header.stamp = self.latest.header.stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        # 只发布平面位姿（z、roll、pitch 交给静态 TF）
        tf.transform.translation.x = self.latest.pose.pose.position.x
        tf.transform.translation.y = self.latest.pose.pose.position.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = self.latest.pose.pose.orientation
        self.broadcaster.sendTransform(tf)
        self.count += 1
        if self.count == 30:
            self.get_logger().info("已开始发布 odom → base_footprint（导航现在可以激活了）")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
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
