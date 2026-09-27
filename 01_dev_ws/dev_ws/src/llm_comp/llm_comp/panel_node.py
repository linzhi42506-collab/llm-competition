"""可视化面板聚合节点 —— 对应评分项 6「状态反馈」10 分。

比赛要求面板必须呈现：① 任务信息（题目、抓取顺序、每种资源数量）；② 导航路径
（机器人实时路径）；③ 当前状态（任务进度、工作状态、末端状态）。

本节点把所有信息聚合成**一个 JSON 话题** ``/competition/panel``，这样在
Foxglove / coStudio 里只用一个 "Raw Message / JSON" 面板就能显示全部内容，
减少现场布局出错的风险；同时把路径转成 ``nav_msgs/Path`` 方便用 3D 面板画线。

话题一览::

    /competition/panel          String(JSON)   面板总数据
    /competition/panel_text     String         纯文本（直接贴面板）
    /competition/nav_path       Path           Nav2 规划的路径
    /competition/robot_path     Path           机器人实际走过的路径
    /competition/task_info_text String         任务信息文本
    /competition/task_status_text String       当前状态文本
    /competition/detections_text  String       识别结果文本
"""

from __future__ import annotations

import json
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

try:
    from visualization_msgs.msg import Marker, MarkerArray

    _HAS_MARKERS = True
except Exception:  # pragma: no cover
    _HAS_MARKERS = False


class PanelNode(Node):
    def __init__(self) -> None:
        super().__init__("panel_node")

        # 仿真环境必须用仿真时钟，否则消息时间戳会和 TF 对不上
        # （踩过坑：时间戳是墙上时钟 → AMCL 无法定位 → 导航全废）
        try:
            from rclpy.parameter import Parameter
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        except Exception:
            pass

        self.declare_parameter("path_topic", "/plan")
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("max_path_points", 2000)
        self.declare_parameter("publish_rate", 2.0)

        self.max_points = int(self.get_parameter("max_path_points").value)

        self.task_info_text = "等待大模型解析任务..."
        self.task_status_text = "等待任务开始..."
        self.detection_text = "等待识别结果..."
        self.task_info: dict = {}
        self.task_status: dict = {}

        self.create_subscription(String, "/competition/task_info", self.on_task_info, 10)
        self.create_subscription(String, "/competition/task_status", self.on_task_status, 10)
        self.create_subscription(String, "/competition/detections", self.on_detections, 10)
        self.create_subscription(String, "/competition/task_info_text",
                                 lambda m: setattr(self, "task_info_text", m.data), 10)
        self.create_subscription(String, "/competition/task_status_text",
                                 lambda m: setattr(self, "task_status_text", m.data), 10)
        self.create_subscription(String, "/competition/detections_text",
                                 lambda m: setattr(self, "detection_text", m.data), 10)

        self.create_subscription(Path, str(self.get_parameter("path_topic").value), self.on_plan_path, 10)
        self.create_subscription(PoseStamped, str(self.get_parameter("pose_topic").value), self.on_pose, 10)

        self.pub_panel = self.create_publisher(String, "/competition/panel", 10)
        self.pub_panel_text = self.create_publisher(String, "/competition/panel_text", 10)
        self.pub_nav_path = self.create_publisher(Path, "/competition/nav_path", 10)
        self.pub_robot_path = self.create_publisher(Path, "/competition/robot_path", 10)
        self.pub_markers = (
            self.create_publisher(MarkerArray, "/competition/panel_markers", 10) if _HAS_MARKERS else None)

        self.robot_path = Path()
        self.robot_path.header.frame_id = "map"
        self.nav_path: Optional[Path] = None
        self.last_pose: Optional[PoseStamped] = None

        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / max(rate, 0.5), self.publish_panel)
        self.get_logger().info(
            f"可视化聚合节点就绪：规划路径={self.get_parameter('path_topic').value}, "
            f"定位={self.get_parameter('pose_topic').value}")

    # ------------------------------------------------------------------ #
    def on_task_info(self, msg: String) -> None:
        try:
            self.task_info = json.loads(msg.data)
        except Exception:
            self.task_info = {"raw": msg.data}

    def on_task_status(self, msg: String) -> None:
        try:
            self.task_status = json.loads(msg.data)
        except Exception:
            self.task_status = {"raw": msg.data}

    def on_detections(self, msg: String) -> None:
        try:
            self.last_detection = json.loads(msg.data)
        except Exception:
            self.last_detection = {}

    def on_plan_path(self, msg: Path) -> None:
        self.nav_path = msg
        self.pub_nav_path.publish(msg)

    def on_pose(self, msg: PoseStamped) -> None:
        self.last_pose = msg
        self.robot_path.header.stamp = msg.header.stamp
        self.robot_path.header.frame_id = msg.header.frame_id or "map"
        self.robot_path.poses.append(msg)
        if len(self.robot_path.poses) > self.max_points:
            self.robot_path.poses = self.robot_path.poses[-self.max_points:]
        self.pub_robot_path.publish(self.robot_path)

    # ------------------------------------------------------------------ #
    def publish_panel(self) -> None:
        plan = self.task_info.get("plan", [])
        order = " → ".join(
            f"{item.get('color')}x{item.get('count')}@{item.get('zone')}" for item in plan) or "等待解析"

        payload = {
            "1_任务信息": {
                "题目": self.task_info.get("question", ""),
                "映射规则": self.task_info.get("rule", ""),
                "抓取顺序": order,
                "结构化指令": self.task_info.get("structured", ""),
                "解析来源": self.task_info.get("source", ""),
                "每种资源数量": plan,
            },
            "2_导航路径": {
                "规划路径点数": len(self.nav_path.poses) if self.nav_path else 0,
                "已走过点数": len(self.robot_path.poses),
                "当前位姿": self._pose_text(),
            },
            "3_当前状态": {
                "任务进度": self.task_status.get("progress_text", "等待任务"),
                "工作状态": self.task_status.get("work_state", "空闲"),
                "末端状态": self.task_status.get("gripper_state", "空闲张开"),
                "当前目标": self.task_status.get("current_cube", ""),
                "目标区域": self.task_status.get("current_zone", ""),
                "已用时(s)": self.task_status.get("elapsed", 0),
            },
            "4_识别结果": self.detection_text,
        }
        self.pub_panel.publish(String(data=json.dumps(payload, ensure_ascii=False)))

        text = "\n".join([
            "========== 大模型仓储分拣 · 可视化面板 ==========",
            "【1 任务信息】",
            f"  题目: {payload['1_任务信息']['题目']}",
            f"  规则: {payload['1_任务信息']['映射规则']}",
            f"  抓取顺序: {order}",
            f"  结构化指令: {payload['1_任务信息']['结构化指令']}",
            "【2 导航路径】",
            f"  规划点数: {payload['2_导航路径']['规划路径点数']}  "
            f"已走点数: {payload['2_导航路径']['已走过点数']}",
            f"  当前位姿: {payload['2_导航路径']['当前位姿']}",
            "【3 当前状态】",
            f"  任务进度: {payload['3_当前状态']['任务进度']}",
            f"  工作状态: {payload['3_当前状态']['工作状态']}",
            f"  末端状态: {payload['3_当前状态']['末端状态']}",
            "【4 识别结果】",
            self.detection_text,
        ])
        self.pub_panel_text.publish(String(data=text))

        if self.pub_markers is not None:
            self.pub_markers.publish(self._markers())

    def _pose_text(self) -> str:
        if self.last_pose is None:
            return "未定位"
        p = self.last_pose.pose.position
        return f"({p.x:.2f}, {p.y:.2f})"

    def _markers(self):
        array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "panel"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        status = self.task_status
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.5
        marker.scale.z = 0.3
        marker.color.r = marker.color.g = marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = (f"{status.get('progress_text', '')}\n{status.get('work_state', '')}\n"
                       f"{status.get('gripper_state', '')}")
        array.markers.append(marker)
        return array


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PanelNode()
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
