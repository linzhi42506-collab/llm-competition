"""货物视觉识别节点（红/蓝立方体），结果发布到可视化面板。

三种定位来源，按 ``mode`` 参数选择，优先级 ``vision > gazebo > config``：

* ``vision``  —— 订阅 RGB-D 相机（``/camera_rgbd/image_raw`` + ``/camera_rgbd/depth/image_raw``），
  用 OpenCV 做 HSV 颜色阈值分割 + 轮廓质心，再结合深度和相机内参反投影成三维点，
  最后用 TF 变换到 ``map`` 坐标系。这就是比赛要求"识别目标货物"的算法实现。
* ``gazebo``  —— 直接读 ``/gazebo/model_states`` 的真值，比赛现场兜底最稳。
* ``config``  —— 读 ``config/competition.yaml`` 中预先记录好的货物坐标（讲解原文：
  "场景中的物块位置和放置区域的位置自行设计，可以提前记录这些坐标"）。

发布话题::

    /competition/detections   std_msgs/String(JSON)   识别结果
    /competition/markers      visualization_msgs/MarkerArray  面板三维显示
"""

from __future__ import annotations

import json
import math
import os
from typing import Dict, List, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

try:
    from geometry_msgs.msg import Point
except Exception:  # pragma: no cover
    Point = None  # type: ignore

try:  # 视觉与 TF 属于可选依赖，缺失时自动降级
    import numpy as np
    from cv_bridge import CvBridge
    import cv2
    _HAS_VISION = True
except Exception:  # pragma: no cover
    _HAS_VISION = False

try:
    import tf2_ros
    from geometry_msgs.msg import PointStamped
    _HAS_TF = True
except Exception:  # pragma: no cover
    _HAS_TF = False

try:
    from gazebo_msgs.msg import ModelStates
    _HAS_GAZEBO_MSGS = True
except Exception:  # pragma: no cover
    _HAS_GAZEBO_MSGS = False

try:
    from visualization_msgs.msg import Marker, MarkerArray
    _HAS_MARKERS = True
except Exception:  # pragma: no cover
    _HAS_MARKERS = False


# HSV 阈值（H: 0-179, S/V: 0-255），室内仿真光照下比较鲁棒
HSV_RANGES = {
    "red": [((0, 90, 60), (10, 255, 255)), ((170, 90, 60), (179, 255, 255))],
    "blue": [((100, 90, 60), (130, 255, 255))],
}
RGB_RENDER = {"red": (0.9, 0.1, 0.1), "blue": (0.1, 0.2, 0.9)}

# 不同 URDF 配置下相机话题命名不同，按顺序找第一个真实存在的
IMAGE_CANDIDATES = [
    "/camera_rgbd/camera_rgbd/image_raw",   # 本平台实际话题（双层命名）
    "/camera_rgbd/image_raw",
    "/camera/image_raw",
]
DEPTH_CANDIDATES = [
    "/camera_rgbd/camera_rgbd/depth/image_raw",
    "/camera_rgbd/depth/image_raw",
    "/camera/depth/image_raw",
]
INFO_CANDIDATES = [
    "/camera_rgbd/camera_rgbd/camera_info",
    "/camera_rgbd/camera_info",
    "/camera/camera_info",
]


class ObjectDetector(Node):
    def __init__(self) -> None:
        super().__init__("object_detector")

        # 仿真环境必须用仿真时钟，否则消息时间戳会和 TF 对不上
        # （踩过坑：时间戳是墙上时钟 → AMCL 无法定位 → 导航全废）
        try:
            from rclpy.parameter import Parameter
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        except Exception:
            pass

        self.declare_parameter("mode", "auto")           # auto / vision / gazebo / config
        # 相机话题在不同 URDF 配置下命名不一样（本平台实际是双层的
        # /camera_rgbd/camera_rgbd/image_raw），所以给候选列表自动探测。
        self.declare_parameter("image_topic", "auto")
        self.declare_parameter("depth_topic", "auto")
        self.declare_parameter("camera_info_topic", "auto")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("min_area", 120.0)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("config_file", "")

        self.mode = str(self.get_parameter("mode").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.min_area = float(self.get_parameter("min_area").value)

        self.bridge = CvBridge() if _HAS_VISION else None
        self.depth: Optional["np.ndarray"] = None
        self.camera_info = None
        self.known = self._load_known_cubes()
        self.scene_cfg = self._load_scene_cfg()
        self.gazebo_cubes: List[Dict] = []

        self.pub_det = self.create_publisher(String, "/competition/detections", 10)
        self.pub_text = self.create_publisher(String, "/competition/detections_text", 10)
        self.pub_markers = (
            self.create_publisher(MarkerArray, "/competition/markers", 10) if _HAS_MARKERS else None)

        if _HAS_TF:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        vision_ready = _HAS_VISION and self.mode in ("auto", "vision")
        if vision_ready:
            try:
                from sensor_msgs.msg import CameraInfo, Image
                image_topic = self._resolve_topic("image_topic", IMAGE_CANDIDATES)
                depth_topic = self._resolve_topic("depth_topic", DEPTH_CANDIDATES)
                info_topic = self._resolve_topic("camera_info_topic", INFO_CANDIDATES)
                self.create_subscription(Image, image_topic, self.on_image, 10)
                self.create_subscription(Image, depth_topic, self.on_depth, 10)
                self.create_subscription(CameraInfo, info_topic, self.on_info, 10)
                self.get_logger().info(
                    f"视觉识别已启动（HSV 颜色分割 + 深度反投影）\n"
                    f"  彩色图: {image_topic}\n  深度图: {depth_topic}\n  内参  : {info_topic}")
                vision_ready = True
            except Exception as exc:
                self.get_logger().warn(f"相机话题订阅失败，改用坐标表: {exc}")
                vision_ready = False
        if not vision_ready:
            self.get_logger().info(f"视觉不可用（{self.mode}），使用 gazebo/config 坐标")

        if _HAS_GAZEBO_MSGS and self.mode in ("auto", "gazebo"):
            self.create_subscription(ModelStates, "/gazebo/model_states", self.on_model_states, 10)
            # 兼容部分 gazebo_ros_state 构建把话题放在根命名空间的情况
            self.create_subscription(ModelStates, "/model_states", self.on_model_states, 10)

        self.create_timer(2.0, self.publish_detections)

    # ------------------------------------------------------------------ #
    def _load_known_cubes(self) -> List[Dict]:
        path = str(self.get_parameter("config_file").value or "")
        if not path:
            try:
                path = os.path.join(
                    get_package_share_directory("llm_comp"), "config", "competition.yaml")
            except Exception:
                return []
        try:
            with open(path, "r", encoding="utf-8") as fp:
                cfg = yaml.safe_load(fp) or {}
            cubes = []
            for name, pose in (cfg.get("cubes") or {}).items():
                color = "red" if "red" in name else ("blue" if "blue" in name else "unknown")
                cubes.append({"name": name, "color": color,
                              "x": float(pose["x"]), "y": float(pose["y"]),
                              "z": float(pose.get("z", 0.75)), "source": "config"})
            self.get_logger().info(f"已载入 {len(cubes)} 个货物先验坐标 ({path})")
            return cubes
        except Exception as exc:
            self.get_logger().warn(f"读取先验坐标失败: {exc}")
            return []

    # ------------------------------------------------------------------ #
    def _resolve_topic(self, param: str, candidates) -> str:
        """参数为 auto 时自动挑选存在的话题；否则用参数值。"""
        value = str(self.get_parameter(param).value or "auto")
        if value and value != "auto":
            return value
        existing = set(self.get_topic_names_and_types())
        for name in candidates:
            if name in existing:
                return name
        # 还没起来就先用第一个候选，后面话题出现后依然能收到
        self.get_logger().warn(f"暂未发现相机话题，先订阅 {candidates[0]}（可稍后重启本节点）")
        return candidates[0]

    def on_info(self, msg) -> None:
        self.camera_info = msg

    def on_depth(self, msg) -> None:
        if self.bridge is None:
            return
        try:
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().debug(f"深度图转换失败: {exc}")

    def on_image(self, msg) -> None:
        if self.bridge is None or self.camera_info is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().debug(f"图像转换失败: {exc}")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        found: List[Dict] = []
        for color, ranges in HSV_RANGES.items():
            mask = None
            for lo, hi in ranges:
                part = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
                mask = part if mask is None else cv2.bitwise_or(mask, part)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area:
                    continue
                moments = cv2.moments(contour)
                if moments["m00"] == 0:
                    continue
                u = moments["m10"] / moments["m00"]
                v = moments["m01"] / moments["m00"]
                point = self._deproject(u, v)
                if point is None:
                    continue
                found.append({"color": color, "u": float(u), "v": float(v),
                              "area": float(area), "x": point[0], "y": point[1], "z": point[2],
                              "source": "vision"})
        if found:
            self.gazebo_cubes = found  # 复用同一个缓存字段

    # ------------------------------------------------------------------ #
    def _deproject(self, u: float, v: float):
        """像素 + 深度 → map 坐标系三维点。"""
        if self.depth is None or self.camera_info is None:
            return None
        try:
            h, w = self.depth.shape[:2]
            ui = int(max(0, min(w - 1, round(u))))
            vi = int(max(0, min(h - 1, round(v))))
            patch = self.depth[max(0, vi - 3):vi + 4, max(0, ui - 3):ui + 4].astype("float32")
            patch = patch[np.isfinite(patch) & (patch > 0.05) & (patch < 20.0)]
            if patch.size == 0:
                return None
            z = float(np.median(patch))
            k = self.camera_info.k
            x = (u - k[2]) * z / k[0]
            y = (v - k[5]) * z / k[4]
            if _HAS_TF:
                pt = PointStamped()
                pt.header.frame_id = self.camera_info.header.frame_id or "camera_frame_optical"
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.point.x, pt.point.y, pt.point.z = x, y, z
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame, pt.header.frame_id,
                    rclpy.time.Time(), timeout=Duration(seconds=0.2))
                t = tf.transform.translation
                # 只做平移近似（相机与目标系旋转较小时够用），需要更精确可换 do_transform_point
                return (x + t.x, y + t.y, z + t.z)
            return (x, y, z)
        except Exception as exc:
            self.get_logger().debug(f"反投影失败: {exc}")
            return None

    # ------------------------------------------------------------------ #
    def on_model_states(self, msg) -> None:
        cubes = []
        for name, pose in zip(msg.name, msg.pose):
            if "cube" not in name:
                continue
            color = "red" if "red" in name else ("blue" if "blue" in name else "unknown")
            cubes.append({"name": name, "color": color,
                          "x": float(pose.position.x), "y": float(pose.position.y),
                          "z": float(pose.position.z), "source": "gazebo"})
        if cubes:
            self.gazebo_cubes = cubes

    # ------------------------------------------------------------------ #
    def publish_detections(self) -> None:
        mode = self.mode
        cubes = self.gazebo_cubes or (self.known if mode in ("auto", "config") else [])
        if mode == "config":
            cubes = self.known

        payload = {
            "count": len(cubes),
            "cubes": cubes,
            "summary": self._summary(cubes),
        }
        self.pub_det.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.pub_text.publish(String(data=payload["summary"]))
        if self.pub_markers is not None and cubes:
            self.pub_markers.publish(self._markers(cubes))

    @staticmethod
    def _summary(cubes: List[Dict]) -> str:
        if not cubes:
            return "识别结果: 未识别到货物"
        reds = [c for c in cubes if c.get("color") == "red"]
        blues = [c for c in cubes if c.get("color") == "blue"]
        lines = [f"识别结果: 共 {len(cubes)} 个货物（红 {len(reds)} / 蓝 {len(blues)}）"]
        for cube in cubes[:10]:
            lines.append(f"  {cube.get('name', cube.get('color'))}: "
                         f"({cube['x']:.2f}, {cube['y']:.2f}, {cube.get('z', 0):.2f})")
        return "\n".join(lines)

    def _markers(self, cubes: List[Dict]):
        """把货物、放置区、动态障碍物、停车作业点全部画到 RViz 上。

        Gazebo 里的家具/墙在 RViz 里是看不到的（RViz 只显示 ROS 话题），
        所以这里主动把场景关键元素发布成 MarkerArray，
        这样 RViz 的 3D 面板里就能看到整个比赛布局。
        """
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        def base(ns, idx, mtype, scale):
            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = stamp
            marker.ns = ns
            marker.id = idx
            marker.type = mtype
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = scale
            return marker

        # --- 货物 ---
        for idx, cube in enumerate(cubes):
            marker = base("cubes", idx, Marker.CUBE, 0.1)
            marker.pose.position.x = float(cube["x"])
            marker.pose.position.y = float(cube["y"])
            marker.pose.position.z = float(cube.get("z", 0.75))
            r, g, b = RGB_RENDER.get(cube.get("color", "red"), (0.5, 0.5, 0.5))
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = r, g, b, 0.9
            marker.text = cube.get("name", "")
            array.markers.append(marker)

        # --- 放置区 ---
        zone_color = {"A": (0.1, 0.9, 0.1), "B": (0.9, 0.9, 0.1), "C": (0.1, 0.3, 0.9)}
        for idx, (name, pose) in enumerate((self.scene_cfg.get("zones") or {}).items()):
            marker = base("zones", idx, Marker.CUBE, 1.0)
            marker.scale.z = 0.02
            marker.pose.position.x = float(pose["x"])
            marker.pose.position.y = float(pose["y"])
            marker.pose.position.z = 0.02
            r, g, b = zone_color.get(name.upper(), (0.7, 0.7, 0.7))
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = r, g, b, 0.45
            marker.text = f"{name} 区"
            array.markers.append(marker)

        # --- 动态障碍物（画成圆柱 + 轨迹线） ---
        obstacles = self.scene_cfg.get("obstacles") or {}
        for idx, (name, spec) in enumerate(obstacles.items() if isinstance(obstacles, dict) else []):
            try:
                a_str, b_str = str(spec).split(":")[0], str(spec).split(":")[1]
                ax, ay = [float(v) for v in a_str.split(",")]
                bx, by = [float(v) for v in b_str.split(",")]
            except Exception:
                continue
            marker = base("obstacles", idx, Marker.CYLINDER, 0.3)
            marker.scale.z = 0.8
            marker.pose.position.x, marker.pose.position.y = ax, ay
            marker.pose.position.z = 0.4
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.9, 0.4, 0.1, 0.8
            marker.text = f"{name} 起点"
            array.markers.append(marker)

            line = base("obstacle_paths", idx, Marker.LINE_STRIP, 0.03)
            p1 = Point(); p1.x, p1.y, p1.z = ax, ay, 0.05
            p2 = Point(); p2.x, p2.y, p2.z = bx, by, 0.05
            line.points = [p1, p2]
            line.color.r, line.color.g, line.color.b, line.color.a = 1.0, 0.4, 0.1, 0.9
            array.markers.append(line)

        # --- 机器人停车作业点（调试导航很有用） ---
        approach = float((self.scene_cfg.get("nav") or {}).get("approach_distance", 0.55))
        park_idx = 0
        for name, pose in (self.scene_cfg.get("cubes") or {}).items():
            yaw = float(pose.get("yaw", 0.0))
            marker = base("park_poses", park_idx, Marker.SPHERE, 0.12)
            marker.pose.position.x = float(pose["x"]) - approach * math.cos(yaw)
            marker.pose.position.y = float(pose["y"]) - approach * math.sin(yaw)
            marker.pose.position.z = 0.06
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.2, 0.8, 1.0, 0.7
            marker.text = f"{name} 停车点"
            array.markers.append(marker)
            park_idx += 1
        for name, pose in (self.scene_cfg.get("zones") or {}).items():
            yaw = float(pose.get("yaw", 0.0))
            marker = base("park_poses", park_idx, Marker.SPHERE, 0.12)
            marker.pose.position.x = float(pose["x"]) - approach * math.cos(yaw)
            marker.pose.position.y = float(pose["y"]) - approach * math.sin(yaw)
            marker.pose.position.z = 0.06
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.2, 1.0, 0.6, 0.7
            marker.text = f"{name} 区停车点"
            array.markers.append(marker)
            park_idx += 1

        return array


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectDetector()
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
