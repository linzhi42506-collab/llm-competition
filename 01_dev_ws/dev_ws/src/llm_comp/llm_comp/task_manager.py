"""任务调度节点：把大模型解析出的抓取计划变成机器人动作。

状态机::

    IDLE ─► 解析等待 ─► [每个货物] 前往货物 ─► 识别 ─► 抓取(吸附) ─► 前往放置区 ─► 放置(释放)
                                                        └──────────► 下一个货物 ──► 完成

对应评分项：
* 3 货物抓取（20 分）—— 机械臂成功抓取；
* 4 货物搬运放置（20 分）—— 抓取后放到指定区域；
* 6 状态反馈（10 分）—— 每一步都往 ``/competition/task_status`` 发布中文状态。

导航用 Nav2（nav2_simple_commander），机械臂用 ``/arm_controller``、
``/gripper_controller`` 的 FollowJointTrajectory 动作，吸附/释放用
``linkattacher_msgs`` 的 ``/ATTACHLINK`` ``/DETACHLINK`` 服务。

``dry_run:=true`` 时不做任何真实动作，只按时间推进状态机 —— 用来单独验证
可视化面板与状态反馈是否正常（赛前彩排非常有用）。
"""

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

    _HAS_NAV2 = True
except Exception:  # pragma: no cover
    _HAS_NAV2 = False

try:
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    from trajectory_msgs.msg import JointTrajectoryPoint

    _HAS_ARM = True
except Exception:  # pragma: no cover
    _HAS_ARM = False

try:
    from linkattacher_msgs.srv import AttachLink, DetachLink

    _HAS_LINK = True
except Exception:  # pragma: no cover
    _HAS_LINK = False

COLOR_CN = {"red": "红色", "blue": "蓝色"}

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER_JOINT = ["finger_joint1"]

# 机械臂姿态（关节角度，单位弧度）——先给一组"参考位姿"，
# 需要在 Gazebo 里对着实际货物位置标定后写进 config/competition.yaml。
DEFAULT_POSES = {
    "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "pre_grasp": [0.0, 0.9, 1.17, 0.0, -0.3, 0.0],
    "grasp": [0.0, 1.2, 1.17, 0.0, -0.3, 0.0],
    "carry": [0.0, 0.6, 0.9, 0.0, -0.2, 0.0],
    "place": [0.0, 0.5, 0.8, 0.0, -0.1, 0.0],
}
GRIPPER_OPEN = 0.3
GRIPPER_CLOSE = 0.0


@dataclass
class Job:
    """一个待执行的抓取-放置任务。"""

    color: str
    zone: str
    cube_name: str
    pick: Dict[str, float]
    place: Dict[str, float]
    index: int = 0


class TaskManager(Node):
    def __init__(self) -> None:
        super().__init__("task_manager")

        # 仿真环境必须用仿真时钟，否则消息时间戳会和 TF 对不上
        # （踩过坑：时间戳是墙上时钟 → AMCL 无法定位 → 导航全废）
        try:
            from rclpy.parameter import Parameter
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        except Exception:
            pass

        self.declare_parameter("dry_run", False)
        self.declare_parameter("config_file", "")
        self.declare_parameter("robot_name", "six_arm")
        self.declare_parameter("grasp_link", "link6")
        self.declare_parameter("cube_link", "link")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("nav_timeout", 90.0)

        cfg = self._load_config()
        self.scene = cfg
        self.poses = {**DEFAULT_POSES, **(cfg.get("arm_poses") or {})}
        self.zones: Dict[str, Dict[str, float]] = cfg.get("zones") or {}
        self.cube_table: Dict[str, Dict[str, float]] = cfg.get("cubes") or {}
        self.approach = float((cfg.get("nav") or {}).get("approach_distance", 0.55))

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.robot_name = str(self.get_parameter("robot_name").value)
        self.grasp_link = str(self.get_parameter("grasp_link").value)
        self.cube_link = str(self.get_parameter("cube_link").value)
        self.nav_timeout = float(self.get_parameter("nav_timeout").value)

        # -- 可视化面板相关发布 ------------------------------------------ #
        self.pub_status = self.create_publisher(String, "/competition/task_status", 10)
        # 用于卡死时手动后退脱困
        try:
            from geometry_msgs.msg import Twist
            self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        except Exception:
            self.pub_cmd = None
        self.pub_status_text = self.create_publisher(String, "/competition/task_status_text", 10)

        # -- 计划输入 ---------------------------------------------------- #
        self.create_subscription(String, "/competition/task_plan", self.on_plan, 10)
        self.create_subscription(String, "/competition/task_info", self.on_info, 10)

        # -- 机械臂 / 夹爪 / 吸附 ---------------------------------------- #
        self.arm_client = None
        self.gripper_client = None
        if _HAS_ARM and not self.dry_run:
            self.arm_client = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
            self.gripper_client = ActionClient(self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        self.attach_client = None
        self.detach_client = None
        if _HAS_LINK and not self.dry_run:
            self.attach_client = self.create_client(AttachLink, "/ATTACHLINK")
            self.detach_client = self.create_client(DetachLink, "/DETACHLINK")

        # -- 导航 -------------------------------------------------------- #
        self.navigator = None
        if _HAS_NAV2 and not self.dry_run:
            try:
                self.navigator = BasicNavigator()
            except Exception as exc:
                self.get_logger().error(f"Nav2 初始化失败: {exc}")
        elif not _HAS_NAV2:
            self.get_logger().warn("未检测到 nav2_simple_commander，导航功能不可用（可先用 dry_run 验证流程）")

        # -- 状态 -------------------------------------------------------- #
        self.plan_items: List[Dict] = []
        self.jobs: List[Job] = []
        self.job_index = 0
        self.state = "空闲"
        self.detail = "等待任务"
        self.started = False
        self.finished = False
        self.t0 = time.time()
        self.used_cubes: List[str] = []
        self.question = ""

        self.create_timer(1.0, self.tick)
        self.publish_status()
        self.get_logger().info(
            f"任务调度节点就绪（dry_run={self.dry_run}, nav2={'可用' if self.navigator else '不可用'}）")

    # ------------------------------------------------------------------ #
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
        except Exception as exc:
            self.get_logger().warn(f"场景配置读取失败 {path}: {exc}")
            return {}

    # ------------------------------------------------------------------ #
    def on_info(self, msg: String) -> None:
        try:
            info = json.loads(msg.data)
        except Exception:
            return
        if not self.plan_items:
            self.plan_items = info.get("plan", []) or []
        self.question = info.get("question", "")

    def on_plan(self, msg: String) -> None:
        """收到形如 ``[red,4,A;blue,1,C]`` 的计划，展开成 job 列表。"""
        if self.started:
            self.get_logger().warn("任务已开始，忽略新的计划")
            return
        text = msg.data.strip()
        self.get_logger().info(f"收到抓取计划: {text}")
        items = self._parse_plan_text(text)
        if not items:
            self.get_logger().error(f"计划解析失败: {text}")
            return
        self.jobs = self._build_jobs(items)
        self.get_logger().info(f"共生成 {len(self.jobs)} 个抓取-放置动作")
        if bool(self.get_parameter("auto_start").value) and self.jobs:
            self.start()

    @staticmethod
    def _parse_plan_text(text: str) -> List[Dict]:
        import re

        items: List[Dict] = []
        match = re.search(r"\[\s*([^\]]+?)\s*\]", text)
        body = match.group(1) if match else text
        for chunk in body.split(";"):
            parts = [p.strip() for p in chunk.split(",")]
            if len(parts) < 3:
                continue
            color = parts[0].lower()
            color = {"红": "red", "红色": "red", "蓝": "blue", "蓝色": "blue"}.get(color, color)
            try:
                count = int(re.sub(r"\D", "", parts[1]) or 0)
            except Exception:
                continue
            zone_match = re.search(r"[ABCabc]", parts[2])
            if count > 0 and zone_match:
                items.append({"color": color, "count": count, "zone": zone_match.group(0).upper()})
        return items

    def _build_jobs(self, items: List[Dict]) -> List[Job]:
        """把「红4个去A区」展开成 4 个具体动作，并分配货物实例。"""
        jobs: List[Job] = []
        used: set = set()
        for item in items:
            color = item["color"]
            zone = item["zone"]
            candidates = [n for n in self.cube_table if color in n and n not in used]
            candidates.sort(key=lambda n: int(n.rsplit("_", 1)[-1]) if n.rsplit("_", 1)[-1].isdigit() else 0)
            for i in range(int(item["count"])):
                cube = candidates[i] if i < len(candidates) else f"{color}_cube_{i + 1}"
                used.add(cube)
                jobs.append(Job(
                    color=color, zone=zone, cube_name=cube,
                    pick=self._pick_pose(cube),
                    place=self._place_pose(zone),
                    index=len(jobs) + 1,
                ))
        return jobs

    def _pick_pose(self, cube_name: str) -> Dict[str, float]:
        pose = dict(self.cube_table.get(cube_name, {"x": -1.5, "y": 1.0, "yaw": 0.0}))
        return pose

    def _place_pose(self, zone: str) -> Dict[str, float]:
        pose = dict(self.zones.get(zone, {"x": 3.0, "y": 0.0, "yaw": 0.0}))
        return pose

    # ------------------------------------------------------------------ #
    def start(self) -> None:
        self.started = True
        self.t0 = time.time()
        self.job_index = 0
        self.state = "任务开始"
        self.detail = f"共 {len(self.jobs)} 个货物待搬运"
        self.publish_status()
        if self.navigator is not None:
            try:
                self.navigator.waitUntilNav2Active()
            except Exception as exc:
                self.get_logger().warn(f"等待 Nav2 激活失败: {exc}")

    # ------------------------------------------------------------------ #
    def tick(self) -> None:
        if not self.started or self.finished:
            self.publish_status()
            return
        if self.job_index >= len(self.jobs):
            self.finished = True
            self.state = "任务完成"
            self.detail = f"已完成 {len(self.jobs)} 个货物"
            self.publish_status()
            self.get_logger().info("===== 全部任务完成 =====")
            return

        job = self.jobs[self.job_index]
        try:
            self._run_job(job)
            self.job_index += 1
        except Exception as exc:
            self.get_logger().error(f"第 {job.index} 个货物执行失败: {exc}")
            self.state = "异常"
            self.detail = str(exc)
            self.job_index += 1
            self.publish_status()

    # ------------------------------------------------------------------ #
    def _run_job(self, job: Job) -> None:
        total = len(self.jobs)
        prefix = f"第 {job.index}/{total} 个"

        self._set_state("前往资源点", f"{prefix}：前往 {COLOR_CN.get(job.color, job.color)}货物 {job.cube_name}")
        self._navigate(job.pick, f"{job.cube_name} 取货点")

        self._set_state("识别资源", f"{prefix}：识别 {job.cube_name}")
        self._sleep(0.5)

        self._set_state("机械臂抓取", f"{prefix}：机械臂下降并夹取")
        self._arm_sequence("pre_grasp")
        self._gripper(GRIPPER_CLOSE)
        self._arm_sequence("grasp")
        self._attach(job.cube_name)
        self._arm_sequence("carry")

        self._set_state("移动到放置区", f"{prefix}：搬运到 {job.zone} 区")
        self._navigate(job.place, f"{job.zone} 区放置点")

        self._set_state("机械臂放置", f"{prefix}：放置到 {job.zone} 区")
        self._arm_sequence("place")
        self._detach(job.cube_name)
        self._gripper(GRIPPER_OPEN)
        self._arm_sequence("home")

        self.used_cubes.append(job.cube_name)
        self._set_state("完成一轮任务", f"{prefix} 已完成 → {job.zone} 区")

    # ------------------------------------------------------------------ #
    def _navigate(self, pose: Dict[str, float], label: str) -> None:
        if self.navigator is None or self.dry_run:
            self._sleep(2.0)
            return
        target = PoseStamped()
        target.header.frame_id = "map"
        target.header.stamp = self.get_clock().now().to_msg()
        # 停在货物/放置区前方 approach 米处，方便机械臂够到
        yaw = float(pose.get("yaw", 0.0))
        target.pose.position.x = float(pose["x"]) - self.approach * math.cos(yaw)
        target.pose.position.y = float(pose["y"]) - self.approach * math.sin(yaw)
        target.pose.orientation.z = math.sin(yaw / 2.0)
        target.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f"导航到 {label}: ({target.pose.position.x:.2f}, {target.pose.position.y:.2f})")
        for attempt in (1, 2):
            self.navigator.goToPose(target)
            deadline = time.time() + self.nav_timeout
            # --- 卡死检测：15 秒内剩余距离没有实质减少就认为卡住了 ---
            check_t = time.time()
            best_remain = None
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback is not None:
                    remain = getattr(feedback, "distance_remaining", None)
                    if remain is not None:
                        self.detail = f"→ {label}，剩余 {remain:.2f} m"
                        self.publish_status()
                        if best_remain is None or remain < best_remain - 0.10:
                            best_remain = remain
                            check_t = time.time()
                        elif time.time() - check_t > 15.0:
                            self.get_logger().warn(f"导航到 {label} 卡住（剩余 {remain:.2f} m），取消并尝试脱困")
                            self.navigator.cancelTask()
                            self._escape()
                            break
                if time.time() > deadline:
                    self.get_logger().warn(f"导航到 {label} 超时，取消")
                    self.navigator.cancelTask()
                    break
                time.sleep(0.2)
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                return
            self.get_logger().warn(f"导航到 {label} 第 {attempt} 次未成功（{result}）")
        self.get_logger().warn(f"导航到 {label} 最终失败，跳过该动作")

    def _escape(self) -> None:
        """脱困：先后退再原地转一下，甩开卡住的姿态。"""
        if self.pub_cmd is None:
            time.sleep(1.0)
            return
        try:
            from geometry_msgs.msg import Twist
            for _ in range(15):
                m = Twist(); m.linear.x = -0.15
                self.pub_cmd.publish(m); time.sleep(0.1)
            for _ in range(20):
                m = Twist(); m.angular.z = 0.6
                self.pub_cmd.publish(m); time.sleep(0.1)
            self.pub_cmd.publish(Twist())
        except Exception as exc:
            self.get_logger().warn(f"脱困动作失败: {exc}")
        time.sleep(0.5)

    # ------------------------------------------------------------------ #
    def _arm_sequence(self, name: str) -> None:
        self._send_trajectory(self.arm_client, ARM_JOINTS, self.poses.get(name, DEFAULT_POSES["home"]), 3.0)

    def _gripper(self, position: float) -> None:
        self._send_trajectory(self.gripper_client, GRIPPER_JOINT, [position], 2.0)

    def _send_trajectory(self, client, joints: List[str], positions: List[float], sec: float) -> None:
        if client is None:
            self._sleep(sec * 0.2)
            return
        if not client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("机械臂动作服务不可用，跳过")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joints)
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = int(sec)
        point.time_from_start.nanosec = int((sec % 1) * 1e9)
        goal.trajectory.points = [point]
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn("机械臂动作被拒绝")
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=sec + 10.0)

    # ------------------------------------------------------------------ #
    def _attach(self, cube_name: str) -> None:
        self._link_service(self.attach_client, AttachLink, cube_name, "吸附")

    def _detach(self, cube_name: str) -> None:
        self._link_service(self.detach_client, DetachLink, cube_name, "释放")

    def _link_service(self, client, srv_type, cube_name: str, label: str) -> None:
        if client is None:
            self._sleep(0.3)
            return
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f"{label}服务不可用")
            return
        request = srv_type.Request()
        request.model1_name = self.robot_name
        request.link1_name = self.grasp_link
        request.model2_name = cube_name
        request.link2_name = self.cube_link
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        self.get_logger().info(f"{label} {cube_name}: {future.result()}")

    # ------------------------------------------------------------------ #
    def _sleep(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            time.sleep(0.05)

    def _set_state(self, state: str, detail: str) -> None:
        self.state = state
        self.detail = detail
        self.publish_status()
        self.get_logger().info(f"[{state}] {detail}")

    # ------------------------------------------------------------------ #
    def publish_status(self) -> None:
        done = min(self.job_index, len(self.jobs))
        payload = {
            "task_state": self.state,
            "work_state": self.detail,
            "progress_current": done,
            "progress_total": len(self.jobs),
            "progress_text": f"已完成 {done}/{len(self.jobs)} 个货物" if self.jobs else "等待任务",
            "gripper_state": self._gripper_state(),
            "current_cube": self.jobs[self.job_index].cube_name if done < len(self.jobs) else "",
            "current_zone": self.jobs[self.job_index].zone if done < len(self.jobs) else "",
            "elapsed": round(time.time() - self.t0, 1),
            "finished": self.finished,
        }
        self.pub_status.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.pub_status_text.publish(String(data=(
            f"【当前状态】{self.state}\n{self.detail}\n"
            f"进度: {payload['progress_text']}\n末端状态: {payload['gripper_state']}\n"
            f"已用时: {payload['elapsed']} s")))

    def _gripper_state(self) -> str:
        if "抓取" in self.state:
            return "夹取中"
        if "放置" in self.state:
            return "释放中"
        if "搬运" in self.state or "移动到放置区" in self.state:
            return "夹持搬运"
        return "空闲张开"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskManager()
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
