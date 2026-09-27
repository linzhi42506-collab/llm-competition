"""大模型指令解析节点（对应评分项 2「指令解析」10 分）。

流程::

    /competition/question  ──►  LLM（本地 llama-server 或云端 API）
                              │
                              ├─► /competition/task_plan   "[red,4,A;blue,1,C]"
                              ├─► /competition/task_info   JSON（题目/数量/区域/来源）
                              └─► /competition/task_info_text  面板用中文摘要

大模型答不出来（断网、欠费、小模型算错）时自动切换规则兜底，保证解析结果正确。
"""

from __future__ import annotations

import json
import os
from typing import Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from llm_comp.llm_client import LLMClient
from llm_comp.task_solver import TaskPlan, format_plan, solve

COLOR_CN = {"red": "红色", "blue": "蓝色", "green": "绿色", "yellow": "黄色"}


class LLMTaskParser(Node):
    def __init__(self) -> None:
        super().__init__("llm_task_parser")

        # 仿真环境必须用仿真时钟，否则消息时间戳会和 TF 对不上
        # （踩过坑：时间戳是墙上时钟 → AMCL 无法定位 → 导航全废）
        try:
            from rclpy.parameter import Parameter
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        except Exception:
            pass

        self.declare_parameter("use_llm", True)
        self.declare_parameter("provider", "")
        self.declare_parameter("base_url", "")
        self.declare_parameter("model", "")
        self.declare_parameter("api_key", "")
        self.declare_parameter("timeout", 0.0)
        self.declare_parameter("config_file", "")
        self.declare_parameter("question", "")  # 直接给文本则无需出题程序
        self.declare_parameter("retry_with_rule", True)

        cfg = self._load_config()
        llm_cfg = cfg.get("llm", {})
        provider = self._param_or("provider", llm_cfg.get("provider", "local"))
        base_url = self._param_or("base_url", llm_cfg.get("base_url", ""))
        model = self._param_or("model", llm_cfg.get("model", ""))
        api_key = self._param_or("api_key", llm_cfg.get("api_key", ""))
        timeout = self._param_or("timeout", llm_cfg.get("timeout", 30.0), cast=float)

        self.use_llm = bool(self.get_parameter("use_llm").value)
        self.client: Optional[LLMClient] = None
        if self.use_llm:
            try:
                self.client = LLMClient(
                    provider=provider, base_url=base_url or None, model=model or None,
                    api_key=api_key or None, timeout=timeout or 30.0,
                    temperature=llm_cfg.get("temperature", 0.1),
                    max_tokens=llm_cfg.get("max_tokens", 512),
                )
                self.get_logger().info(f"大模型配置: {self.client.describe()}")
                if self.client.is_cloud and not self.client.api_key:
                    self.get_logger().warn(
                        f"未检测到 {provider} 的 API Key（环境变量 {os.environ.get('LLM_API_KEY_ENV', '')} 或参数 api_key），"
                        "将自动使用规则兜底")
            except Exception as exc:
                self.get_logger().error(f"大模型客户端初始化失败: {exc}；改用规则兜底")

        self.pub_plan = self.create_publisher(String, "/competition/task_plan", 10)
        self.pub_info = self.create_publisher(String, "/competition/task_info", 10)
        self.pub_text = self.create_publisher(String, "/competition/task_info_text", 10)
        self.create_subscription(String, "/competition/question", self.on_question, 10)

        self.last: Optional[TaskPlan] = None
        inline = str(self.get_parameter("question").value or "")
        if inline:
            self.get_logger().info("检测到 question 参数，直接解析")
            self.create_timer(1.0, lambda: self._parse_once(inline))

    # ------------------------------------------------------------------ #
    def _param_or(self, name: str, default, cast=str):
        value = self.get_parameter(name).value
        if value in (None, "", 0.0, 0):
            return default
        try:
            return cast(value)
        except Exception:
            return default

    def _load_config(self) -> dict:
        path = str(self.get_parameter("config_file").value or "")
        if not path:
            try:
                path = os.path.join(
                    get_package_share_directory("llm_comp"), "config", "llm_config.yaml")
            except Exception:
                path = ""
        if path and os.path.isfile(path):
            try:
                with open(path, "r", encoding="utf-8") as fp:
                    data = yaml.safe_load(fp) or {}
                self.get_logger().info(f"已加载配置: {path}")
                return data
            except Exception as exc:
                self.get_logger().warn(f"配置文件解析失败 {path}: {exc}")
        return {}

    # ------------------------------------------------------------------ #
    def on_question(self, msg: String) -> None:
        self.get_logger().info(f"收到题目: {msg.data[:120]}...")
        self._parse_once(msg.data)

    def _parse_once(self, text: str) -> None:
        plan = solve(text, client=self.client, use_llm=self.client is not None)
        self.last = plan

        structured = format_plan(plan.items)
        info = {
            "question": plan.question,
            "rule": plan.rule,
            "x": plan.x,
            "y": plan.y,
            "structured": structured,
            "source": plan.source,
            "note": plan.note,
            "raw_llm": plan.raw_llm.strip()[:300],
            "total": plan.total,
            "plan": plan.color_zone(),
        }
        self.pub_plan.publish(String(data=structured))
        self.pub_info.publish(String(data=json.dumps(info, ensure_ascii=False)))

        lines = ["【任务信息】", f"题目: {plan.question}", f"规则: {plan.rule}",
                 f"大模型解: x={plan.x}, y={plan.y}"]
        for item in plan.items:
            lines.append(f"  {COLOR_CN.get(item.color, item.color)} x{item.count} → {item.zone} 区")
        lines.append(f"结构化指令: {structured}")
        lines.append(f"解析来源: {plan.source} {plan.note}")
        self.pub_text.publish(String(data="\n".join(lines)))

        self.get_logger().info(f"解析完成 [{plan.source}] {structured} ({plan.note})")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LLMTaskParser()
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
