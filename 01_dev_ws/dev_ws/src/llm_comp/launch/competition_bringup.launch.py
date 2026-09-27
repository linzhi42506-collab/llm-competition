"""比赛一键启动：仿真 + 导航 + 机械臂 + 大模型解析 + 可视化。

使用::

    # 1) 全流程（正式比赛用，仿真与导航分别用官方 launch 启动更稳）
    ros2 launch llm_comp competition_bringup.launch.py

    # 2) 只启动"大脑"部分（大模型解析 + 调度 + 面板），仿真手动启动
    ros2 launch llm_comp competition_bringup.launch.py start_sim:=false start_nav:=false

    # 3) 彩排：不接真实动作，只跑状态机与面板
    ros2 launch llm_comp competition_bringup.launch.py dry_run:=true

常用参数::

    provider:=local|deepseek|zhipu|...     大模型服务商
    question:="题目:....映射规则:..."       直接喂一道题（跳过出题程序）
    use_question_generator:=true           用模拟出题程序自动出题
    start_obstacles:=true                  启动动态障碍物（避障分）
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    llm_share = get_package_share_directory("llm_comp")
    config_file = os.path.join(llm_share, "config", "competition.yaml")
    llm_config = os.path.join(llm_share, "config", "llm_config.yaml")

    args = [
        DeclareLaunchArgument("start_sim", default_value="true",
                              description="是否启动 Gazebo 仿真（mybot gazebo_world.launch.py）"),
        DeclareLaunchArgument("start_nav", default_value="true",
                              description="是否启动 Nav2 + rviz（bot_navigation）"),
        DeclareLaunchArgument("start_obstacles", default_value="true",
                              description="是否启动动态障碍物节点"),
        DeclareLaunchArgument("use_question_generator", default_value="true",
                              description="是否启动模拟出题程序"),
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="true=不执行真实动作，只跑状态机与面板"),
        DeclareLaunchArgument("provider", default_value="local",
                              description="local / deepseek / zhipu / volcengine / dashscope ..."),
        DeclareLaunchArgument("question", default_value="",
                              description="直接给一道题目文本，留空则等出题程序"),
        DeclareLaunchArgument("use_llm", default_value="true",
                              description="false=只用规则兜底解析（离线彩排）"),
        DeclareLaunchArgument("world", default_value="room.world"),
    ]

    # --------------------------- 1. Gazebo 仿真 ---------------------------- #
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("mybot"), "launch", "gazebo_world.launch.py")),
        condition=IfCondition(LaunchConfiguration("start_sim")),
    )

    # --------------------------- 2. Nav2 导航 ------------------------------ #
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("bot_navigation"), "launch",
                         "nav_bringup_gazebo.launch.py")),
        condition=IfCondition(LaunchConfiguration("start_nav")),
    )

    # --------------------------- 3. 大模型解析 ----------------------------- #
    parser = Node(
        package="llm_comp", executable="llm_task_parser", name="llm_task_parser", output="screen",
        parameters=[{"use_sim_time": True,
            "use_llm": ParameterValue(LaunchConfiguration("use_llm"), value_type=bool),
            "provider": LaunchConfiguration("provider"),
            "question": LaunchConfiguration("question"),
            "config_file": llm_config,
        }],
    )

    # --------------------------- 4. 任务调度 ------------------------------- #
    manager = Node(
        package="llm_comp", executable="task_manager", name="task_manager", output="screen",
        parameters=[{"use_sim_time": True,
                     "dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
                     "config_file": config_file}],
    )

    # --------------------------- 5. 视觉识别 ------------------------------- #
    detector = Node(
        package="llm_comp", executable="object_detector", name="object_detector", output="screen",
        parameters=[{"use_sim_time": True, "config_file": config_file, "mode": "auto"}],
    )

    # --------------------------- 6. 可视化聚合 ----------------------------- #
    panel = Node(
        package="llm_comp", executable="panel_node", name="panel_node", output="screen",
        parameters=[{"use_sim_time": True, "path_topic": "/plan", "pose_topic": "/amcl_pose"}],
    )

    # --------------------------- 7. 动态障碍物 ----------------------------- #
    obstacles = Node(
        package="llm_comp", executable="dynamic_obstacles", name="dynamic_obstacles",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_obstacles")),
    )

    # --------------------------- 8. 模拟出题程序 --------------------------- #
    generator = Node(
        package="llm_comp", executable="question_generator", name="question_generator",
        output="screen", parameters=[{"count": 1, "interval": 0.0}],
        condition=IfCondition(LaunchConfiguration("use_question_generator")),
    )

    # --------------------------- 9. rosbridge（可视化面板数据源）----------- #
    rosbridge = ExecuteProcess(
        cmd=["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"],
        output="screen",
    )

    return LaunchDescription(args + [
        sim,
        TimerAction(period=6.0, actions=[nav]),          # 等 Gazebo 起来再起导航
        TimerAction(period=10.0, actions=[
            parser, manager, detector, panel, obstacles, generator, rosbridge]),
    ])
