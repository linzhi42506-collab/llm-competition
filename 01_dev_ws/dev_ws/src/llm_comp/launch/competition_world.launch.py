"""启动 Gazebo 比赛场景（competition.world）。

与官方 `mybot gazebo_world.launch.py` 完全一致，只是把世界文件换成经过
「场景设计」改造的 competition.world（可用 world:= 参数切回 room.world）。

先执行 `06_脚本/生成比赛场景.py` 生成 competition.world。

    ros2 launch llm_comp competition_world.launch.py
    ros2 launch llm_comp competition_world.launch.py world:=room.world   # 用回官方原始场景
"""

import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def remove_comments(text: str) -> str:
    return re.sub(r"<!--(.*?)-->", "", text, flags=re.DOTALL)


def generate_launch_description() -> LaunchDescription:
    robot_name_in_model = "six_arm"
    package_name = "mybot_description"
    urdf_name = "originbot_with_rgbd_gazebo_arm.xacro"

    pkg_share = get_package_share_directory(package_name)
    urdf_model_path = os.path.join(pkg_share, "urdf", urdf_name)

    world_arg = DeclareLaunchArgument("world", default_value="competition.world",
                                      description="mybot_description/worlds 下的世界文件名")

    start_gazebo_cmd = ExecuteProcess(
        cmd=["gazebo", "--verbose",
             [os.path.join(pkg_share, "worlds"), "/", LaunchConfiguration("world")],
             "-s", "libgazebo_ros_init.so",
             "-s", "libgazebo_ros_factory.so"],
             # 说明: /gazebo/model_states 与 /gazebo/set_entity_state 由 world 插件
             # libgazebo_ros_state.so 提供，已写进 competition.world（不能用 -s 加载）
        output="screen")

    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)
    params = {"robot_description": remove_comments(doc.toxml())}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True}, params, {"publish_frequency": 15.0}],
        output="screen")

    spawn_entity_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", robot_name_in_model, "-topic", "robot_description"],
        output="screen")

    load_joint_state_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_state_broadcaster"], output="screen")
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "arm_controller"], output="screen")
    load_gripper_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "gripper_controller"], output="screen")

    return LaunchDescription([
        world_arg,
        RegisterEventHandler(OnProcessExit(target_action=spawn_entity_cmd,
                                           on_exit=[load_joint_state_controller])),
        RegisterEventHandler(OnProcessExit(target_action=load_joint_state_controller,
                                           on_exit=[load_joint_trajectory_controller])),
        RegisterEventHandler(OnProcessExit(target_action=load_joint_trajectory_controller,
                                           on_exit=[load_gripper_controller])),
        start_gazebo_cmd,
        node_robot_state_publisher,
        spawn_entity_cmd,
    ])
