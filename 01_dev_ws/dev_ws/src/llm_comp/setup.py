import os
from glob import glob

from setuptools import find_packages, setup

package_name = "llm_comp"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "doc"), glob("doc/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="llm_comp team",
    maintainer_email="team@example.com",
    description="大模型技术创新赛参赛功能包",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "question_generator = llm_comp.question_generator:main",
            "llm_task_parser = llm_comp.llm_task_parser:main",
            "task_manager = llm_comp.task_manager:main",
            "object_detector = llm_comp.object_detector:main",
            "dynamic_obstacles = llm_comp.dynamic_obstacles:main",
            "panel_node = llm_comp.panel_node:main",
            "record_positions = llm_comp.record_positions:main",
            "auto_mapper = llm_comp.auto_mapper:main",
            "odom_tf_broadcaster = llm_comp.odom_tf_broadcaster:main",
            "auto_initial_pose = llm_comp.auto_initial_pose:main",
            "task_solver_cli = llm_comp.task_solver:cli_main",
        ],
    },
)
