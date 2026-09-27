# 大模型技术创新赛 · 参赛作品

全球校园人工智能算法精英大赛 · 算法应用赛 · **大模型技术创新赛**

基于 **ROS2 + Gazebo + Nav2 + MoveIt2 + 大语言模型** 的仓储分拣搬运智能系统：
大模型解析自然语言题目 → 输出结构化抓取指令 → 机器人自主导航避障 →
6 自由度机械臂抓取搬运放置 → 可视化面板实时反馈。

> 📖 **完整说明请看 [`README_先看我.md`](README_先看我.md)**
> 📋 **相对官方原始版本改了什么：[`05_文档/10_完整变更清单（对比官方原始版本）.md`](05_文档/10_完整变更清单（对比官方原始版本）.md)**

---

## 比赛要求 → 本作品对应实现

| 评分项 | 分值 | 对应节点 |
|--------|------|----------|
| 系统启动 | 10 | `competition_world.launch.py`（Gazebo 场景 + 机器人 + 传感器） |
| **指令解析** | 10 | `llm_task_parser` + `task_solver`（大模型优先，规则兜底） |
| **货物抓取** | 20 | `task_manager` + `/arm_controller` + `/ATTACHLINK` |
| **货物搬运放置** | 20 | `task_manager` + Nav2 + `/DETACHLINK` |
| 机器人避障 | 10 | `dynamic_obstacles`（2 个可移动障碍物 + 安全停让） |
| **状态反馈** | 10 | `panel_node` → `/competition/panel_text` → rosbridge → Foxglove |
| 场景设计 | 10 | `生成比赛场景.py`（货物间距 2.2m + 2 个有效障碍物，只增不删） |
| 技术文档 | 20 | `05_文档/04_技术文档（提交用草稿）.md` |

---

## 快速开始

```bash
# 1. 加载环境（隔离系统 ROS2，激活独立环境）
source 06_脚本/env.sh

# 2. 编译工作空间
./06_脚本/00_编译工作空间.sh

# 3. 一键启动全部（仿真 → 导航 → 大模型解析 → 调度 → 面板 → 障碍物）
./06_脚本/10_一键启动全部.sh
```

可视化面板连接 `ws://127.0.0.1:9090`（Foxglove / coStudio）。

---

## 目录结构

```
├── README_先看我.md          总说明（先看这个）
├── 00_原始资料/              官方 PDF 与原始压缩包
├── 01_dev_ws/dev_ws/src/
│   ├── llm_comp/             ★ 本次比赛新写的功能包（12 个节点）
│   └── yzbot/                官方仿真平台（仅改动 1 行，见变更清单）
├── 05_文档/                  10 份中文文档（含提交用技术文档草稿）
├── 06_脚本/                  16 个工具脚本
└── .gitignore                排除大件依赖（11GB conda 环境等）
```

### `llm_comp` 功能包核心节点

| 节点 | 作用 |
|------|------|
| `llm_task_parser` | 接收题目 → 大模型解题 → 发出 `[red,4,A;blue,1,C]` |
| `task_solver` | 题目求解 + 映射规则解析 + **规则兜底**（断网也能答对） |
| `task_manager` | 状态机：导航→抓取→搬运→放置（含卡死检测与自动脱困） |
| `object_detector` | HSV 颜色分割 + RGB-D 反投影货物识别 |
| `dynamic_obstacles` | 可移动障碍物驱动（含安全停让，防止撞翻机器人） |
| `panel_node` | 四类必显信息聚合成单个面板话题 |
| `auto_initial_pose` | 自动设置 AMCL 初始位姿 |
| `odom_tf_broadcaster` | 补上平台缺失的 `odom→base_footprint` 变换 |

---

## 环境说明

本仓库只含**源码与文档**。运行需要 ROS2 Humble + Gazebo Classic 11 + Nav2 + MoveIt2，
可用官方比赛 U 盘环境，或用 `05_文档/01_环境搭建与配置记录.md` 里的
RoboStack（conda）方案搭建**无需 sudo** 的独立环境。

依赖未纳入仓库：
- 独立 ROS2 环境（11 GB）
- llama.cpp 与本地模型（809 MB）
- Foxglove Studio（487 MB）

---

## ⚠️ 使用注意

- 本仓库若为 **public**，其他参赛队可见你的完整方案，建议设为 **private**。
- 提交比赛材料前请确认 `06_脚本/api_keys.sh`（云端大模型密钥）**没有**被提交。
