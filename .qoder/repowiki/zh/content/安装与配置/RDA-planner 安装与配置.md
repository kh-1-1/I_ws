# RDA-planner 安装与配置

<cite>
**本文档引用的文件**   
- [setup.py](file://RDA-planner/setup.py)
- [README.md](file://RDA-planner/README.md)
- [mpc.py](file://RDA-planner/RDA_planner/mpc.py)
- [rda_solver.py](file://RDA-planner/RDA_planner/rda_solver.py)
- [corridor.yaml](file://RDA-planner/example/corridor/corridor.yaml)
- [dynamic_obs.yaml](file://RDA-planner/example/dynamic_obs/dynamic_obs.yaml)
</cite>

## 目录
1. [简介](#简介)
2. [安装与依赖管理](#安装与依赖管理)
3. [配置文件详解](#配置文件详解)
4. [ROS集成指南](#ros集成指南)
5. [优化求解器问题排查](#优化求解器问题排查)
6. [安装验证](#安装验证)

## 简介
RDA-planner 是一种高性能的基于优化的模型预测控制（MPC）运动规划器，专为复杂和密集环境中的自主导航而设计。该规划器利用交替方向乘子法（ADMM）将复杂的优化问题分解为多个简单的子问题，从而实现并行计算，显著提高计算速度。本指南详细说明了如何安装、配置和使用 RDA-planner，包括与 ROS 环境的集成。

## 安装与依赖管理

### 使用 setup.py 安装
RDA-planner 提供了标准的 Python setuptools 安装方式。安装步骤如下：

```bash
git clone https://github.com/hanruihua/RDA_planner
cd RDA_planner
pip install -e .
```

该安装过程会自动处理所有依赖项。`setup.py` 文件中定义了以下核心依赖：

- **CVXPY (1.5.2)**: 用于凸优化问题求解的核心库
- **NumPy**: 数值计算基础库
- **Pathos**: 支持多进程并行计算
- **Matplotlib**: 可视化支持
- **GCTL (1.2)**: 图形控制库
- **OpenCV-Python**: 图像处理支持
- **ImageIO**: 图像输入输出
- **Scikit-learn 和 Scikit-image**: 机器学习和图像处理工具

**注意**: 项目要求 Python 版本 >= 3.8。

**Section sources**
- [setup.py](file://RDA-planner/setup.py#L1-L20)
- [README.md](file://RDA-planner/README.md#L0-L85)

## 配置文件详解

### MPC 控制器配置
RDA-planner 的 MPC 控制器通过 `mpc.py` 文件中的 `MPC` 类实现。控制器的关键配置参数包括：

- **receding (int, 默认=10)**: MPC 的预测时域
- **iter_num (int, 默认=4)**: MPC 的最大迭代次数
- **sample_time (float, 默认=0.1)**: 世界的时间步长
- **enable_reverse (bool, 默认=False)**: 是否允许机器人前进和后退
- **accelerated (bool, 默认=True)**: 是否使用加速 ADMM
- **process_num (int, 默认=4)**: 用于并行求解的进程数

控制器还支持动态调整参数，如权重系数 `ws`、`wu`，ADMM 惩罚参数 `ro1`、`ro2`，以及安全距离 `max_sd`、`min_sd`。

### 碰撞避免算法配置
碰撞避免算法在 `rda_solver.py` 中实现，其核心是通过 ADMM 分解优化问题。关键配置包括：

- **max_edge_num (int, 默认=5)**: 多边形障碍物考虑的最大边数
- **max_obs_num (int, 默认=5)**: 考虑的最大障碍物数量
- **obstacle_order (bool, 默认=True)**: 障碍物列表是否按与机器人的最小距离排序

**Section sources**
- [mpc.py](file://RDA-planner/RDA_planner/mpc.py#L0-L569)
- [rda_solver.py](file://RDA-planner/RDA_planner/rda_solver.py#L0-L799)

### 示例配置文件分析

#### Corridor.yaml 配置
`corridor.yaml` 配置文件定义了一个走廊场景，其主要参数包括：

- **world**: 定义世界尺寸（高度20，宽度63）、时间步长（0.1秒）和碰撞模式
- **robot**: 定义机器人运动学（阿克曼转向）、形状（长4.6宽1.6的矩形）、初始状态和目标
- **obstacle**: 定义6个矩形障碍物，包括它们的位置和方向

此配置展示了如何设置静态障碍物环境，适用于路径跟踪和走廊导航场景。

#### Dynamic_obs.yaml 配置
`dynamic_obs.yaml` 配置文件专注于动态障碍物场景，其特点包括：

- **obstacle**: 定义7个圆形动态障碍物，使用差速驱动运动学
- **behavior**: 配置障碍物行为为"dash"（冲刺），支持随机游走
- **vel_min/vel_max**: 设置动态障碍物的速度范围

此配置展示了如何模拟动态环境，测试规划器的实时避障能力。

**Section sources**
- [corridor.yaml](file://RDA-planner/example/corridor/corridor.yaml#L0-L35)
- [dynamic_obs.yaml](file://RDA-planner/example/dynamic_obs/dynamic_obs.yaml#L0-L48)

## ROS集成指南

### ROS 包装器
RDA-planner 提供了专门的 ROS 包装器，位于 [rda_ros](https://github.com/hanruihua/rda_ros) 仓库。集成步骤如下：

1. 克隆 rda_ros 仓库到您的 ROS 工作空间
2. 确保 RDA-planner 已正确安装
3. 使用 catkin_make 或 catkin build 编译工作空间

### 动态重配置参数
RDA-planner 支持通过 ROS 动态重配置（dynamic_reconfigure）实时调整参数。关键可调参数包括：

- MPC 控制器参数（预测时域、迭代次数）
- 优化求解器参数（ADMM 惩罚系数）
- 碰撞避免参数（安全距离、障碍物数量）
- 机器人运动学参数

这些参数可以通过 `rqt_reconfigure` 工具进行可视化调整，无需重新启动节点。

**Section sources**
- [README.md](file://RDA-planner/README.md#L0-L85)

## 优化求解器问题排查

### OSQP 安装问题
虽然 RDA-planner 默认使用 ECOS 和 SCS 求解器，但用户可能希望使用 OSQP。常见问题及解决方案：

1. **安装失败**: 确保系统已安装必要的编译工具（如 Visual Studio Build Tools 或 gcc）
2. **版本冲突**: 使用虚拟环境隔离依赖
3. **求解器不可用**: 在 `rda_solver.py` 中修改求解器选择逻辑

```python
# 在 rda_solver.py 中可以指定求解器
self.prob_su.solve(solver=cp.OSQP, verbose=False)
```

### 求解稳定性问题
如果遇到求解不稳定的情况，建议：

- 将 `accelerated` 参数设为 False
- 减小 `ro1` 参数值（如 ro1=1）
- 增加 `iter_num` 迭代次数
- 检查输入数据的数值稳定性

**Section sources**
- [rda_solver.py](file://RDA-planner/RDA_planner/rda_solver.py#L0-L799)

## 安装验证

### 运行示例测试
安装完成后，通过运行示例来验证安装是否成功：

```bash
# 安装 ir-sim 仿真环境
pip install ir-sim

# 运行路径跟踪示例
python example/path_track/path_track.py

# 运行动态避障示例
python example/dynamic_obs/dynamic_obs.py
```

### 验证输出
成功的安装应产生以下结果：

- 无 Python 导入错误
- 示例程序正常启动并显示可视化界面
- MPC 控制器能够生成平滑的控制指令
- 碰撞避免算法有效工作，机器人能成功避开障碍物

如果所有示例都能正常运行，则表明 RDA-planner 已正确安装和配置。

**Section sources**
- [README.md](file://RDA-planner/README.md#L0-L85)
- [example](file://RDA-planner/example#L0-L0)