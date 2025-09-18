# teb_local_planner 使用指南

<cite>
**本文档中引用的文件**  
- [TebLocalPlannerReconfigure.cfg](file://teb_local_planner/cfg/TebLocalPlannerReconfigure.cfg)
- [teb_local_planner_ros.h](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h)
- [publish_test_obstacles.py](file://teb_local_planner/scripts/publish_test_obstacles.py)
- [publish_viapoints.py](file://teb_local_planner/scripts/publish_viapoints.py)
</cite>

## 目录
1. [简介](#简介)
2. [teb_local_planner 概述](#teb_local_planner-概述)
3. [核心配置参数详解](#核心配置参数详解)
4. [ROS接口与teb_local_planner_ros.h使用](#ros接口与teb_local_planner_rossh使用)
5. [测试与调试脚本使用方法](#测试与调试脚本使用方法)
6. [使用示例](#使用示例)
7. [与ROS导航栈集成](#与ros导航栈集成)
8. [常见问题排查](#常见问题排查)
9. [结论](#结论)

## 简介
teb_local_planner（Timed Elastic Band Local Planner）是一种基于优化的局部路径规划器，专为移动机器人设计，能够在动态环境中实现平滑、安全且高效的路径跟踪。本指南详细介绍了其配置、调试、ROS接口使用方法以及与ROS导航栈的集成方式，帮助用户充分发挥其性能。

## teb_local_planner 概述
teb_local_planner 通过将路径建模为“时序弹性带”（Timed Elastic Band），结合非线性优化技术，在满足机器人动力学约束的同时，最小化路径长度、时间、与障碍物的距离等代价函数。它支持差速驱动、阿克曼转向和全向移动机器人，并具备多拓扑路径探索（Homotopy Class Planning）能力，能够有效避开局部最优陷阱。

**Section sources**
- [teb_local_planner_ros.h](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h)

## 核心配置参数详解

### 轨迹参数（Trajectory）
这些参数控制轨迹的生成、优化和动态调整。

- **teb_autosize**: 启用后，轨迹在优化过程中会根据时间分辨率自动调整大小，推荐启用。
- **dt_ref**: 轨迹的时间分辨率，通常设置为控制频率的倒数（如1/30秒）。
- **dt_hysteresis**: 自动调整时间分辨率时的滞后值，通常为`dt_ref`的10%。
- **max_global_plan_lookahead_dist**: 用于优化的全局路径子集的最大长度，限制了局部规划的视野范围。
- **feasibility_check_lookahead_distance**: 每个采样周期内检查轨迹可行性的最大距离。
- **exact_arc_length**: 是否使用精确的弧长计算（计算量大但更准确），否则使用欧几里得距离近似。

### 机器人参数（Robot）
定义机器人的运动学和动力学限制。

- **max_vel_x**: x方向最大线速度。
- **max_vel_theta**: 最大角速度。
- **acc_lim_x**: 最大线加速度。
- **acc_lim_theta**: 最大角加速度。
- **min_turning_radius**: 阿克曼转向机器人的最小转弯半径。
- **cmd_angle_instead_rotvel**: 对于阿克曼机器人，是否将控制指令中的角速度替换为转向角。

### 目标容差（GoalTolerance）
控制机器人到达目标时的精度。

- **xy_goal_tolerance**: 到达目标位置的允许误差。
- **yaw_goal_tolerance**: 到达目标朝向的允许误差。
- **free_goal_vel**: 是否允许机器人以非零速度到达目标。

### 障碍物参数（Obstacles）
控制机器人与障碍物的交互行为。

- **min_obstacle_dist**: 与障碍物之间的最小安全距离。
- **inflation_dist**: 成本地图中障碍物的膨胀距离，应大于`min_obstacle_dist`。
- **include_dynamic_obstacles**: 是否预测动态障碍物的运动。
- **obstacle_association_force_inclusion_factor**: 强制将距离轨迹一定倍数`min_obstacle_dist`内的障碍物纳入优化。
- **obstacle_proximity_ratio_max_vel**: 靠近障碍物时，最大速度的缩减比例。

### 优化参数（Optimization）
控制非线性优化器的行为和权重。

- **optimization_activate**: 是否激活优化过程。
- **weight_max_vel_x**: 满足最大线速度约束的优化权重。
- **weight_obstacle**: 与障碍物保持最小距离的优化权重，值越大避障越强。
- **weight_optimaltime**: 最小化轨迹时间的优化权重。
- **weight_shortest_path**: 最小化路径长度的优化权重。
- **weight_kinematics_nh**: 满足非完整约束（如差速驱动）的优化权重，通常设为较高值。

### 同伦类规划（HCPlanning）
控制多拓扑路径的探索和选择。

- **enable_multithreading**: 是否启用多线程并行规划多条路径。
- **max_number_classes**: 允许探索的最大同伦类数量。
- **selection_cost_hysteresis**: 新路径被选为最佳路径所需的成本优势比例。
- **switching_blocking_period**: 切换到新同伦类前需要等待的时间，防止频繁切换。

**Section sources**
- [TebLocalPlannerReconfigure.cfg](file://teb_local_planner/cfg/TebLocalPlannerReconfigure.cfg#L1-L447)

## ROS接口与teb_local_planner_ros.h使用

`teb_local_planner_ros.h` 是 teb_local_planner 的核心ROS接口头文件。它继承自`nav_core::BaseLocalPlanner`，实现了ROS导航栈要求的接口。该类负责：

- 订阅`/odom`、`/map`或`/costmap`等传感器和地图数据。
- 接收来自全局规划器的`global_plan`。
- 发布`/cmd_vel`速度指令。
- 发布用于可视化的轨迹和障碍物信息（如`/teb_local_planner/teb_path`）。
- 处理动态重配置（Dynamic Reconfigure）请求，允许在运行时修改参数。

用户通常不需要直接修改此文件，而是通过ROS参数服务器或`rqt_reconfigure`工具来调整其行为。关键的配置通过`TebLocalPlannerReconfigure.cfg`生成的动态重配置服务器进行管理。

**Section sources**
- [teb_local_planner_ros.h](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h#L1-L50)

## 测试与调试脚本使用方法

teb_local_planner 提供了多个Python脚本用于测试和调试，位于`scripts/`目录下。

### publish_test_obstacles.py
该脚本发布自定义的障碍物消息，用于测试规划器的避障能力。

```python
# 关键代码片段
obstacle_msg = ObstacleArrayMsg()
obstacle_msg.header.frame_id = "odom" # 坐标系

# 创建点障碍物
obstacle_msg.obstacles[0].polygon.points[0].x = 1.5
obstacle_msg.obstacles[0].polygon.points[0].y = 0

# 创建线障碍物
line_start = Point32(x=-2.5, y=0.5)
line_end = Point32(x=-2.5, y=2)
obstacle_msg.obstacles[1].polygon.points = [line_start, line_end]

# 动态移动障碍物
obstacle_msg.obstacles[0].polygon.points[0].y = 1*math.sin(t)
```

**使用方法**:
1. 确保`costmap_converter`包已安装。
2. 修改脚本中的`frame_id`（如`odom`或`map`）以匹配你的坐标系。
3. 修改`Publisher`的topic名称，使其与teb_local_planner订阅的障碍物topic一致（默认为`/obstacles`）。
4. 运行脚本：`rosrun teb_local_planner publish_test_obstacles.py`。

**Section sources**
- [publish_test_obstacles.py](file://teb_local_planner/scripts/publish_test_obstacles.py#L1-L77)

### publish_viapoints.py
该脚本发布途经点（Via-points），强制规划器经过这些点。

```python
# 关键代码片段
via_points_msg = Path()
via_points_msg.header.frame_id = "odom"

# 添加途经点
point1 = PoseStamped()
point1.pose.position.x = 0.0
point1.pose.position.y = 1.5

point2 = PoseStamped()
point2.pose.position.x = 2.0
point2.pose.position.y = -0.5

via_points_msg.poses = [point1, point2]
```

**使用方法**:
1. 修改`frame_id`和途经点的坐标。
2. 确保teb_local_planner的`global_plan_viapoint_sep`参数为负值（禁用从全局路径提取途经点），并确保`via_points_ordered`设置正确。
3. 运行脚本：`rosrun teb_local_planner publish_viapoints.py`。

**Section sources**
- [publish_viapoints.py](file://teb_local_planner/scripts/publish_viapoints.py#L1-L47)

## 使用示例

### 基本路径跟踪
1. 启动ROS核心、机器人驱动、定位（如AMCL）和地图服务器。
2. 启动move_base节点，并配置teb_local_planner为局部规划器。
3. 在RViz中设置目标点，机器人将自动规划并跟踪路径。

### 复杂动态环境避障
1. 使用`publish_test_obstacles.py`脚本创建移动的障碍物。
2. 观察teb_local_planner如何实时重新规划路径以避开动态障碍物。
3. 调整`min_obstacle_dist`和`weight_obstacle`等参数，观察避障行为的变化。

### 途经点导航
1. 使用`publish_viapoints.py`脚本发布一系列途经点。
2. 设置目标点，规划器将生成一条经过所有途经点的平滑轨迹。

## 与ROS导航栈集成
要将teb_local_planner集成到ROS导航栈中，需在`move_base`的配置文件中指定：

```yaml
# move_base_params.yaml
base_local_planner: "teb_local_planner/TebLocalPlannerROS"
```

同时，创建一个`teb_local_planner_params.yaml`文件，包含所有自定义参数，并在launch文件中加载。

## 常见问题排查

- **机器人振荡或无法到达目标**:
  - 检查`xy_goal_tolerance`和`yaw_goal_tolerance`是否过小。
  - 尝试降低`weight_obstacle`或增加`min_obstacle_dist`。
  - 启用`oscillation_recovery`恢复行为。

- **规划失败或频繁重规划**:
  - 检查传感器数据和`costmap`是否正常。
  - 增加`max_global_plan_lookahead_dist`以扩大规划视野。
  - 检查`transform_tolerance`是否足够大以容忍TF延迟。

- **不避障**:
  - 确认`include_costmap_obstacles`已启用。
  - 检查障碍物消息的`frame_id`是否与`costmap`一致。
  - 确认`min_obstacle_dist`小于`inflation_dist`。

- **动态障碍物不生效**:
  - 确保`include_dynamic_obstacles`已启用。
  - 确认发布的障碍物消息包含速度信息。

## 结论
teb_local_planner 是一个功能强大且灵活的局部规划器。通过深入理解其核心参数、熟练使用调试脚本，并正确集成到ROS导航栈中，用户可以为机器人实现高性能的自主导航。持续的参数调优和对实际环境的测试是充分发挥其潜力的关键。