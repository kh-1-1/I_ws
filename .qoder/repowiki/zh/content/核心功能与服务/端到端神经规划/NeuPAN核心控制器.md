# NeuPAN核心控制器

<cite>
**本文档引用的文件**  
- [neupan.py](file://NeuPAN/neupan/neupan.py)
- [blocks/pan.py](file://NeuPAN/neupan/blocks/pan.py)
- [blocks/dune.py](file://NeuPAN/neupan/blocks/dune.py)
- [blocks/nrmp.py](file://NeuPAN/neupan/blocks/nrmp.py)
- [blocks/initial_path.py](file://NeuPAN/neupan/blocks/initial_path.py)
- [robot/robot.py](file://NeuPAN/neupan/robot/robot.py)
- [README.md](file://NeuPAN/README.md)
- [example/run_exp.py](file://NeuPAN/example/run_exp.py)
</cite>

## 目录
1. [简介](#简介)
2. [项目结构](#项目结构)
3. [核心组件](#核心组件)
4. [架构概述](#架构概述)
5. [详细组件分析](#详细组件分析)
6. [依赖关系分析](#依赖关系分析)
7. [性能考量](#性能考量)
8. [故障排除指南](#故障排除指南)
9. [结论](#结论)

## 简介
NeuPAN（Neural Proximal Alternating-minimization Network）是一种端到端、实时、无需地图且易于部署的基于模型预测控制（MPC）的机器人运动规划器。它通过整合基于学习和基于优化的技术，直接将障碍物点数据映射为实时控制动作，通过求解一个包含大量点级避障约束的端到端数学模型来实现。这消除了中间模块设计，避免了误差传播，实现了高精度，使机器人能够在杂乱和未知的环境中高效、安全地导航。

## 项目结构
NeuPAN项目遵循模块化设计，主要组件位于`neupan`包内。核心控制器`neupan.py`作为主入口，封装了底层神经网络块（如PAN、DUNE）。`blocks`目录包含核心算法组件，`robot`目录定义机器人模型，`configuration`和`util`提供配置和工具函数。`example`目录包含多个演示场景的配置文件和运行脚本。

```mermaid
graph TD
neupan[neupan.py] --> blocks[blocks/]
neupan --> robot[robot/]
blocks --> pan[PAN]
blocks --> dune[DUNE]
blocks --> nrmp[NRMP]
blocks --> initial_path[InitialPath]
robot --> robot_class[robot]
```

**图表来源**
- [neupan.py](file://NeuPAN/neupan/neupan.py)
- [blocks/pan.py](file://NeuPAN/neupan/blocks/pan.py)
- [blocks/dune.py](file://NeuPAN/neupan/blocks/dune.py)
- [blocks/nrmp.py](file://NeuPAN/neupan/blocks/nrmp.py)
- [blocks/initial_path.py](file://NeuPAN/neupan/blocks/initial_path.py)
- [robot/robot.py](file://NeuPAN/neupan/robot/robot.py)

## 核心组件
`neupan.py`模块是NeuPAN算法的主类，它封装了PAN类并提供了更友好的用户接口。其主要职责包括状态估计、轨迹优化和控制指令生成。它作为端到端MPC规划器，协调底层神经网络块和机器人动力学模型。

**章节来源**
- [neupan.py](file://NeuPAN/neupan/neupan.py)

## 架构概述
NeuPAN的核心架构由`neupan`控制器、`PAN`（Proximal Alternating-minimization Network）网络、`DUNE`（Deep Unfolded Neural Encoder）和`NRMP`（Neural Regularized Motion Planner）组成。`neupan`控制器接收机器人状态和传感器数据，生成初始路径，并调用`PAN`进行优化。`PAN`通过`DUNE`和`NRMP`的交替最小化过程求解优化问题。

```mermaid
graph TD
subgraph "NeuPAN控制器"
neupan[neupan]
initial_path[InitialPath]
end
subgraph "PAN网络"
pan[PAN]
dune[DUNE]
nrmp[NRMP]
end
subgraph "机器人模型"
robot[robot]
end
neupan --> pan
neupan --> initial_path
pan --> dune
pan --> nrmp
pan --> robot
initial_path --> robot
```

**图表来源**
- [neupan.py](file://NeuPAN/neupan/neupan.py)
- [blocks/pan.py](file://NeuPAN/neupan/blocks/pan.py)
- [blocks/dune.py](file://NeuPAN/neupan/blocks/dune.py)
- [blocks/nrmp.py](file://NeuPAN/neupan/blocks/nrmp.py)
- [blocks/initial_path.py](file://NeuPAN/neupan/blocks/initial_path.py)
- [robot/robot.py](file://NeuPAN/neupan/robot/robot.py)

## 详细组件分析

### NeuPAN控制器分析
`neupan`类是整个系统的主控制器，负责协调所有组件并执行主控制循环。

#### 类图
```mermaid
classDiagram
class neupan {
+int T
+float dt
+float ref_speed
+dict info
+np.ndarray cur_vel_array
+robot robot
+InitialPath ipath
+PAN pan
+float collision_threshold
+__init__(receding, step_time, ref_speed, device, robot_kwargs, ipath_kwargs, pan_kwargs, adjust_kwargs, train_kwargs, **kwargs)
+init_from_yaml(yaml_file, **kwargs)
+forward(state, points, velocities)
+check_stop()
+scan_to_point(state, scan, scan_offset, angle_range, down_sample)
+scan_to_point_velocity(state, scan, scan_offset, angle_range, down_sample)
+train_dune()
+reset()
+set_initial_path(path)
+set_initial_path_from_state(state)
+set_reference_speed(speed)
+update_initial_path_from_goal(start, goal)
+update_initial_path_from_waypoints(waypoints)
+update_adjust_parameters(**kwargs)
+min_distance()
+dune_points()
+nrmp_points()
+initial_path()
+adjust_parameters()
+waypoints()
+opt_trajectory()
+ref_trajectory()
}
neupan --> InitialPath : "uses"
neupan --> PAN : "uses"
neupan --> robot : "uses"
```

**图表来源**
- [neupan.py](file://NeuPAN/neupan/neupan.py)

#### 主控制循环序列图
```mermaid
sequenceDiagram
participant User as "用户"
participant Controller as "neupan"
participant Path as "InitialPath"
participant PAN as "PAN"
participant DUNE as "DUNE"
participant NRMP as "NRMP"
User->>Controller : 初始化 (init_from_yaml)
Controller->>Path : 初始化初始路径
Controller->>PAN : 初始化
PAN->>DUNE : 初始化
PAN->>NRMP : 初始化
loop 主控制循环
User->>Controller : 提供状态和点云 (forward)
Controller->>Path : 检查是否到达目标
alt 到达目标
Controller-->>User : 返回停止指令
break
end
Controller->>Path : 生成名义参考状态
Controller->>PAN : 执行优化 (forward)
PAN->>DUNE : 生成点流和变换
DUNE->>DUNE : 映射到潜在距离空间 (mu, lam)
PAN->>NRMP : 执行优化迭代
NRMP->>NRMP : 求解优化问题
PAN->>PAN : 检查收敛条件
alt 未收敛
PAN->>DUNE : 使用新状态重新计算
PAN->>NRMP : 再次优化
end
PAN-->>Controller : 返回最优状态和速度
Controller->>Controller : 更新当前速度数组
Controller->>Controller : 检查停止条件
alt 应停止
Controller-->>User : 返回停止指令
else
Controller-->>User : 返回控制动作
end
end
```

**图表来源**
- [neupan.py](file://NeuPAN/neupan/neupan.py)
- [blocks/pan.py](file://NeuPAN/neupan/blocks/pan.py)
- [blocks/dune.py](file://NeuPAN/neupan/blocks/dune.py)
- [blocks/nrmp.py](file://NeuPAN/neupan/blocks/nrmp.py)
- [blocks/initial_path.py](file://NeuPAN/neupan/blocks/initial_path.py)

### PAN网络分析
PAN（Proximal Alternating-minimization Network）是NeuPAN的核心优化网络，由NRMP和DUNE两个组件通过交替最小化过程组成。

#### 类图
```mermaid
classDiagram
class PAN {
+int T
+float dt
+int iter_num
+float iter_threshold
+NRMP nrmp_layer
+DUNE dune_layer
+list current_nom_values
+bool printed
+__init__(receding, step_time, robot, iter_num, dune_max_num, nrmp_max_num, dune_checkpoint, iter_threshold, adjust_kwargs, train_kwargs, **kwargs)
+forward(nom_s, nom_u, ref_s, ref_us, obs_points, point_velocities)
+generate_point_flow(nom_s, obs_points, point_velocities)
+point_state_transform(state, obs_points)
+stop_criteria(nom_s, nom_u, mu_list, lam_list)
+min_distance()
+dune_points()
+nrmp_points()
}
PAN --> NRMP : "contains"
PAN --> DUNE : "contains"
PAN --> robot : "uses"
```

**图表来源**
- [blocks/pan.py](file://NeuPAN/neupan/blocks/pan.py)

### DUNE组件分析
DUNE（Deep Unfolded Neural Encoder）将点流映射到潜在距离空间（mu和lambda），为NRMP提供避障约束。

#### 类图
```mermaid
classDiagram
class DUNE {
+int T
+int max_num
+torch.Tensor G
+torch.Tensor h
+int edge_dim
+int state_dim
+ObsPointNet model
+torch.Tensor obstacle_points
+float min_distance
+__init__(receding, checkpoint, robot, dune_max_num, train_kwargs)
+forward(point_flow, R_list, obs_points_list)
+cal_objective_distance(mu, p0)
+load_model(checkpoint, train_kwargs)
+train_dune(train_kwargs)
+ask_to_train()
+ask_to_continue()
+points()
}
DUNE --> ObsPointNet : "uses"
DUNE --> robot : "uses"
```

**图表来源**
- [blocks/dune.py](file://NeuPAN/neupan/blocks/dune.py)

### NRMP组件分析
NRMP（Neural Regularized Motion Planner）求解集成神经潜在距离空间的优化问题，生成最优控制序列。

#### 类图
```mermaid
classDiagram
class NRMP {
+int T
+float dt
+torch.Tensor G
+torch.Tensor h
+int max_num
+bool no_obs
+torch.Tensor eta
+torch.Tensor d_max
+torch.Tensor d_min
+torch.Tensor q_s
+torch.Tensor p_u
+float ro_obs
+float bk
+list adjust_parameters
+str solver
+torch.Tensor obstacle_points
+__init__(receding, step_time, robot, nrmp_max_num, eta, d_max, d_min, q_s, p_u, ro_obs, bk, **kwargs)
+forward(nom_s, nom_u, ref_s, ref_us, mu_list, lam_list, point_list)
+generate_parameter_value(nom_s, nom_u, ref_s, ref_us, mu_list, lam_list, point_list)
+generate_adjust_parameter_value()
+update_adjust_parameters_value(**kwargs)
+generate_coefficient_parameter_value(mu_list, lam_list, point_list)
+variable_definition()
+parameter_definition()
+problem_definition()
+construct_prob()
+nav_cost_cons()
+dune_cost_cons()
+bound_dis_constraints()
+C1_cost_d()
+points()
}
NRMP --> robot : "uses"
NRMP --> CvxpyLayer : "uses"
```

**图表来源**
- [blocks/nrmp.py](file://NeuPAN/neupan/blocks/nrmp.py)

### 初始路径组件分析
InitialPath类负责从给定的航点生成朴素的初始路径，为MPC提供参考轨迹。

#### 类图
```mermaid
classDiagram
class InitialPath {
+int T
+float dt
+float ref_speed
+robot robot
+list waypoints
+bool loop
+str curve_style
+float min_radius
+float interval
+float arrive_threshold
+float close_threshold
+int ind_range
+int arrive_index_threshold
+bool arrive_flag
+curve_generator cg
+list initial_path
+list curve_list
+int curve_index
+int point_index
+__init__(receding, step_time, ref_speed, robot, waypoints, loop, curve_style, **kwargs)
+generate_nom_ref_state(state, cur_vel_array, ref_speed)
+set_initial_path(path)
+cal_average_interval(path)
+closest_point(state, threshold, ind_range)
+find_interaction_point(ref_state, ref_index, length)
+range_cir_seg(circle, r, segment)
+check_arrive(state)
+check_curve_arrive(state, arrive_threshold, arrive_index_threshold)
+split_path_with_gear()
+init_path_with_state(state)
+init_check(state)
+set_ipath_with_state(state)
+update_initial_path_from_goal(start, goal)
+set_ipath_with_waypoints(waypoints)
+motion_predict_model(robot_state, vel, wheel_base, sample_time)
+ackermann_model(car_state, vel, wheel_base, sample_time)
+diff_model(robot_state, vel, sample_time)
+cur_waypoints()
+cur_curve()
+cur_point()
+curve_number()
+default_turn_radius()
+_ensure_consistent_angles()
+trans_to_np_list(point_list)
}
InitialPath --> robot : "uses"
InitialPath --> curve_generator : "uses"
```

**图表来源**
- [blocks/initial_path.py](file://NeuPAN/neupan/blocks/initial_path.py)

### 机器人模型分析
robot类定义了机器人的模型和运动学模型，为优化问题生成约束和成本函数。

#### 类图
```mermaid
classDiagram
class robot {
+int T
+float dt
+float L
+str kinematics
+np.ndarray max_speed
+np.ndarray max_acce
+np.ndarray speed_bound
+np.ndarray acce_bound
+str name
+np.ndarray vertices
+torch.Tensor G
+torch.Tensor h
+str shape
+float length
+float width
+float wheelbase
+__init__(receding, step_time, kinematics, vertices, max_speed, max_acce, wheelbase, length, width, **kwargs)
+define_variable(no_obs, indep_dis)
+state_parameter_define()
+coefficient_parameter_define(no_obs, max_num)
+C0_cost(para_p_u, para_q_s)
+proximal_cost()
+I_cost(indep_dis, ro_obs)
+dynamics_constraint()
+bound_su_constraints()
+generate_state_parameter_value(nom_s, nom_u, qs_ref_s, pu_ref_us)
+linear_ackermann_model(nom_st, nom_ut, dt, L)
+linear_diff_model(nom_state, nom_u, dt)
+cal_vertices_from_length_width(length, width, wheelbase)
+cal_vertices(vertices, length, width, wheelbase)
}
robot --> cvxpy : "uses"
robot --> torch : "uses"
```

**图表来源**
- [robot/robot.py](file://NeuPAN/neupan/robot/robot.py)

## 依赖关系分析
NeuPAN的组件之间存在清晰的依赖关系。`neupan`控制器依赖于`PAN`、`InitialPath`和`robot`。`PAN`依赖于`DUNE`、`NRMP`和`robot`。`DUNE`和`NRMP`都直接依赖于`robot`模型。这种分层依赖确保了系统的模块化和可维护性。

```mermaid
graph TD
neupan --> PAN
neupan --> InitialPath
neupan --> robot
PAN --> DUNE
PAN --> NRMP
PAN --> robot
InitialPath --> robot
DUNE --> robot
NRMP --> robot
```

**图表来源**
- [neupan.py](file://NeuPAN/neupan/neupan.py)
- [blocks/pan.py](file://NeuPAN/neupan/blocks/pan.py)
- [blocks/dune.py](file://NeuPAN/neupan/blocks/dune.py)
- [blocks/nrmp.py](file://NeuPAN/neupan/blocks/nrmp.py)
- [blocks/initial_path.py](file://NeuPAN/neupan/blocks/initial_path.py)
- [robot/robot.py](file://NeuPAN/neupan/robot/robot.py)

## 性能考量
NeuPAN的性能受多个因素影响。计算能力是关键，推荐使用强大的CPU（如Intel i7）以实现高于10Hz的控制频率。参数调整也至关重要，`receding`、`nrmp_max_num`、`dune_max_num`、`iter_num`和`iter_threshold`等参数可以调整以平衡性能和计算成本。实时调整`adjust`参数可以适应不同场景，使行为更激进或更保守。

**章节来源**
- [README.md](file://NeuPAN/README.md)

## 故障排除指南
常见问题包括DUNE模型未找到、性能不佳和传感器集成。如果DUNE模型未找到，系统会提示训练新模型。性能不佳时，应检查硬件平台、调整参数或优化初始路径。对于3D激光雷达或相机等传感器，可以将3D点投影到2D平面或从图像中提取2D点来使用。

**章节来源**
- [README.md](file://NeuPAN/README.md)

## 结论
NeuPAN是一个先进的端到端MPC规划器，通过整合学习和优化技术，实现了高效、安全的机器人导航。其模块化设计和清晰的接口使其易于扩展和部署。通过合理配置和优化，NeuPAN可以在各种复杂环境中表现出色。