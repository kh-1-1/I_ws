# RDA求解器核心

<cite>
**本文档中引用的文件**  
- [rda_solver.py](file://RDA-planner/RDA_planner/rda_solver.py)
- [mpc.py](file://RDA-planner/RDA_planner/mpc.py)
- [corridor.py](file://RDA-planner/example/corridor/corridor.py)
- [corridor.yaml](file://RDA-planner/example/corridor/corridor.yaml)
</cite>

## 目录
1. [引言](#引言)
2. [项目结构](#项目结构)
3. [核心组件](#核心组件)
4. [架构概述](#架构概述)
5. [详细组件分析](#详细组件分析)
6. [依赖分析](#依赖分析)
7. [性能考量](#性能考量)
8. [故障排除指南](#故障排除指南)
9. [结论](#结论)

## 引言
RDA求解器是一种基于凸优化的实时轨迹规划算法，专为动态环境中的机器人避障设计。该求解器通过将非凸的碰撞避免问题转化为高效的凸优化问题，实现了在复杂环境下的实时路径规划。其核心思想是利用交替方向乘子法（ADMM）进行并行优化，将原问题分解为状态更新（su）子问题和对偶变量更新（LamMuZ）子问题，从而实现高效的求解。本文档深入解析rda_solver.py中的核心求解算法，详细阐述其并行优化架构和数学原理。

## 项目结构
RDA-planner项目采用模块化设计，主要包含求解器核心（rda_solver.py）和MPC控制器（mpc.py）两个核心模块。求解器核心负责实现具体的优化算法，而MPC控制器则负责与外部环境交互，提供高层控制逻辑。项目还包含多个示例文件，用于演示不同场景下的应用。

```mermaid
graph TD
RDA-planner --> RDA_planner
RDA_planner --> rda_solver.py
RDA_planner --> mpc.py
RDA-planner --> example
example --> corridor
example --> dynamic_obs
example --> lidar_nav
example --> path_track
example --> reverse
```

**图源**  
- [rda_solver.py](file://RDA-planner/RDA_planner/rda_solver.py)
- [mpc.py](file://RDA-planner/RDA_planner/mpc.py)

**节源**  
- [rda_solver.py](file://RDA-planner/RDA_planner/rda_solver.py)
- [mpc.py](file://RDA-planner/RDA_planner/mpc.py)

## 核心组件
RDA求解器的核心组件包括状态变量、对偶变量、障碍物参数和优化问题的构建。状态变量包括机器人的位置、速度和方向，对偶变量用于处理障碍物约束，障碍物参数则描述了环境中障碍物的几何形状和位置。优化问题的构建分为两个子问题：状态更新子问题和对偶变量更新子问题，通过交替求解这两个子问题来逼近最优解。

**节源**  
- [rda_solver.py](file://RDA-planner/RDA_planner/rda_solver.py#L16-L1067)

## 架构概述
RDA求解器的架构基于ADMM算法，将原问题分解为两个子问题：状态更新（su）子问题和对偶变量更新（LamMuZ）子问题。状态更新子问题负责更新机器人的状态变量，包括位置、速度和方向，同时考虑动力学约束和目标函数。对偶变量更新子问题负责更新与障碍物相关的对偶变量，包括拉格朗日乘子和松弛变量，以确保机器人与障碍物之间的安全距离。

```mermaid
graph TD
A[初始化] --> B[状态更新子问题]
B --> C[对偶变量更新子问题]
C --> D[检查收敛性]
D --> |未收敛| B
D --> |已收敛| E[输出最优解]
```

**图源**  
- [rda_solver.py](file://RDA-planner/RDA_planner/rda_solver.py#L16-L1067)

## 详细组件分析
### RDA求解器类分析
RDA求解器类（RDA_solver）是整个算法的核心，负责管理所有变量、参数和优化问题的求解。该类的初始化方法（__init__）设置了求解器的基本参数，包括预测时域、车辆模型、最大障碍物数量等。定义方法（definition）负责定义所有变量和参数，包括状态变量、对偶变量、障碍物参数等。

#### 对象导向组件
```mermaid
classDiagram
class RDA_solver {
+T : int
+car_tuple : tuple
+max_edge_num : int
+max_obs_num : int
+iter_num : int
+dt : float
+iter_threshold : float
+process_num : int
+accelerated : bool
+time_print : bool
+indep_s : cp.Variable
+indep_u : cp.Variable
+indep_dis : cp.Variable
+indep_rot_list : list
+indep_lam_list : list
+indep_mu_list : list
+indep_z_list : list
+para_ref_s : cp.Parameter
+para_ref_speed : cp.Parameter
+para_s : cp.Parameter
+para_u : cp.Parameter
+para_dis : cp.Parameter
+para_rot_list : list
+para_drot_list : list
+para_drot_phi_list : list
+para_A_list : list
+para_B_list : list
+para_C_list : list
+para_lam_list : list
+para_mu_list : list
+para_z_list : list
+para_xi_list : list
+para_zeta_list : list
+para_obstacle_list : list
+para_obsA_lam_list : list
+para_obsb_lam_list : list
+para_obsA_rot_list : list
+para_obsA_trans_list : list
+para_slack_gain : cp.Parameter
+para_max_sd : cp.Parameter
+para_min_sd : cp.Parameter
+ro1 : cp.Parameter
+ro2 : cp.Parameter
+prob_su : cp.Problem
+prob_LamMuZ_list : list
+__init__(self, receding, car_tuple, max_edge_num, max_obs_num, iter_num, step_time, iter_threshold, process_num, accelerated, time_print, **kwargs)
+definition(self, **kwargs)
+state_variable_define(self)
+dual_variable_define(self)
+state_parameter_define(self)
+dual_parameter_define(self)
+obstacle_parameter_define(self)
+combine_parameter_define(self)
+combine_variable_define(self)
+adjust_parameter_define(self, **kwargs)
+construct_problem(self, **kwargs)
+construct_mp_problem(self, process_num, **kwargs)
+construct_su_prob(self, **kwargs)
+construct_LamMuZ_prob(self, **kwargs)
+init_prob_LamMuZ(self, kwargs)
+construct_LamMuZ_prob_parallel(self, para_xi_list, para_zeta_list, para_s, para_rot_list, para_dis, para_obstacle_list, para_obsA_rot_list, para_obsA_trans_list, **kwargs)
+nav_cost_cons(self, ws, wu)
+update_su_cost_cons(self, slack_gain, ro1, ro2)
+LamMuZ_cost_cons(self, indep_lam, indep_mu, indep_z, indep_Im_lamMuZ, indep_Hm_lamMuZ, para_s, para_rot_list, para_xi, para_dis, para_zeta, para_obs, para_obsA_rot, para_obsA_trans, receding, ro1, ro2)
+assign_adjust_parameter(self, **kwargs)
+assign_state_parameter(self, nom_s, nom_u, nom_dis)
+assign_state_parameter_parallel(self, input)
+assign_dual_parameter(self, LamMuZ_list)
+assign_obstacle_parameter(self, obstacle_list)
+assign_combine_parameter_lamobs(self)
+assign_combine_parameter_stateobs(self)
+iterative_solve(self, nom_s, nom_u, ref_states, ref_speed, obstacle_list, **kwargs)
+rda_solver(self)
+update_zeta(self)
+update_xi(self)
+su_prob_solve(self)
+LamMuZ_prob_solve(self)
+solve_parallel(input)
+solve_direct(self, input)
+Im_su(self, state, distance, para_lam, para_mu, para_z, para_zeta, para_obs, para_obsA_lam, para_obsb_lam)
+Hm_su(self, rot, para_mu, para_lam, para_xi, para_obs, receding, para_obsA_lam)
+Hm_LamMu(self, indep_lam, indep_mu, para_rot_list, para_xi, para_obs, receding, para_obsA_rot)
+Im_LamMu(self, indep_lam, indep_mu, indep_z, para_s, para_dis, para_zeta, para_obs, para_obsA_trans)
+dynamics_constraint(self, state, control_u, receding)
+bound_su_constraints(self, state, control_u, para_s, max_speed, acce_bound)
+bound_dis_constraints(self, indep_dis)
+linear_ackermann_model(self, nom_state, nom_u, dt, L)
+linear_diff_model(self, nom_state, nom_u, dt)
+linear_omni_model(self, nom_u, dt)
+C0_cost(self, ref_s, ref_speed, state, control_u, ws, wu)
+C1_cost(self, indep_dis, slack_gain)
+cone_cp_array(self, array, cone)
+cone_para_array(self, array, cone_flag)
+get_adjust_parameter(self)
+reset(self)
}
```

**图源**  
- [rda_solver.py](file://RDA-planner/RDA_planner/rda_solver.py#L16-L1067)

### MPC控制器分析
MPC控制器（MPC）是RDA求解器的上层控制器，负责与外部环境交互，提供高层控制逻辑。该控制器接收机器人的当前状态、参考路径和障碍物列表，通过调用RDA求解器来计算最优控制输入。MPC控制器还负责处理参考路径的更新、障碍物的转换和排序等任务。

#### API/服务组件
```mermaid
sequenceDiagram
participant 用户 as 用户
participant MPC as MPC控制器
participant RDA as RDA求解器
用户->>MPC : control(state, ref_speed, obstacle_list)
MPC->>MPC : pre_process(state, ref_path, cur_index, ref_speed)
MPC->>MPC : convert_rda_obstacle(obstacle_list, state, obstacle_order)
MPC->>RDA : iterative_solve(nom_s, nom_u, ref_traj_list, gear_flag * ref_speed, rda_obs_list)
RDA->>RDA : su_prob_solve()
RDA->>RDA : LamMuZ_prob_solve()
RDA-->>MPC : opt_velocity_array, info
MPC-->>用户 : opt_velocity_array[ : , 0 : 1], info
```

**图源**  
- [mpc.py](file://RDA-planner/RDA_planner/mpc.py#L14-L568)

## 依赖分析
RDA求解器依赖于多个外部库，包括cvxpy用于凸优化求解，numpy用于数值计算，pathos用于多进程并行计算。此外，求解器还依赖于MPC控制器提供的高层控制逻辑，以及外部环境提供的状态信息和障碍物信息。

```mermaid
graph TD
RDA_solver --> cvxpy
RDA_solver --> numpy
RDA_solver --> pathos
RDA_solver --> MPC
MPC --> RDA_solver
MPC --> 用户
用户 --> MPC
```

**图源**  
- [rda_solver.py](file://RDA-planner/RDA_planner/rda_solver.py#L16-L1067)
- [mpc.py](file://RDA-planner/RDA_planner/mpc.py#L14-L568)

**节源**  
- [rda_solver.py](file://RDA-planner/RDA_planner/rda_solver.py#L16-L1067)
- [mpc.py](file://RDA-planner/RDA_planner/mpc.py#L14-L568)

## 性能考量
RDA求解器的性能主要受预测时域、障碍物数量和迭代次数的影响。较长的预测时域可以提高规划的准确性，但会增加计算时间。较多的障碍物数量会增加优化问题的复杂度，从而影响求解速度。较多的迭代次数可以提高解的精度，但也会增加计算时间。为了提高性能，可以调整求解器的参数，如减少预测时域、限制障碍物数量或减少迭代次数。

## 故障排除指南
在使用RDA求解器时，可能会遇到一些常见问题，如求解失败、收敛速度慢等。求解失败可能是由于初始状态不合法或障碍物约束过于严格导致的。收敛速度慢可能是由于参数设置不当或问题复杂度过高导致的。为了解决这些问题，可以检查初始状态的合法性，调整障碍物约束的严格程度，或优化求解器的参数设置。

**节源**  
- [rda_solver.py](file://RDA-planner/RDA_planner/rda_solver.py#L16-L1067)
- [mpc.py](file://RDA-planner/RDA_planner/mpc.py#L14-L568)

## 结论
RDA求解器是一种高效的实时轨迹规划算法，通过将非凸的碰撞避免问题转化为凸优化问题，实现了在复杂环境下的实时路径规划。其基于ADMM的并行优化架构，使得求解过程高效且稳定。通过合理调整求解器的参数，可以在保证规划质量的同时，满足实时性要求。未来的工作可以进一步优化求解器的性能，提高其在更复杂环境下的适用性。