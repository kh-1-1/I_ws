# PAN路径规划注意力网络

<cite>
**本文档引用的文件**   
- [pan.py](file://NeuPAN/neupan/blocks/pan.py)
- [dune.py](file://NeuPAN/neupan/blocks/dune.py)
- [nrmp.py](file://NeuPAN/neupan/blocks/nrmp.py)
- [initial_path.py](file://NeuPAN/neupan/blocks/initial_path.py)
- [neupan.py](file://NeuPAN/neupan/neupan.py)
- [robot.py](file://NeuPAN/neupan/robot/robot.py)
- [obs_point_net.py](file://NeuPAN/neupan/blocks/obs_point_net.py)
</cite>

## 目录
1. [项目结构](#项目结构)
2. [核心组件](#核心组件)
3. [架构概述](#架构概述)
4. [详细组件分析](#详细组件分析)
5. [依赖分析](#依赖分析)
6. [性能考虑](#性能考虑)
7. [故障排除指南](#故障排除指南)
8. [结论](#结论)

## 项目结构

NeuPAN项目采用模块化设计，主要包含路径规划、机器人模型和工具函数等核心模块。项目结构清晰，各模块职责分明。

```mermaid
graph TD
subgraph "NeuPAN"
subgraph "blocks"
pan[pan.py]
dune[dune.py]
nrmp[nrmp.py]
initial_path[initial_path.py]
obs_point_net[obs_point_net.py]
end
subgraph "robot"
robot[robot.py]
end
neupan[neupan.py]
end
```

**图源**
- [pan.py](file://NeuPAN/neupan/blocks/pan.py)
- [dune.py](file://NeuPAN/neupan/blocks/dune.py)
- [nrmp.py](file://NeuPAN/neupan/blocks/nrmp.py)
- [initial_path.py](file://NeuPAN/neupan/blocks/initial_path.py)
- [neupan.py](file://NeuPAN/neupan/neupan.py)
- [robot.py](file://NeuPAN/neupan/robot/robot.py)

**章节源**
- [pan.py](file://NeuPAN/neupan/blocks/pan.py)
- [dune.py](file://NeuPAN/neupan/blocks/dune.py)
- [nrmp.py](file://NeuPAN/neupan/blocks/nrmp.py)

## 核心组件

PAN（Path Planning Attention Network）是NeuPAN算法的核心，采用交替最小化网络架构，结合NRMP和DUNE模块解决多点碰撞规避优化问题。系统通过初始路径生成、环境感知和优化求解三个主要阶段完成路径规划。

**章节源**
- [pan.py](file://NeuPAN/neupan/blocks/pan.py#L1-L50)
- [neupan.py](file://NeuPAN/neupan/neupan.py#L1-L50)
- [initial_path.py](file://NeuPAN/neupan/blocks/initial_path.py#L1-L50)

## 架构概述

PAN路径规划系统采用编码器-解码器架构，通过注意力机制处理环境观测并生成路径点。系统整体架构包含多个协同工作的组件，实现从环境感知到路径优化的完整流程。

```mermaid
graph TD
subgraph "输入层"
State[当前状态]
Points[障碍物点云]
Velocities[障碍物速度]
end
subgraph "处理层"
InitialPath[初始路径生成]
PAN[PAN核心网络]
subgraph "PAN内部"
DUNE[DUNE模块]
NRMP[NRMP模块]
end
end
subgraph "输出层"
OptState[优化状态]
OptVel[优化速度]
Distance[安全距离]
end
State --> PAN
Points --> PAN
Velocities --> PAN
InitialPath --> PAN
PAN --> OptState
PAN --> OptVel
PAN --> Distance
DUNE --> NRMP
NRMP --> DUNE
```

**图源**
- [pan.py](file://NeuPAN/neupan/blocks/pan.py#L1-L50)
- [neupan.py](file://NeuPAN/neupan/neupan.py#L1-L50)
- [dune.py](file://NeuPAN/neupan/blocks/dune.py#L1-L50)
- [nrmp.py](file://NeuPAN/neupan/blocks/nrmp.py#L1-L50)

## 详细组件分析

### PAN核心网络分析

PAN类作为路径规划的核心，实现了交替最小化算法，通过迭代优化生成安全路径。网络采用编码器-解码器架构，结合注意力机制处理环境观测。

```mermaid
classDiagram
class PAN {
+int T
+float dt
+robot robot
+int iter_num
+float iter_threshold
+NRMP nrmp_layer
+DUNE dune_layer
+forward(nom_s, nom_u, ref_s, ref_us, obs_points, point_velocities)
+generate_point_flow(nom_s, obs_points, point_velocities)
+point_state_transform(state, obs_points)
+stop_criteria(nom_s, nom_u, mu_list, lam_list)
}
PAN --> NRMP : "包含"
PAN --> DUNE : "包含"
PAN --> InitialPath : "依赖"
```

**图源**
- [pan.py](file://NeuPAN/neupan/blocks/pan.py#L1-L272)

**章节源**
- [pan.py](file://NeuPAN/neupan/blocks/pan.py#L1-L272)

### 注意力机制实现

DUNE（Deep Unfolded Neural Encoder）模块实现了注意力机制的核心功能，将障碍物点云映射到潜在距离特征空间，生成注意力权重。

```mermaid
sequenceDiagram
participant PAN as PAN
participant DUNE as DUNE
participant ObsNet as ObsPointNet
PAN->>DUNE : generate_point_flow()
DUNE->>DUNE : 转换点云到机器人坐标系
DUNE->>ObsNet : 批量处理所有时间步的点云
ObsNet->>DUNE : 返回潜在特征mu
loop 每个时间步
DUNE->>DUNE : 切片获取当前时间步的mu
DUNE->>DUNE : 计算lambda特征
DUNE->>DUNE : 计算目标距离
DUNE->>DUNE : 按距离排序点云
end
DUNE-->>PAN : 返回mu_list, lam_list, sort_point_list
```

**图源**
- [dune.py](file://NeuPAN/neupan/blocks/dune.py#L1-L251)
- [obs_point_net.py](file://NeuPAN/neupan/blocks/obs_point_net.py#L1-L72)

**章节源**
- [dune.py](file://NeuPAN/neupan/blocks/dune.py#L1-L251)
- [obs_point_net.py](file://NeuPAN/neupan/blocks/obs_point_net.py#L1-L72)

### 初始路径生成

InitialPath类负责从给定航点生成初始路径，为PAN网络提供参考轨迹。该模块支持多种曲线样式和路径循环功能。

```mermaid
flowchart TD
Start([开始]) --> CheckPath{"初始路径是否存在?"}
CheckPath --> |否| GeneratePath["生成初始路径"]
CheckPath --> |是| UseExisting["使用现有路径"]
GeneratePath --> SplitGear["按档位分割路径"]
SplitGear --> SetIndex["设置曲线和点索引"]
UseExisting --> SetIndex
SetIndex --> End([完成])
```

**图源**
- [initial_path.py](file://NeuPAN/neupan/blocks/initial_path.py#L1-L483)

**章节源**
- [initial_path.py](file://NeuPAN/neupan/blocks/initial_path.py#L1-L483)

### 优化求解器

NRMP（Neural Regularized Motion Planner）模块作为优化求解器，集成神经潜在距离空间，生成最优控制序列。该模块基于凸优化框架实现。

```mermaid
classDiagram
class NRMP {
+int T
+float dt
+robot robot
+int max_num
+forward(nom_s, nom_u, ref_s, ref_us, mu_list, lam_list, point_list)
+generate_parameter_value()
+generate_coefficient_parameter_value()
+construct_prob()
+nav_cost_cons()
+dune_cost_cons()
}
NRMP --> robot : "使用"
NRMP --> CvxpyLayer : "封装"
```

**图源**
- [nrmp.py](file://NeuPAN/neupan/blocks/nrmp.py#L1-L325)

**章节源**
- [nrmp.py](file://NeuPAN/neupan/blocks/nrmp.py#L1-L325)

## 依赖分析

PAN系统各组件之间存在复杂的依赖关系，形成了一个协同工作的整体。理解这些依赖关系对于系统调试和优化至关重要。

```mermaid
graph TD
neupan[neupan.py] --> pan[pan.py]
pan[pan.py] --> nrmp[nrmp.py]
pan[pan.py] --> dune[dune.py]
dune[dune.py] --> obs_point_net[obs_point_net.py]
pan[pan.py] --> initial_path[initial_path.py]
neupan[neupan.py] --> robot[robot.py]
nrmp[nrmp.py] --> robot[robot.py]
initial_path[initial_path.py] --> robot[robot.py]
dune[dune.py] --> robot[robot.py]
```

**图源**
- [neupan.py](file://NeuPAN/neupan/neupan.py#L1-L402)
- [pan.py](file://NeuPAN/neupan/blocks/pan.py#L1-L272)
- [nrmp.py](file://NeuPAN/neupan/blocks/nrmp.py#L1-L325)
- [dune.py](file://NeuPAN/neupan/blocks/dune.py#L1-L251)
- [initial_path.py](file://NeuPAN/neupan/blocks/initial_path.py#L1-L483)
- [robot.py](file://NeuPAN/neupan/robot/robot.py#L1-L349)

**章节源**
- [neupan.py](file://NeuPAN/neupan/neupan.py#L1-L402)
- [pan.py](file://NeuPAN/neupan/blocks/pan.py#L1-L272)
- [nrmp.py](file://NeuPAN/neupan/blocks/nrmp.py#L1-L325)

## 性能考虑

PAN系统在设计时考虑了多个性能优化方面，包括计算效率、内存使用和实时性要求。系统通过多种机制确保在复杂环境中的高效运行。

1. **点云降采样**: 当障碍物点云数量超过设定阈值时，系统自动进行降采样处理
2. **无梯度推理**: DUNE模块在前向传播时使用`torch.no_grad()`上下文管理器
3. **迭代收敛**: 系统通过`stop_criteria`方法判断迭代收敛，避免不必要的计算
4. **批量处理**: 系统将所有时间步的点云合并后一次性处理，提高计算效率

**章节源**
- [pan.py](file://NeuPAN/neupan/blocks/pan.py#L100-L120)
- [dune.py](file://NeuPAN/neupan/blocks/dune.py#L60-L80)

## 故障排除指南

在使用PAN系统时可能遇到一些常见问题，以下提供相应的解决方案。

**章节源**
- [dune.py](file://NeuPAN/neupan/blocks/dune.py#L200-L250)
- [neupan.py](file://NeuPAN/neupan/neupan.py#L300-L350)

### 模型检查点问题

当系统无法找到DUNE模型检查点时，会提示用户选择是否立即训练模型：

```python
def ask_to_train(self):
    while True:
        choice = input("Do not find the DUNE model; Do you want to train the model now, input Y or N:").upper()
        if choice == 'Y':
            return True
        elif choice == 'N':
            print('Please set the your model path for the DUNE layer.')
            sys.exit()
        else:
            print("Wrong input, Please input Y or N.")
```

### 碰撞检测

系统通过`min_distance`属性和`collision_threshold`参数实现碰撞检测：

```python
def check_stop(self):
    return self.min_distance < self.collision_threshold
```

## 结论

PAN路径规划注意力网络通过创新的编码器-解码器架构和注意力机制，实现了高效、安全的路径规划。系统将深度学习与传统优化方法相结合，在复杂动态环境中表现出色。通过合理调整超参数，系统可适应不同场景的需求，为自动驾驶和机器人导航提供了可靠的解决方案。