# NeuPAN 安装与配置

<cite>
**本文档中引用的文件**  
- [pyproject.toml](file://NeuPAN/pyproject.toml)
- [README.md](file://NeuPAN/README.md)
- [planner.yaml](file://NeuPAN/example/corridor/acker/planner.yaml)
- [env.yaml](file://NeuPAN/example/corridor/acker/env.yaml)
- [run_exp.py](file://NeuPAN/example/run_exp.py)
- [setup.py](file://NeuPAN/setup.py)
</cite>

## 目录
1. [简介](#简介)
2. [环境准备](#环境准备)
3. [通过pip安装NeuPAN](#通过pip安装neupan)
4. [依赖项版本要求](#依赖项版本要求)
5. [pyproject.toml配置说明](#pyprojecttoml配置说明)
6. [本地开发安装（setup.py）](#本地开发安装setuppy)
7. [配置文件详解](#配置文件详解)
8. [完整运行流程](#完整运行流程)
9. [常见问题与解决方案](#常见问题与解决方案)
10. [附录](#附录)

## 简介

NeuPAN（Neural Proximal Alternating-minimization Network）是一种端到端、实时、无需地图且易于部署的基于模型预测控制（MPC）的机器人运动规划器。它通过将学习方法与优化技术相结合，直接将障碍物点云数据映射为控制动作，实现实时导航。本指南详细说明NeuPAN子项目的安装、配置及运行流程，涵盖从环境搭建到示例执行的完整步骤。

## 环境准备

在安装NeuPAN之前，请确保系统满足以下基本要求：

- **Python版本**：≥ 3.10（官方推荐）
- **可选Python 3.8支持**：可通过`py38`分支获取兼容版本
- **操作系统**：支持Windows、Linux、macOS
- **硬件建议**：由于NeuPAN依赖CPU进行优化求解，建议使用高性能CPU（如Intel i7）以实现10Hz以上的控制频率

**推荐使用Python虚拟环境进行安装**，以避免依赖冲突：

```bash
# 创建虚拟环境
python -m venv neupan_env

# 激活虚拟环境
# Windows:
neupan_env\Scripts\activate
# Linux/macOS:
source neupan_env/bin/activate
```

**Section sources**
- [README.md](file://NeuPAN/README.md#L25-L27)

## 通过pip安装NeuPAN

NeuPAN可通过`pip`直接安装，支持从GitHub仓库或本地目录安装。

### 从GitHub安装（推荐）

```bash
# 克隆仓库
git clone https://github.com/hanruihua/NeuPAN
cd NeuPAN

# 使用pip安装（开发模式）
pip install -e .
```

此命令将以可编辑模式安装NeuPAN及其所有必需依赖项，便于开发和调试。

### 从PyPI安装（若已发布）

```bash
pip install neupan
```

目前NeuPAN主要通过GitHub分发，建议使用源码安装方式。

**Section sources**
- [README.md](file://NeuPAN/README.md#L30-L35)

## 依赖项版本要求

NeuPAN的依赖项在`pyproject.toml`文件中明确定义，确保安装时版本兼容。

### 核心依赖项

| 依赖包 | 版本要求 | 说明 |
|--------|---------|------|
| torch | >=2.1.0 | PyTorch深度学习框架 |
| numpy | 无 | 数值计算基础库 |
| scipy | <=1.13.0 | 科学计算库（版本限制） |
| cvxpylayers | 无 | 可微优化层支持 |
| ecos | 无 | 凸优化求解器 |
| pyyaml | 无 | YAML配置文件解析 |
| gctl | ==1.2 | 控制工具库 |
| scikit-learn | 无 | 机器学习工具库 |
| rich | 无 | 终端美化输出 |
| dill | 无 | 序列化支持 |
| colorama | 无 | 跨平台颜色输出 |

### 可选依赖项

| 依赖包 | 版本要求 | 用途 |
|--------|---------|------|
| ir-sim | >=2.4.0 | 仿真环境支持（通过`[irsim]`选项安装） |

可通过以下命令安装带可选依赖的完整版本：

```bash
pip install neupan[all]
```

**Section sources**
- [pyproject.toml](file://NeuPAN/pyproject.toml#L6-L22)

## pyproject.toml配置说明

`pyproject.toml`是NeuPAN项目的现代Python构建配置文件，取代了传统的`setup.py`。

### 构建系统配置

```toml
[build-system]
requires = ["setuptools >= 61.0"]
build-backend = "setuptools.build_meta"
```

指定使用`setuptools`作为构建后端，要求版本不低于61.0。

### 项目元数据

```toml
[project]
name = 'neupan'
version = "1.1"
requires-python = ">= 3.10"
authors = [{name = "Han Ruihua", email = "hanrh@connect.hku.hk"}]
description = "NeuPAN"
readme = "README.md"
```

定义项目名称、版本、Python版本要求、作者信息等。

### 依赖项声明

```toml
dependencies = [
    'cvxpylayers',
    'numpy',
    'scipy<=1.13.0',
    'rich',
    'dill',
    'gctl==1.2',
    'colorama',
    'scikit-learn',
    'pyyaml',
    "torch>=2.1.0",
    "ecos",
]
```

列出所有必需依赖项及其版本约束。

### 可选依赖

```toml
[project.optional-dependencies]
irsim = ['ir-sim>=2.4.0']
all = ['ir-sim>=2.4.0']
```

定义可选功能模块的依赖，支持通过`pip install neupan[irsim]`安装。

**Section sources**
- [pyproject.toml](file://NeuPAN/pyproject.toml#L1-L34)

## 本地开发安装（setup.py）

尽管NeuPAN主要使用`pyproject.toml`，但项目结构中仍包含`setup.py`用于兼容性支持。

### 创建setup.py（如需）

若需使用`setup.py`进行本地开发安装，可创建如下文件：

```python
from setuptools import setup, find_packages

setup(
    name='neupan',
    version='1.1',
    packages=find_packages(),
    install_requires=[
        'torch>=2.1.0',
        'numpy',
        'scipy<=1.13.0',
        'cvxpylayers',
        'pyyaml',
        'gctl==1.2',
        'ecos',
        'scikit-learn',
        'rich',
        'dill',
        'colorama'
    ],
    python_requires='>=3.10',
    description='NeuPAN: Neural Proximal Alternating-minimization Network',
    author='Han Ruihua',
    author_email='hanrh@connect.hku.hk'
)
```

### 安装命令

```bash
pip install -e .
```

此命令将项目安装为可编辑包，便于开发调试。

**Section sources**
- [setup.py](file://NeuPAN/setup.py)

## 配置文件详解

NeuPAN使用YAML文件进行参数配置，主要包含`planner.yaml`和`env.yaml`。

### planner.yaml 参数说明

`planner.yaml`定义规划器核心参数。

#### MPC参数
| 参数 | 类型/默认值 | 说明 |
|------|------------|------|
| `receding` | int / 10 | MPC预测步数 |
| `step_time` | float / 0.1 | MPC时间步长（秒） |
| `ref_speed` | float / 4.0 | 参考速度（m/s） |
| `device` | str / "cpu" | 运行设备（cpu/cuda） |
| `time_print` | bool / False | 是否打印前向计算耗时 |

#### 机器人参数（robot）
| 参数 | 类型/默认值 | 说明 |
|------|------------|------|
| `kinematics` | str / "diff" | 运动学类型（diff/acker） |
| `max_speed` | list[float] | 最大速度限制 |
| `max_acce` | list[float] | 最大加速度限制 |
| `length`/`width` | float | 机器人长宽（矩形） |
| `wheelbase` | float | 轴距（阿克曼车辆） |

#### 初始路径参数（ipath）
| 参数 | 类型/默认值 | 说明 |
|------|------------|------|
| `waypoints` | list[list[float]] | 路径航点 |
| `curve_style` | str / "line" | 曲线类型（dubins/reeds） |
| `min_radius` | float | 最小转弯半径 |

#### PAN网络参数（pan）
| 参数 | 类型/默认值 | 说明 |
|------|------------|------|
| `iter_num` | int / 2 | 迭代次数 |
| `dune_max_num` | int / 100 | DUNE层最大障碍点数 |
| `dune_checkpoint` | str | DUNE模型检查点路径 |

#### 调整参数（adjust）
| 参数 | 类型/默认值 | 说明 |
|------|------------|------|
| `q_s` | float / 1.0 | 状态代价权重 |
| `p_u` | float / 1.0 | 速度代价权重 |
| `ro_obs` | float / 400 | 碰撞避免惩罚参数 |

**Section sources**
- [planner.yaml](file://NeuPAN/example/corridor/acker/planner.yaml#L1-L42)
- [README.md](file://NeuPAN/README.md#L60-L200)

### env.yaml 参数说明

`env.yaml`定义仿真环境参数。

#### 世界参数（world）
| 参数 | 类型 | 说明 |
|------|------|------|
| `height`/`width` | float | 世界尺寸 |
| `step_time` | float | 仿真步长 |
| `collision_mode` | str | 碰撞模式（stop/unobstructed） |

#### 机器人配置
| 参数 | 类型 | 说明 |
|------|------|------|
| `state` | list | 初始状态[x, y, θ, v] |
| `goal` | list | 目标状态[x, y, θ] |
| `sensors` | dict | 传感器配置（如激光雷达） |

#### 障碍物配置（obstacle）
| 参数 | 类型 | 说明 |
|------|------|------|
| `number` | int | 障碍物数量 |
| `state` | list | 障碍物位置和朝向 |

**Section sources**
- [env.yaml](file://NeuPAN/example/corridor/acker/env.yaml#L1-L53)

## 完整运行流程

### 1. 环境搭建
```bash
python -m venv neupan_env
source neupan_env/bin/activate  # 或 neupan_env\Scripts\activate
```

### 2. 安装NeuPAN
```bash
git clone https://github.com/hanruihua/NeuPAN
cd NeuPAN
pip install -e .
```

### 3. 安装仿真环境
```bash
pip install ir-sim
```

### 4. 运行示例
```bash
# 运行走廊场景（阿克曼车辆）
python example/run_exp.py -e corridor -d acker

# 运行动态障碍物场景（差速车辆）
python example/run_exp.py -e dyna_obs -d diff -v
```

### 5. 自定义配置
修改`example/`目录下的`planner.yaml`和`env.yaml`文件，调整参数以适应特定场景。

**Section sources**
- [run_exp.py](file://NeuPAN/example/run_exp.py#L1-L94)
- [README.md](file://NeuPAN/README.md#L37-L58)

## 常见问题与解决方案

### 依赖冲突问题
**问题**：`scipy`版本冲突  
**解决方案**：确保安装`scipy<=1.13.0`，使用：
```bash
pip install scipy==1.13.0
```

### GPU支持问题
**问题**：`cvxpy`不支持GPU  
**解决方案**：NeuPAN优化求解在CPU上运行，建议：
- 使用`device: 'cpu'`配置
- 配备高性能CPU（如Intel i7）
- GPU仅用于DUNE模型训练

### DUNE模型训练
**何时需要重新训练**：
- 更改机器人尺寸或形状时
- 使用自定义凸几何体时

**训练命令**：
```bash
# 在dune_train示例中运行
python example/dune_train/dune_train_acker.py
```

### 性能优化建议
- 降低`receding`步数
- 减少`dune_max_num`和`nrmp_max_num`
- 调整`iter_num`和`iter_threshold`
- 使用更简单的初始路径

**Section sources**
- [README.md](file://NeuPAN/README.md#L202-L242)

## 附录

### 示例目录结构
```
example/
├── corridor/          # 走廊场景
├── dyna_obs/         # 动态障碍物
├── pf/               # 路径跟踪
├── reverse/          # 倒车入库
└── dune_train/       # DUNE模型训练
```

### 主要Python模块
- `neupan.neupan`：核心规划器
- `neupan.blocks`：网络模块
- `neupan.robot`：机器人模型
- `irsim`：仿真环境

### 联系方式
如有问题，请联系：hanrh@connect.hku.hk 或提交GitHub Issue。