# mpc-mpnet-py 安装与配置

<cite>
**本文档引用的文件**
- [README.md](file://mpc-mpnet-py/README.md)
- [datagen_quadrotor_obs_batch1.sh](file://mpc-mpnet-py/data_gen/datagen_quadrotor_obs_batch1.sh)
- [mp_path_default.py](file://mpc-mpnet-py/params/car_obs/mp_path_default.py)
- [mp_path_default.py](file://mpc-mpnet-py/params/quadrotor_obs/mp_path_default.py)
- [CMakeLists.txt](file://mpc-mpnet-py/mpnet/exported/CMakeLists.txt)
- [env.yaml](file://AEMCARL/env.yaml)
</cite>

## 目录
1. [简介](#简介)
2. [深度学习环境配置](#深度学习环境配置)
3. [项目依赖项安装](#项目依赖项安装)
4. [参数配置说明](#参数配置说明)
5. [数据生成与训练流程](#数据生成与训练流程)
6. [常见问题与解决方案](#常见问题与解决方案)
7. [结论](#结论)

## 简介
MPC-MPNet 是一种基于模型预测的运动规划网络，旨在实现快速、近最优的运动规划。本项目 `mpc-mpnet-py` 提供了 MPC-MPNet 的 Python 实现，支持多种机器人模型（如 car_obs、quadrotor_obs 等）的路径规划与树搜索算法。本文档详细说明了如何配置深度学习环境、安装项目依赖、设置参数、生成数据以及解决常见问题。

**Section sources**
- [README.md](file://mpc-mpnet-py/README.md)

## 深度学习环境配置
为了运行 `mpc-mpnet-py` 项目，需要配置合适的深度学习环境，包括 PyTorch、CUDA 和 cuDNN。根据项目依赖文件 `env.yaml` 中的信息，推荐使用以下版本组合：

- **PyTorch**: 1.8.0+cu111
- **CUDA**: 11.1
- **cuDNN**: 与 CUDA 11.1 兼容的版本

确保系统中已安装 NVIDIA 驱动，并通过 `nvidia-smi` 命令验证 GPU 可用性。然后使用 Conda 或 Pip 安装指定版本的 PyTorch：

```bash
# 使用 Conda 安装
conda install pytorch==1.8.0 torchvision==0.9.0 torchaudio==0.8.0 cudatoolkit=11.1 -c pytorch

# 或使用 Pip 安装
pip install torch==1.8.0+cu111 torchvision==0.9.0+cu111 torchaudio==0.8.0+cu111 -f https://download.pytorch.org/whl/torch_stable.html
```

**Section sources**
- [env.yaml](file://AEMCARL/env.yaml)

## 项目依赖项安装
项目依赖项主要通过 `requirements.txt` 或环境文件进行管理。首先确保已激活适当的 Python 环境（如 `linjun`），然后安装必要的 Python 包：

```bash
# 安装项目依赖
pip install -r requirements.txt
```

此外，项目包含 C++ 模块，需编译以支持稀疏 RRT 的 Python 绑定。进入 `deps/` 目录并构建模块：

```bash
cd deps/
# 构建 C++ 模块
```

注意：请确保所有脚本中的 `sys.path` 已更新为本地路径，例如 `benchmark.py` 和 `experiments` 文件夹内的脚本。

**Section sources**
- [README.md](file://mpc-mpnet-py/README.md)

## 参数配置说明
项目中的 `params` 目录包含了不同机器人模型的参数配置文件。每个子目录（如 `car_obs`、`quadrotor_obs`）对应一种机器人类型，其下的 `mp_path_default.py` 等文件定义了具体的规划参数。

### car_obs 参数配置
`car_obs` 模型的参数配置位于 `params/car_obs/mp_path_default.py`，关键参数包括：

- **solver_type**: 使用 CEM（交叉熵方法）作为优化求解器
- **n_sample**: 每次迭代采样 32 条轨迹
- **n_elite**: 保留前 4 条精英轨迹
- **dt**: 时间步长为 2e-3 秒
- **goal_radius**: 目标半径为 2.0
- **device_id**: 指定使用 `cuda:3` 进行计算

### quadrotor_obs 参数配置
`quadrotor_obs` 模型的参数配置位于 `params/quadrotor_obs/mp_path_default.py`，关键参数包括：

- **n_sample**: 每次迭代采样 32 条轨迹
- **max_it**: 最大迭代次数为 5
- **dt**: 时间步长为 2e-3 秒
- **goal_radius**: 目标半径为 2
- **device_id**: 指定使用 `cuda:3` 进行计算
- **max_planning_time**: 最大规划时间为 500 秒

这些参数可根据具体应用场景进行调整，以平衡规划速度与精度。

**Section sources**
- [mp_path_default.py](file://mpc-mpnet-py/params/car_obs/mp_path_default.py)
- [mp_path_default.py](file://mpc-mpnet-py/params/quadrotor_obs/mp_path_default.py)

## 数据生成与训练流程
### 数据生成
数据生成脚本位于 `data_gen` 目录下，支持并行化处理。以 `quadrotor_obs` 为例，使用 `datagen_quadrotor_obs_batch1.sh` 脚本生成数据：

```bash
source activate linjun
python3 data_generation.py --env_name quadrotor_obs --N $1 --NP 32 \
--max_iter 5000000 --path_folder trajectories/quadrotor_obs/ \
--obs_file trajectories/quadrotor_obs/obs.pkl --obc_file trajectories/car_obs/obc.pkl \
--max_time=500
```

该脚本将生成 `quadrotor_obs` 环境下的轨迹数据，并保存至指定路径。

### 模型训练
训练流程包括预处理障碍物和状态-目标对，然后训练网络模型：

1. 使用 `mpnet/sst_envs/process_data.py` 和 `process_obs.py` 预处理数据
2. 使用 `mpnet/train_mpnet.py` 训练主网络
3. 对于 MPC-MPNet-Path 的 costnet，使用 `mpnet/train_costs.py`

**Section sources**
- [README.md](file://mpc-mpnet-py/README.md)
- [datagen_quadrotor_obs_batch1.sh](file://mpc-mpnet-py/data_gen/datagen_quadrotor_obs_batch1.sh)

## 常见问题与解决方案
### CUDA 内存不足
当出现 CUDA 内存不足错误时，可尝试以下解决方案：
- 减少批量大小（batch size）
- 使用更小的模型（如 `mpnet_external_small_model`）
- 清理 GPU 缓存：`torch.cuda.empty_cache()`
- 指定使用其他 GPU 设备（修改 `device_id`）

### 版本不兼容
若遇到 PyTorch 与 CUDA 版本不兼容问题，请严格按照推荐版本安装：
- 检查 PyTorch 官网提供的兼容性矩阵
- 使用 Conda 安装以自动解决依赖冲突
- 确保 `cudatoolkit` 版本与系统 CUDA 驱动匹配

### 编译错误
C++ 模块编译失败可能由于缺少依赖库。检查 `CMakeLists.txt` 中的路径设置，并确保 `libtorch` 和 CUDA 路径正确：

```cmake
list(APPEND CMAKE_PREFIX_PATH
    ${PROJECT_SOURCE_DIR}/../../../external/libtorch
    /usr/local/cuda
)
```

**Section sources**
- [CMakeLists.txt](file://mpc-mpnet-py/mpnet/exported/CMakeLists.txt)
- [env.yaml](file://AEMCARL/env.yaml)

## 结论
本文档详细介绍了 `mpc-mpnet-py` 子项目的安装与配置流程，涵盖深度学习环境搭建、依赖项安装、参数配置、数据生成与训练流程，并提供了常见问题的解决方案。通过遵循本文档，用户可以顺利部署 MPC-MPNet 并进行高效的运动规划实验。