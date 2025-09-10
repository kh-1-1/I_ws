# I_ws 路径规划与自主导航算法仓库

本仓库汇总了多种路径规划、运动控制与机器学习相关算法模块（Python/ROS/C++/Julia 混合）。包含前沿的端到端学习规划器、优化方法、经典路径规划算法及仿真工具。

## 📋 项目简介

此仓库整合了多个研究级的路径规划与自主导航系统，涵盖从传统优化方法到最新的深度学习规划器。每个模块都是独立的子项目，具有完整的文档和示例。

## 🗂️ 目录结构

### 核心规划器

#### [NeuPAN](NeuPAN/) - 端到端神经规划器
**技术栈**: Python 3.10+, PyTorch, CVXPY  
**特点**: 首个直接映射障碍物点到控制指令的端到端MPC规划器  
**性能**: 实时运行（>15Hz），无需地图，支持任意凸形机器人  
**引用**: IEEE Transactions on Robotics 2025

**快速开始**:
```bash
cd NeuPAN
pip install -e .
python example/run_exp.py -e corridor -d acker
```

#### [RDA-planner](RDA-planner/) - 加速碰撞避免规划器
**技术栈**: Python 3.9+, NumPy, IR-SIM  
**特点**: 基于ADMM的并行碰撞避免优化，支持动态障碍物  
**性能**: 实时处理复杂环境，支持多种机器人运动学模型  
**引用**: IEEE Robotics and Automation Letters 2023

**快速开始**:
```bash
cd RDA-planner
pip install -e .
pip install ir-sim
python example/corridor.py
```

#### [OBCA](OBCA/) - 优化碰撞避免框架
**技术栈**: Julia 1.6+  
**特点**: 通用碰撞避免约束的平滑重构，支持四旋翼和自动驾驶停车  
**应用**: 高质量轨迹生成，满足系统动力学和安全约束  
**引用**: arXiv:1711.03449

**快速开始**:
```bash
cd OBCA
julia --project=AutonomousParking AutonomousParking/main.jl
```

### 经典路径规划

#### [hybrid_astar_planner](hybrid_astar_planner/) - 混合A*算法
**技术栈**: C++17, Python绑定  
**特点**: 考虑车辆运动学的路径规划，支持Reeds-Shepp曲线  
**性能**: 毫秒级响应（简单场景1ms，复杂场景3s）  
**引用**: Stanford AI Lab, "Practical Search Techniques in Path Planning"

**快速开始**:
```bash
cd hybrid_astar_planner
./build.sh
python3 HybridAStar/hybrid_astar.py
```

#### [teb_local_planner](teb_local_planner/) - 时序弹性带规划器
**技术栈**: C++14, ROS  
**特点**: ROS导航栈插件，优化轨迹执行时间和避障能力  
**应用**: 2D移动机器人局部路径规划  
**引用**: IEEE IROS 2017

**快速开始**:
```bash
# ROS环境下
rosdep install teb_local_planner
catkin_make
```

### 学习规划方法

#### [mpc-mpnet-py](mpc-mpnet-py/) - 模型预测运动规划网络
**技术栈**: Python 3.8+, PyTorch, TensorFlow  
**特点**: 结合MPC和深度学习的快速近优规划  
**支持系统**: 倒立摆、小车、四旋翼、汽车等  
**引用**: arXiv:2101.06798

**快速开始**:
```bash
cd mpc-mpnet-py
cd deps && make
cd ../data_gen
bash datagen_car_batch1.sh
```

#### [MPNet](MPNet/) - 神经运动规划器
**技术栈**: Python 3.7+, PyTorch  
**特点**: 基于学习的运动规划，支持高维状态空间  
**应用**: 复杂环境中的快速路径规划  
**引用**: IEEE IROS 2020

### 群体机器人

#### [CrowdNav](CrowdNav/) - 人群导航强化学习
**技术栈**: Python 3.7+, PyTorch, CrowdSim  
**特点**: 密集人群中的机器人导航，强化学习方法  
**场景**: 商场、机场等拥挤环境

#### [AEMCARL](AEMCARL/) - 自适应人群规避
**技术栈**: Python 3.7+, ROS  
**特点**: 基于注意力机制的人群导航  
**应用**: 服务机器人在动态人群中的导航

### 其他工具

#### [H-OBCA](H-OBCA/) - 混合停车规划器
**技术栈**: Julia 1.6+  
**特点**: 专为自动驾驶停车场景优化的OBCA变体  
**功能**: 平行泊车、倒车入库等复杂停车场景

#### [Dftpav](Dftpav/) - 分布式轨迹规划验证
**技术栈**: C++14, ROS  
**特点**: 分布式多机器人轨迹规划验证框架

#### [field_local_planner](field_local_planner/) - 场地局部规划器
**技术栈**: C++14, ROS  
**特点**: 专为农业/工业场地设计的局部路径规划

## 🚀 环境依赖与构建

### 系统要求
- **操作系统**: Ubuntu 20.04+ / Windows 10+ / macOS 10.15+
- **Python**: 3.8-3.10（根据具体模块要求）
- **ROS**: Noetic（ROS1）或 ROS2 Foxy+（如使用ROS模块）
- **Julia**: 1.6+（用于OBCA相关模块）
- **编译器**: GCC 9+ 或 Clang 12+

### 通用安装步骤

#### 1. Python环境准备
```bash
# 创建虚拟环境
python -m venv planning_env
source planning_env/bin/activate  # Linux/Mac
# 或
planning_env\Scripts\activate     # Windows

# 升级基础工具
pip install --upgrade pip setuptools wheel
```

#### 2. ROS环境配置（可选）
```bash
# 安装ROS Noetic（Ubuntu 20.04）
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install ros-noetic-desktop-full
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

#### 3. Julia环境（可选）
```bash
# 安装Julia
wget https://julialang-s3.julialang.org/bin/linux/x64/1.6/julia-1.6.7-linux-x86_64.tar.gz
tar -xvzf julia-1.6.7-linux-x86_64.tar.gz
sudo cp -r julia-1.6.7 /opt/
sudo ln -s /opt/julia-1.6.7/bin/julia /usr/local/bin/julia
```

### 模块特定安装

#### NeuPAN安装
```bash
cd NeuPAN
pip install -e .
# 安装依赖
pip install torch torchvision cvxpy ir-sim
```

#### ROS模块构建
```bash
# 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
cp -r /path/to/teb_local_planner .
cd ..
catkin_make
source devel/setup.bash
```

#### C++模块构建
```bash
cd hybrid_astar_planner
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## 📊 性能基准

| 模块 | 场景复杂度 | 运行频率 | 内存占用 | 备注 |
|---|---|---|---|---|
| NeuPAN | 中等 | 15-20 Hz | ~500MB | CPU优化 |
| RDA-planner | 高 | 10-15 Hz | ~200MB | 并行计算 |
| hybrid_astar | 中等 | 1-10 Hz | ~50MB | 启发式搜索 |
| teb_local_planner | 低-中 | 20-40 Hz | ~100MB | ROS优化 |

## 🎯 使用建议

### 选择指南
- **实时性要求高**: NeuPAN、RDA-planner
- **复杂动力学约束**: hybrid_astar、OBCA
- **ROS集成**: teb_local_planner、RDA-planner
- **学习型规划**: NeuPAN、MPNet、MPC-MPNet
- **群体机器人**: CrowdNav、AEMCARL

### 开发规范
1. **代码风格**: 遵循PEP8（Python）、Google C++ Style Guide
2. **测试**: 每个模块应包含单元测试和集成测试
3. **文档**: 新功能必须包含文档和示例
4. **提交**: 使用清晰的commit message，禁止直接push到main分支

## 📚 学习资源

### 论文推荐
- [NeuPAN: Direct Point Robot Navigation with End-to-End Model-based Learning](https://ieeexplore.ieee.org/abstract/document/10938329)
- [RDA: An Accelerated Collision Free Motion Planner](https://arxiv.org/pdf/2210.00192.pdf)
- [OBCA: Optimization-Based Collision Avoidance](http://arxiv.org/abs/1711.03449)
- [MPC-MPNet: Model-Predictive Motion Planning Networks](https://arxiv.org/abs/2101.06798)

### 视频教程
- [NeuPAN演示视频](https://youtu.be/SdSLWUmZZgQ)
- [RDA-planner演示](https://www.youtube.com/watch?v=qUNMQQRhNFo)
- [TEB Local Planner教程](http://www.youtube.com/watch?v=e1Bw6JOgHME)

## 🤝 贡献指南

我们欢迎社区贡献！请遵循以下步骤：

1. Fork本仓库
2. 创建功能分支 (`git checkout -b feature/amazing-feature`)
3. 提交更改 (`git commit -m 'Add amazing feature'`)
4. 推送到分支 (`git push origin feature/amazing-feature`)
5. 创建Pull Request

## 📄 许可证

各子模块保持其原始许可证：
- NeuPAN, RDA-planner: MIT License
- OBCA, H-OBCA: MIT License
- teb_local_planner: BSD License
- hybrid_astar_planner: Apache 2.0 License

## 📞 支持与联系

如有问题，请：
1. 先查阅各子模块的README和文档
2. 在对应子模块的GitHub仓库提交Issue
3. 联系维护者：通过GitHub Issues或Pull Requests

## 🙏 致谢

感谢所有原作者和研究团队的开源贡献，使得这些先进的算法能够被更广泛地使用和研究。

---

**注意**: 本仓库为研究代码集合，生产环境使用前请充分测试和验证。