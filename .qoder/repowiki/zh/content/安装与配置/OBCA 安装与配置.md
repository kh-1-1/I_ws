# OBCA 安装与配置

<cite>
**本文档中引用的文件**  
- [setup.jl](file://OBCA/AutonomousParking/setup.jl)
- [main.jl](file://OBCA/AutonomousParking/main.jl)
- [setupQuadcopter.jl](file://OBCA/QuadcopterNavigation/setupQuadcopter.jl)
- [mainQuadcopter.jl](file://OBCA/QuadcopterNavigation/mainQuadcopter.jl)
- [README.md](file://OBCA/README.md)
</cite>

## 目录
1. [简介](#简介)
2. [Julia环境安装](#julia环境安装)
3. [依赖包管理与安装](#依赖包管理与安装)
4. [应用场景配置差异](#应用场景配置差异)
5. [关键参数设置](#关键参数设置)
6. [跨平台配置注意事项](#跨平台配置注意事项)
7. [常见问题与解决方案](#常见问题与解决方案)
8. [结论](#结论)

## 简介
OBCA（基于优化的碰撞避免）是一种用于自主导航路径规划的先进方法，通过平滑重构碰撞避免约束，使通用非线性优化求解器能够生成高质量路径。本指南重点介绍OBCA子项目中Julia环境的搭建、依赖包管理以及在不同应用场景下的配置方法。

**Section sources**
- [README.md](file://OBCA/README.md#L1-L30)

## Julia环境安装
安装Julia是运行OBCA项目的首要步骤。建议使用官方发布的最新稳定版本以确保兼容性。

### 下载与安装
1. 访问[Julia官网](https://julialang.org/downloads/)下载适用于您操作系统的安装包。
2. 对于Linux系统，可通过以下命令安装：
   ```bash
   wget https://julialang-s3.julialang.org/bin/linux/x64/1.6/julia-1.6.7-linux-x86_64.tar.gz
   tar -xvzf julia-1.6.7-linux-x86_64.tar.gz
   sudo cp -r julia-1.6.7 /opt/
   sudo ln -s /opt/julia-1.6.7/bin/julia /usr/local/bin/julia
   ```
3. Windows用户可直接运行安装程序并将其添加到系统PATH中。

### 验证安装
安装完成后，在终端输入`julia`启动交互式环境，确认版本信息显示正常。

## 依赖包管理与安装
OBCA项目依赖多个Julia包，主要通过`Project.toml`文件或直接命令进行管理。

### 使用Project.toml管理依赖
1. 在项目根目录下创建`Project.toml`文件，声明所需包及其版本。
2. 使用`import Pkg; Pkg.instantiate()`自动安装所有依赖项。

### 直接命令安装
对于OBCA项目，核心依赖包包括JuMP、Ipopt和PyPlot。可通过以下命令安装：
```julia
using Pkg
Pkg.add("JuMP")
Pkg.add("Ipopt")
Pkg.add("PyPlot")
Pkg.add("NearestNeighbors")
```

### setup.jl文件中的依赖加载
在`AutonomousParking`和`QuadcopterNavigation`目录下的`setup.jl`文件中，已明确列出所需包：
- `AutonomousParking/setup.jl`加载了`JuMP, Ipopt, PyPlot, NearestNeighbors`
- `QuadcopterNavigation/setupQuadcopter.jl`加载了`JuMP, Ipopt, PyPlot`

这些文件需在运行主程序前执行，以确保所有模块正确加载。

**Section sources**
- [setup.jl](file://OBCA/AutonomousParking/setup.jl#L1-L53)
- [setupQuadcopter.jl](file://OBCA/QuadcopterNavigation/setupQuadcopter.jl#L1-L36)

## 应用场景配置差异
OBCA提供了两个典型应用场景：自动驾驶泊车（AutonomousParking）和四旋翼导航（QuadcopterNavigation），二者在配置上有显著差异。

### AutonomousParking配置特点
- 使用二维混合A*算法（hybrid_a_star.jl）进行路径搜索
- 车辆动力学模型基于阿克曼转向几何
- 碰撞检测针对多边形障碍物优化
- 主程序为`main.jl`，调用`ParkingDist.jl`和`ParkingSignedDist.jl`实现距离和符号距离方法

### QuadcopterNavigation配置特点
- 使用三维A*算法（a_star_3D.jl）进行空间路径规划
- 无人机模型考虑三维姿态与速度约束
- 环境建模包含多个立方体障碍物
- 主程序为`mainQuadcopter.jl`，调用`QuadcopterDist.jl`和`QuadcopterSignedDist.jl`

两种场景均采用相同的优化框架，但状态空间维度、控制输入和障碍物表示方式不同。

**Section sources**
- [main.jl](file://OBCA/AutonomousParking/main.jl#L1-L289)
- [mainQuadcopter.jl](file://OBCA/QuadcopterNavigation/mainQuadcopter.jl#L1-L158)

## 关键参数设置
`main.jl`文件包含多个关键参数，直接影响规划性能与结果。

### 场景选择
```julia
scenario = "parallel"
# 或
scenario = "backwards"
```
决定使用平行泊车还是倒车泊车场景。

### 时间步长与采样
```julia
Ts = 0.6/3*sampleN  # 变量时间步长
fixTime = 0         # 0表示可变时间，1表示固定时间
```
影响轨迹平滑性和求解复杂度。

### 车辆几何参数
```julia
L = 2.7  # 轴距
ego = [3.7, 1, 1, 1]  # 车辆尺寸 [前, 右, 后, 左]
```
定义车辆外形用于碰撞检测。

### 初始与目标状态
```julia
x0 = [-6, 9.5, 0.0, 0.0]  # 初始位置与朝向
xF = [0, 1.3, pi/2, 0]    # 目标位置与朝向
```
必须符合场景定义的坐标系要求。

**Section sources**
- [main.jl](file://OBCA/AutonomousParking/main.jl#L1-L289)

## 跨平台配置注意事项
在不同操作系统上配置OBCA时需注意以下事项。

### Linux系统
- 确保已安装必要的图形库支持PyPlot后端
- 推荐使用系统包管理器安装Julia以简化依赖处理
- 编译器配置通常无需额外设置

### Windows系统
- 安装过程中需手动将Julia添加到系统PATH
- 可能需要安装Microsoft Visual C++ Redistributable以支持某些原生库
- 图形显示可能需要配置PyCall使用特定Python环境

### 路径设置
Julia对路径分隔符较为敏感，建议使用正斜杠`/`或`joinpath()`函数构建路径，避免因操作系统差异导致错误。

## 常见问题与解决方案
### Julia包依赖解析失败
**问题表现**：`Pkg.resolve()`卡住或报错无法满足版本约束。

**解决方案**：
1. 更新包注册表：`Pkg.update()`
2. 清理缓存：删除`~/.julia/compiled`目录
3. 强制重新实例化：`Pkg.instantiate(; verbose = true)`
4. 检查网络连接，必要时配置代理

### IPOPT求解器无法加载
**问题表现**：`using Ipopt`时报错找不到动态库。

**解决方案**：
1. 确认Ipopt已正确安装：`Pkg.build("Ipopt")`
2. 检查系统是否缺少BLAS/LAPACK库
3. 在Linux上安装`libipopt-dev`系统包

### 图形显示异常
**问题表现**：`PyPlot`无法弹出绘图窗口。

**解决方案**：
1. 设置后端：`ENV["MPLBACKEND"]="qt5agg"`
2. 确保Python环境中已安装matplotlib
3. 使用`pygui(true)`触发GUI事件循环

## 结论
OBCA项目提供了一套完整的基于优化的路径规划解决方案。通过正确配置Julia环境、管理依赖包，并理解不同应用场景的配置差异，用户可以顺利运行自动驾驶泊车和四旋翼导航示例。合理设置`main.jl`中的关键参数，并注意跨平台配置细节，将有助于避免常见问题，确保项目稳定运行。