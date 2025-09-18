# TEB与混合A* 安装与配置

<cite>
**本文档中引用的文件**  
- [teb_local_planner/CMakeLists.txt](file://teb_local_planner/CMakeLists.txt)
- [teb_local_planner/package.xml](file://teb_local_planner/package.xml)
- [teb_local_planner/cfg/TebLocalPlannerReconfigure.cfg](file://teb_local_planner/cfg/TebLocalPlannerReconfigure.cfg)
- [hybrid_astar_planner/CMakeLists.txt](file://hybrid_astar_planner/CMakeLists.txt)
- [hybrid_astar_planner/build.sh](file://hybrid_astar_planner/build.sh)
</cite>

## 目录
1. [简介](#简介)
2. [项目结构](#项目结构)
3. [TEB局部规划器构建配置](#teb局部规划器构建配置)
4. [混合A*规划器构建配置](#混合a*规划器构建配置)
5. [第三方依赖库安装](#第三方依赖库安装)
6. [参数文件配置](#参数文件配置)
7. [ROS环境集成与使用](#ros环境集成与使用)
8. [常见编译错误与解决方案](#常见编译错误与解决方案)
9. [故障排查指南](#故障排查指南)

## 简介
本文档详细说明如何安装和配置TEB局部规划器和混合A*规划器。涵盖使用catkin_make或colcon构建C++/Python混合项目的完整流程，包括G2O、SUITESPARSE等关键第三方依赖库的安装与配置。文档深入解析CMakeLists.txt中的构建选项和package.xml中的ROS依赖声明，提供build.sh脚本的使用方法和编译过程中的常见问题解决方案。同时，说明如何在ROS环境中配置和使用这两个规划器，并提供参数调整和故障排查的完整指南。

## 项目结构
项目包含两个核心规划器：TEB局部规划器和混合A*规划器。TEB规划器位于`teb_local_planner`目录，采用C++实现，依赖ROS导航栈和g2o优化库。混合A*规划器位于`hybrid_astar_planner`目录，采用C++与Python混合架构，使用Qt和Eigen库进行开发。两个规划器均包含完整的构建配置文件和参数定义。

**Section sources**
- [teb_local_planner/CMakeLists.txt](file://teb_local_planner/CMakeLists.txt)
- [hybrid_astar_planner/CMakeLists.txt](file://hybrid_astar_planner/CMakeLists.txt)

## TEB局部规划器构建配置
TEB局部规划器的构建配置通过CMakeLists.txt文件定义。该文件首先声明项目名称和最低CMake版本要求，并设置构建类型为Release以优化性能。通过find_package命令查找ROS核心依赖，包括base_local_planner、costmap_2d、dynamic_reconfigure等。特别地，该规划器依赖SUITESPARSE和G2O两个关键第三方优化库，通过自定义的FindSUITESPARSE.cmake和FindG2O.cmake模块进行查找。

构建系统要求C++11标准支持，这是由于g2o库从ROS Jade版本开始的强制要求。CMakeLists.txt通过检查编译器标志自动启用C++11支持。项目使用catkin_package宏声明其包含目录、库和依赖项，确保其他ROS包能正确链接和使用TEB规划器。

**Section sources**
- [teb_local_planner/CMakeLists.txt](file://teb_local_planner/CMakeLists.txt#L1-L285)

## 混合A*规划器构建配置
混合A*规划器的构建配置同样通过CMakeLists.txt文件管理。该项目要求CMake 3.10或更高版本，并明确设置C++标准为C++14。构建系统启用Qt5的自动元对象编译（AUTOMOC）和用户界面编译（AUTOUIC）功能，以支持Qt框架的信号槽机制和UI文件处理。

项目查找Qt5Widgets、Qt5Core、Qt5Gui和Eigen3库。如果Eigen3未找到，则回退到旧版Eigen查找机制。构建系统定义了优化标志-march=native和-O3以最大化性能。核心库HybridAStar被构建为共享库，链接Qt5的Widgets和Gui组件，包含所有必要的头文件和源文件。

**Section sources**
- [hybrid_astar_planner/CMakeLists.txt](file://hybrid_astar_planner/CMakeLists.txt#L1-L38)

## 第三方依赖库安装
### G2O和SUITESPARSE安装
TEB规划器依赖g2o（通用图优化库）和SUITESPARSE（稀疏矩阵计算库）。这些依赖可通过系统包管理器安装：
```bash
sudo apt-get install libg2o-dev libsuitesparse-dev
```
在teb_local_planner的cmake_modules目录中，FindG2O.cmake和FindSUITESPARSE.cmake文件提供了CMake查找这些库的自定义逻辑。确保这些库正确安装后，CMake能够自动定位其头文件和库文件。

### Qt和Eigen安装
混合A*规划器依赖Qt5和Eigen库。build.sh脚本提供了完整的依赖安装命令：
```bash
sudo apt-get install -y build-essential qtcreator qt5-default libeigen3-dev clang cmake
```
该命令安装了构建工具链、Qt开发环境和Eigen数学库。Eigen是一个头文件-only库，安装后即可通过include_directories直接使用。

**Section sources**
- [teb_local_planner/cmake_modules/FindG2O.cmake](file://teb_local_planner/cmake_modules/FindG2O.cmake)
- [teb_local_planner/cmake_modules/FindSUITESPARSE.cmake](file://teb_local_planner/cmake_modules/FindSUITESPARSE.cmake)
- [hybrid_astar_planner/build.sh](file://hybrid_astar_planner/build.sh#L1-L12)

## 参数文件配置
### TEB规划器参数
TEB规划器的可调参数定义在cfg/TebLocalPlannerReconfigure.cfg文件中，支持动态重配置。参数分为多个逻辑组：
- **轨迹参数**：控制轨迹自适应大小、时间分辨率和全局路径处理
- **机器人参数**：定义最大速度、加速度和转向半径等运动学约束
- **目标容差**：设置到达目标时的位置和方向容差
- **障碍物参数**：配置与障碍物的最小安全距离和膨胀距离
- **优化参数**：调整各种约束的优化权重和迭代次数

这些参数可通过dynamic_reconfigure在运行时调整，无需重启节点。

### 混合A*规划器参数
混合A*规划器的参数主要通过Python脚本的配置字典进行设置，包括搜索分辨率、车辆模型参数和启发式函数权重等。具体参数定义在HybridAStar/hybrid_astar.py文件中。

**Section sources**
- [teb_local_planner/cfg/TebLocalPlannerReconfigure.cfg](file://teb_local_planner/cfg/TebLocalPlannerReconfigure.cfg#L1-L448)

## ROS环境集成与使用
### TEB规划器集成
TEB规划器作为base_local_planner的插件集成到ROS导航栈中。在package.xml中声明了对nav_core、pluginlib等核心ROS包的依赖。通过export标签注册插件：
```xml
<export>
  <nav_core plugin="${prefix}/teb_local_planner_plugin.xml"/>
</export>
```
在move_base的配置文件中，将local_planner设为teb_local_planner/TEBLocalPlannerROS即可启用。

### 混合A*规划器使用
混合A*规划器通过Python包装器hybrid_astar_wrapper.py提供ROS接口。使用时需确保C++库已正确编译并生成.so文件，Python脚本会通过ctypes或pybind11加载该库。

**Section sources**
- [teb_local_planner/package.xml](file://teb_local_planner/package.xml#L1-L58)

## 常见编译错误与解决方案
### C++编译错误
1. **C++11支持错误**：确保编译器支持C++11，GCC 4.8+或Clang 3.3+。在CMakeLists.txt中检查COMPILER_SUPPORTS_CXX11标志。
2. **g2o库链接错误**：确认libg2o-dev已安装，并检查CMAKE_MODULE_PATH是否包含FindG2O.cmake。
3. **Eigen头文件找不到**：确保libeigen3-dev已安装，或手动设置Eigen_INCLUDE_DIRS。

### ROS包依赖缺失
1. **catkin依赖未找到**：运行`rosdep install --from-paths . --ignore-src`安装所有ROS依赖。
2. **dynamic_reconfigure生成错误**：确保在CMakeLists.txt中正确调用generate_dynamic_reconfigure_options，并添加依赖关系。
3. **pluginlib注册失败**：检查plugin.xml文件路径是否正确，并确保在package.xml中正确声明export。

## 故障排查指南
### 构建过程问题
- **权限错误**：build.sh脚本使用sudo安装系统包，确保有管理员权限。
- **Qt版本冲突**：如果系统有多个Qt版本，使用qtchooser指定Qt5。
- **Eigen版本问题**：某些系统可能同时安装Eigen2和Eigen3，确保CMake找到的是Eigen3。

### 运行时问题
- **动态重配置无法连接**：检查roscore是否运行，并使用`rosrun rqt_reconfigure rqt_reconfigure`测试连接。
- **规划器不响应**：验证TF树是否完整，特别是/base_link到/map的变换。
- **性能瓶颈**：对于TEB规划器，减少max_number_classes和no_outer_iterations以降低计算负载。

**Section sources**
- [teb_local_planner/CMakeLists.txt](file://teb_local_planner/CMakeLists.txt#L1-L285)
- [hybrid_astar_planner/build.sh](file://hybrid_astar_planner/build.sh#L1-L12)