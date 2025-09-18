# C++编码规范

<cite>
**本文档中引用的文件**  
- [teb_local_planner_ros.h](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h)
- [base_local_planner.hpp](file://field_local_planner/field_local_planner_base/field_local_planner_base/include/field_local_planner_base/base_local_planner.hpp)
- [base_local_planner.cpp](file://field_local_planner/field_local_planner_base/field_local_planner_base/src/field_local_planner_base/base_local_planner.cpp)
- [base_plugin.cpp](file://field_local_planner/field_local_planner_base/field_local_planner_base_plugin/src/field_local_planner_base_plugin/base_plugin.cpp)
</cite>

## 目录
1. [命名规范](#命名规范)
2. [头文件包含顺序](#头文件包含顺序)
3. [头文件保护宏](#头文件保护宏)
4. [类的声明与实现分离](#类的声明与实现分离)
5. [const正确性](#const正确性)
6. [智能指针的使用](#智能指针的使用)
7. [注释风格与代码组织](#注释风格与代码组织)

## 命名规范

在C++编码中，遵循一致的命名规范有助于提高代码的可读性和可维护性。根据teb_local_planner和field_local_planner_base中的实现，推荐以下命名约定：

- **类名**：采用PascalCase（驼峰式大写），即每个单词首字母大写，无下划线。例如：`TebLocalPlannerROS`、`BaseLocalPlanner`。
- **变量名和函数名**：采用camelCase（小驼峰式），即第一个单词首字母小写，后续单词首字母大写。例如：`computeVelocityCommands`、`robot_pose_`。
- **成员变量**：通常以`_`结尾，以区分局部变量。例如：`costmap_ros_`、`planner_`。
- **常量**：使用全大写字母，单词间用下划线分隔。例如：`MAX_LINEAR_VELOCITY_X`。
- **枚举类型**：采用PascalCase，枚举值采用全大写。例如：`State { NOT_READY, FINISHED, EXECUTING, FAILURE }`。

**示例代码路径**：
- [TebLocalPlannerROS类定义](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h#L78-L126)
- [BaseLocalPlanner类定义](file://field_local_planner/field_local_planner_base/field_local_planner_base/include/field_local_planner_base/base_local_planner.hpp#L89-L133)

**本节来源**
- [teb_local_planner_ros.h](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h#L78-L126)
- [base_local_planner.hpp](file://field_local_planner/field_local_planner_base/field_local_planner_base/include/field_local_planner_base/base_local_planner.hpp#L89-L133)

## 头文件包含顺序

头文件的包含顺序应遵循一定的逻辑层次，以减少依赖冲突并提高编译效率。根据代码库中的实践，推荐以下包含顺序：

1. **对应头文件**：每个`.cpp`文件应首先包含其对应的`.hpp`或`.h`文件。
2. **C系统库**：如`<stdio.h>`、`<stdlib.h>`等。
3. **C++标准库**：如`<vector>`、`<string>`、`<memory>`等。
4. **第三方库**：如`<boost/shared_ptr.hpp>`、`<gtsam/geometry/Pose3.h>`等。
5. **项目内其他头文件**：按模块或功能分组，使用引号包含。

**示例代码路径**：
- [teb_local_planner_ros.h中的包含顺序](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h#L1-L77)
- [base_local_planner.hpp中的包含顺序](file://field_local_planner/field_local_planner_base/field_local_planner_base/include/field_local_planner_base/base_local_planner.hpp#L1-L88)

**本节来源**
- [teb_local_planner_ros.h](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h#L1-L77)
- [base_local_planner.hpp](file://field_local_planner/field_local_planner_base/field_local_planner_base/include/field_local_planner_base/base_local_planner.hpp#L1-L88)

## 头文件保护宏

为防止头文件被多次包含，必须使用头文件保护宏（Include Guards）。推荐使用`#ifndef`、`#define`、`#endif`结构，并确保宏名唯一且具有描述性。

宏名应基于文件路径全大写，并用下划线分隔。例如，对于`teb_local_planner_ros.h`，保护宏为：

```cpp
#ifndef TEB_LOCAL_PLANNER_ROS_H_
#define TEB_LOCAL_PLANNER_ROS_H_
...
#endif // TEB_LOCAL_PLANNER_ROS_H_
```

**示例代码路径**：
- [teb_local_planner_ros.h中的头文件保护宏](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h#L78-L79)
- [base_local_planner.hpp中的头文件保护宏](file://field_local_planner/field_local_planner_base/field_local_planner_base/include/field_local_planner_base/base_local_planner.hpp#L1-L88)

**本节来源**
- [teb_local_planner_ros.h](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h#L78-L79)
- [base_local_planner.hpp](file://field_local_planner/field_local_planner_base/field_local_planner_base/include/field_local_planner_base/base_local_planner.hpp#L1-L88)

## 类的声明与实现分离

遵循声明与实现分离的原则，将类的接口（成员函数声明、成员变量）放在头文件（`.hpp`或`.h`）中，而将函数的具体实现放在源文件（`.cpp`）中。这有助于减少编译依赖，提高编译速度。

例如，`BaseLocalPlanner`类在`base_local_planner.hpp`中声明了`execute`、`setGoalInFixed`等函数，在`base_local_planner.cpp`中实现。

**示例代码路径**：
- [BaseLocalPlanner类声明](file://field_local_planner/field_local_planner_base/field_local_planner_base/include/field_local_planner_base/base_local_planner.hpp#L89-L133)
- [BaseLocalPlanner类实现](file://field_local_planner/field_local_planner_base/field_local_planner_base/src/field_local_planner_base/base_local_planner.cpp#L27-L78)

**本节来源**
- [base_local_planner.hpp](file://field_local_planner/field_local_planner_base/field_local_planner_base/include/field_local_planner_base/base_local_planner.hpp#L89-L133)
- [base_local_planner.cpp](file://field_local_planner/field_local_planner_base/field_local_planner_base/src/field_local_planner_base/base_local_planner.cpp#L27-L78)

## const正确性

`const`关键字用于表明函数不会修改对象的状态，提高代码的安全性和可读性。成员函数若不修改类的成员变量，应声明为`const`。

在`teb_local_planner_ros.h`中，`transformGlobalPlan`和`estimateLocalGoalOrientation`函数均声明为`const`，表示这些函数不会改变`TebLocalPlannerROS`对象的状态。

```cpp
bool transformGlobalPlan(...) const;
double estimateLocalGoalOrientation(...) const;
```

**示例代码路径**：
- [transformGlobalPlan const函数](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h#L268-L278)
- [estimateLocalGoalOrientation const函数](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h#L280-L292)

**本节来源**
- [teb_local_planner_ros.h](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h#L268-L292)

## 智能指针的使用

智能指针用于自动管理动态内存，防止内存泄漏。推荐使用`std::shared_ptr`和`boost::shared_ptr`来管理共享所有权的对象。

在`teb_local_planner_ros.h`中，`costmap_converter_`和`dynamic_recfg_`均使用`boost::shared_ptr`：

```cpp
boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_;
boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg_;
```

在`field_local_planner_base`中，`point_cloud_`使用`pcl::PointCloud<PointType>::Ptr`，其本质也是`shared_ptr`。

**示例代码路径**：
- [teb_local_planner_ros.h中的智能指针](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h#L340-L345)
- [base_local_planner.hpp中的点云指针](file://field_local_planner/field_local_planner_base/field_local_planner_base/include/field_local_planner_base/base_local_planner.hpp#L133-L134)

**本节来源**
- [teb_local_planner_ros.h](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h#L340-L345)
- [base_local_planner.hpp](file://field_local_planner/field_local_planner_base/field_local_planner_base/include/field_local_planner_base/base_local_planner.hpp#L133-L134)

## 注释风格与代码组织

良好的注释风格能显著提升代码的可维护性。推荐使用Doxygen风格的注释，对类、函数、参数和返回值进行详细说明。

- **类注释**：使用`@class`描述类的功能，`@brief`提供简要说明。
- **函数注释**：使用`@brief`描述功能，`@param`说明参数，`@return`说明返回值。
- **成员变量**：在变量声明前使用`//!<`进行简要说明。

代码组织应遵循逻辑分组，使用`public`、`protected`、`private`明确访问权限，并通过空行和注释分隔不同功能模块。

**示例代码路径**：
- [TebLocalPlannerROS类的Doxygen注释](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h#L78-L84)
- [computeVelocityCommands函数的参数注释](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h#L140-L148)
- [成员变量的//!<注释](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h#L330-L331)

**本节来源**
- [teb_local_planner_ros.h](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h#L78-L84)
- [teb_local_planner_ros.h](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h#L140-L148)
- [teb_local_planner_ros.h](file://teb_local_planner/include/teb_local_planner/teb_local_planner_ros.h#L330-L331)