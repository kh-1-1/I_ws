# Python/C++/Julia编码规范

<cite>
**本文档中引用的文件**  
- [pyproject.toml](file://NeuPAN/pyproject.toml)
- [teb_config.h](file://teb_local_planner/include/teb_local_planner/teb_config.h)
- [robot_footprint_model.h](file://teb_local_planner/include/teb_local_planner/robot_footprint_model.h)
- [main.jl](file://OBCA/AutonomousParking/main.jl)
- [ParkingConstraints.jl](file://OBCA/AutonomousParking/ParkingConstraints.jl)
- [a_star.jl](file://OBCA/AutonomousParking/a_star.jl)
</cite>

## 目录
1. [引言](#引言)
2. [Python编码规范](#python编码规范)
3. [C++编码规范](#c编码规范)
4. [Julia编码规范](#julia编码规范)
5. [跨语言一致性指导](#跨语言一致性指导)
6. [结论](#结论)

## 引言
本文档详细说明了项目中Python、C++和Julia三种主要编程语言的编码规范。针对Python，基于NeuPAN项目的pyproject.toml配置，介绍black格式化规则、flake8检查标准以及命名约定；针对C++，参考teb_local_planner中的代码风格，说明命名规范、头文件包含顺序、类设计原则；针对Julia，分析OBCA项目中的代码示例，阐述函数式编程风格、模块组织方式和性能优化实践。通过提供跨语言一致性指导，确保不同子项目间的代码可读性和维护性。

## Python编码规范

### Black格式化规则
NeuPAN项目采用Black作为代码格式化工具，通过pyproject.toml文件进行配置。Black是一种无需配置的代码格式化程序，它会自动将Python代码格式化为一致的风格。项目中Python代码的格式化遵循Black的默认规则，包括每行最大长度为88个字符（Black默认值），使用单引号表示字符串，以及在函数定义和类定义之间插入两个空行等。

**Section sources**
- [pyproject.toml](file://NeuPAN/pyproject.toml)

### Flake8检查标准
项目使用Flake8进行代码质量检查，确保代码符合PEP 8编码规范。Flake8检查包括代码风格、复杂度和潜在错误。通过在开发过程中集成Flake8检查，可以及时发现并修复代码中的问题，提高代码质量和可维护性。

**Section sources**
- [pyproject.toml](file://NeuPAN/pyproject.toml)

### 命名约定
Python代码遵循PEP 8命名约定，使用小写字母和下划线分隔单词的方式命名变量和函数（snake_case），使用驼峰式命名法（CamelCase）命名类。模块和包名应简短且全小写，避免使用下划线。常量名应全大写，单词间用下划线分隔。

**Section sources**
- [pyproject.toml](file://NeuPAN/pyproject.toml)

## C++编码规范

### 命名规范
C++代码遵循Google C++命名规范。类名和类型名使用驼峰式命名法（CamelCase），函数名和变量名使用小写字母和下划线分隔（snake_case）。常量名以k开头，后跟驼峰式命名。枚举值全大写，单词间用下划线分隔。成员变量以_结尾，静态变量以s_开头。

**Section sources**
- [teb_config.h](file://teb_local_planner/include/teb_local_planner/teb_config.h)
- [robot_footprint_model.h](file://teb_local_planner/include/teb_local_planner/robot_footprint_model.h)

### 头文件包含顺序
头文件包含顺序遵循特定规则：首先是对应的头文件（如果存在），然后是C库、C++库、其他库的头文件，最后是本项目内的头文件。每个类别内部按字母顺序排列。这种顺序有助于发现遗漏的包含和循环依赖问题。

**Section sources**
- [teb_config.h](file://teb_local_planner/include/teb_local_planner/teb_config.h)
- [robot_footprint_model.h](file://teb_local_planner/include/teb_local_planner/robot_footprint_model.h)

### 类设计原则
C++类设计遵循单一职责原则和开闭原则。类的接口设计清晰，成员函数尽量小而专注。使用const关键字标记不修改对象状态的成员函数。对于需要多态行为的类，使用虚函数，并在基类中声明虚析构函数。智能指针（如shared_ptr）用于管理动态分配的对象生命周期。

**Section sources**
- [teb_config.h](file://teb_local_planner/include/teb_local_planner/teb_config.h)
- [robot_footprint_model.h](file://teb_local_planner/include/teb_local_planner/robot_footprint_model.h)

## Julia编码规范

### 函数式编程风格
Julia代码采用函数式编程风格，强调不可变数据和纯函数。函数设计遵循单一职责原则，每个函数只完成一个明确的任务。高阶函数和匿名函数被广泛使用，以提高代码的表达力和复用性。避免使用全局变量，函数参数通过值传递或引用传递明确指定。

**Section sources**
- [main.jl](file://OBCA/AutonomousParking/main.jl)
- [a_star.jl](file://OBCA/AutonomousParking/a_star.jl)

### 模块组织方式
Julia代码通过模块（module）组织，每个模块封装相关的函数和类型。模块名使用大写字母开头的驼峰式命名法。模块内部使用export关键字导出公共接口，隐藏实现细节。依赖的外部包通过using或import语句引入，using用于引入整个模块，import用于引入特定函数。

**Section sources**
- [main.jl](file://OBCA/AutonomousParking/main.jl)
- [ParkingConstraints.jl](file://OBCA/AutonomousParking/ParkingConstraints.jl)

### 性能优化实践
Julia代码注重性能优化，利用其JIT编译特性生成高效机器码。避免在循环中分配内存，预分配数组并重用。使用@inbounds和@fastmath宏消除边界检查和浮点数安全假设，提高计算速度。对于热点函数，使用@code_warntype检查类型稳定性，确保编译器能生成最优代码。

**Section sources**
- [main.jl](file://OBCA/AutonomousParking/main.jl)
- [a_star.jl](file://OBCA/AutonomousParking/a_star.jl)

## 跨语言一致性指导

### 命名一致性
尽管不同语言有不同的命名约定，但在跨语言接口处保持命名一致性至关重要。例如，Python模块名与C++库名、Julia模块名应具有相似的语义。公共API的函数名和参数名在不同语言实现中应保持一致，便于开发者理解和使用。

### 错误处理
统一错误处理策略，Python使用异常，C++使用异常或错误码，Julia使用异常。在跨语言调用时，应将底层语言的错误转换为调用语言的错误类型。提供清晰的错误消息和堆栈跟踪，便于调试和问题定位。

### 文档注释
所有语言的代码都应包含详细的文档注释。Python使用docstring，C++使用Doxygen风格注释，Julia使用文档字符串。注释应描述函数目的、参数、返回值和可能的异常。保持注释与代码同步，避免过时的注释误导开发者。

**Section sources**
- [pyproject.toml](file://NeuPAN/pyproject.toml)
- [teb_config.h](file://teb_local_planner/include/teb_local_planner/teb_config.h)
- [robot_footprint_model.h](file://teb_local_planner/include/teb_local_planner/robot_footprint_model.h)
- [main.jl](file://OBCA/AutonomousParking/main.jl)

## 结论
本文档详细阐述了Python、C++和Julia三种语言的编码规范，旨在提高代码质量和团队协作效率。通过遵循这些规范，可以确保代码的一致性、可读性和可维护性。建议所有开发者在编写代码时严格遵守这些规范，并在代码审查中相互监督。随着项目的发展，这些规范也应不断演进和完善，以适应新的需求和技术变化。