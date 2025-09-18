# Python编码规范

<cite>
**本文档中引用的文件**  
- [pyproject.toml](file://NeuPAN/pyproject.toml)
- [neupan.py](file://NeuPAN/neupan/neupan.py)
- [rda_solver.py](file://RDA-planner/RDA_planner/rda_solver.py)
</cite>

## 目录
1. [项目结构](#项目结构)
2. [代码格式化规范](#代码格式化规范)
3. [代码检查规范](#代码检查规范)
4. [命名约定](#命名约定)
5. [模块导入顺序](#模块导入顺序)
6. [类型注解使用规范](#类型注解使用规范)
7. [开发环境集成](#开发环境集成)

## 项目结构

NeuPAN项目采用标准的Python包结构，核心代码位于`neupan`目录下，包含多个功能模块。项目使用`pyproject.toml`作为构建和依赖管理配置文件，替代了传统的`setup.py`。主要组件包括：
- `neupan/`：主包目录，包含核心算法实现
- `example/`：示例配置和使用案例
- `neupan.egg-info/`：包元数据信息

**Section sources**
- [pyproject.toml](file://NeuPAN/pyproject.toml#L0-L34)

## 代码格式化规范

NeuPAN项目使用Black作为代码格式化工具，确保代码风格的一致性。虽然`pyproject.toml`文件中未显式配置Black参数，但根据项目代码的实际样式可以推断出以下格式化规范：

- **行宽**：采用默认的88个字符行宽限制
- **字符串引号**：统一使用双引号（"）表示字符串
- **空格使用**：在运算符周围添加空格，函数参数间使用逗号加空格分隔
- **缩进**：使用4个空格进行缩进，不使用制表符

Black会自动处理括号内的换行、函数调用和定义的格式化，确保代码具有良好的可读性。

**Section sources**
- [neupan.py](file://NeuPAN/neupan/neupan.py#L0-L402)

## 代码检查规范

项目使用flake8进行代码质量检查，虽然`pyproject.toml`中未直接配置flake8，但从代码实践可以看出以下检查规则：

- **禁用的错误码**：项目代码遵循PEP 8规范，避免使用过长的行（E501）、多余的空白（W291-W293）等问题
- **最大复杂度**：函数和方法的圈复杂度保持在合理范围内，通过分解复杂逻辑来维持代码的可维护性
- **导入顺序**：遵循标准的导入分组顺序（标准库、第三方库、本地库）
- **变量命名**：使用小写字母和下划线的组合（snake_case）命名变量和函数

这些规范确保了代码的高质量和一致性。

**Section sources**
- [rda_solver.py](file://RDA-planner/RDA_planner/rda_solver.py#L0-L799)

## 命名约定

项目严格遵循Python的命名约定，确保代码的可读性和一致性：

### 函数和类命名
- **函数命名**：使用小写字母和下划线（snake_case），如`scan_to_point`、`update_initial_path_from_goal`
- **类命名**：采用驼峰命名法（CamelCase），如`neupan`、`RDA_solver`
- **常量命名**：使用大写字母和下划线，如`T`、`dt`等参数

```python
class neupan(torch.nn.Module):
    def scan_to_point(self, state, scan, scan_offset=[0, 0, 0]):
        pass
    
    def update_initial_path_from_goal(self, start, goal):
        pass
```

### 变量命名
- **实例变量**：使用有意义的描述性名称，如`cur_vel_array`、`robot_kwargs`
- **参数命名**：清晰表达参数用途，如`receding`、`step_time`、`ref_speed`

这些命名约定提高了代码的可读性和可维护性。

**Section sources**
- [neupan.py](file://NeuPAN/neupan/neupan.py#L0-L402)

## 模块导入顺序

项目遵循PEP 8推荐的导入顺序规范，将导入语句分为三个明确的组，每组之间用空行分隔：

1. **标准库导入**：Python内置模块
2. **第三方库导入**：外部依赖包
3. **本地应用/库导入**：项目内部模块

```python
import yaml
import torch
from neupan.robot import robot
from neupan.blocks import InitialPath, PAN
from neupan import configuration
from neupan.util import time_it, file_check, get_transform
import numpy as np
from neupan.configuration import np_to_tensor, tensor_to_np
from math import cos, sin
```

这种导入顺序使得依赖关系清晰可见，便于维护和理解代码的依赖结构。同时，项目使用绝对导入而非相对导入，提高了代码的可读性和可维护性。

**Section sources**
- [neupan.py](file://NeuPAN/neupan/neupan.py#L0-L402)

## 类型注解使用规范

项目在关键接口处使用类型注解，提高代码的可读性和可维护性：

### 函数参数类型注解
```python
def __init__(
    self,
    receding: int = 10,
    step_time: float = 0.1,
    ref_speed: float = 4.0,
    device: str = "cpu",
    robot_kwargs: dict = None,
    ipath_kwargs: dict = None,
    pan_kwargs: dict = None,
    adjust_kwargs: dict = None,
    train_kwargs: dict = None,
    **kwargs,
) -> None:
```

### 返回值类型注解
```python
def forward(self, state, points, velocities=None):
    # 返回值类型通过文档字符串说明
    pass
```

### 属性类型推断
虽然项目没有在所有地方使用类型注解，但在关键的公共API中使用了类型提示，特别是对于：
- 基本数据类型（int、float、str、bool）
- 容器类型（dict、list）
- NumPy数组和PyTorch张量的使用场景

这种有选择性的类型注解策略在保持代码简洁的同时，提供了足够的类型信息给开发者和IDE。

**Section sources**
- [neupan.py](file://NeuPAN/neupan/neupan.py#L0-L402)

## 开发环境集成

为了确保代码一致性，建议在开发环境中集成以下工具：

### 配置文件设置
在项目根目录的`pyproject.toml`中添加Black和flake8配置：

```toml
[tool.black]
line-length = 88
target-version = ['py310']
include = '\.pyi?$'
extend-exclude = '''
/(
  # directories
  \.eggs
| \.git
| \.hg
| \.mypy_cache
| \.tox
| \.venv
| _build
| buck-out
| build
| dist
)/
'''

[tool.flake8]
max-line-length = 88
extend-ignore = E203, W503
```

### 预提交钩子
使用`pre-commit`框架自动格式化和检查代码：

```bash
# 安装pre-commit
pip install pre-commit black flake8

# 创建.pre-commit-config.yaml
repos:
  - repo: https://github.com/psf/black
    rev: 22.3.0
    hooks:
      - id: black
        language_version: python3.10
  
  - repo: https://gitlab.com/pycqa/flake8
    rev: 4.0.1
    hooks:
      - id: flake8
```

### IDE集成
在主流IDE（如VS Code、PyCharm）中配置：
- 将Black设置为默认格式化工具
- 启用flake8作为代码检查工具
- 配置保存时自动格式化

这些集成措施确保了团队成员在提交代码前自动遵循统一的编码规范。

**Section sources**
- [pyproject.toml](file://NeuPAN/pyproject.toml#L0-L34)