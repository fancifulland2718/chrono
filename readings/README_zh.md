# Chrono 架构分析文档

本目录包含 Project Chrono 代码库的架构文档。

## 文档列表

### [Chrono架构分析文档.md](./Chrono架构分析文档.md) (中文)

一份全面的架构分析文档，涵盖：

1. **概览**：Project Chrono 介绍和核心设计原则
2. **目录结构与模块总览**：顶层组织结构和 19+ 个可选模块
3. **核心引擎架构**： 
   - 核心类层次结构（ChSystem、ChBody、ChLink 等）
   - 求解器、时间步进器和碰撞系统详情
   - 带图表的仿真时间步流程
4. **模块深入分析**：8 个关键模块的详细分析：
   - Chrono::Multicore（并行刚体/颗粒动力学）
   - Chrono::Vehicle（轮式/履带式车辆仿真）
   - Chrono::FSI（基于 SPH 的流固耦合）
   - Chrono::DEM（GPU 加速的离散元方法）
   - Chrono::Sensor（GPU 传感器仿真）
   - Chrono::SynChrono（分布式多智能体仿真）
   - Chrono::ROS（ROS2 接口）
   - 其他模块（Irrlicht、VSG、Postprocess 等）
5. **跨模块共性机制**：
   - 求解器架构和对比
   - 碰撞检测系统
   - 并行化策略（OpenMP、CUDA、MPI）
   - I/O 和可视化管线
6. **端到端场景**：3 个典型用例及代码示例：
   - 纯刚体多体系统
   - 颗粒地形上的车辆
   - 带传感器和 ROS 集成的机器人/车辆
7. **总结与见解**：架构优势、局限性、演进趋势和开发指导

## 文档统计

- **总行数**：1,408
- **语言**：中文
- **版本**：v1.0
- **基于版本**：Chrono 9.0.1
- **日期**：2025-11-18

## 主要特点

- **全面覆盖**：从核心物理引擎到可选模块
- **可视化图表**：多个 Mermaid 图表展示架构和数据流
- **代码示例**：典型场景的真实代码片段
- **实用指导**：开发建议和最佳实践

## 快速参考

### 核心组件位置

| 组件 | 源代码路径 |
|------|-----------|
| 核心系统 | `src/chrono/physics/ChSystem*.h` |
| 刚体 | `src/chrono/physics/ChBody*.h` |
| 约束 | `src/chrono/physics/ChLink*.h` |
| 求解器 | `src/chrono/solver/ChSolver*.h` |
| 时间步进器 | `src/chrono/timestepper/ChTimestepper*.h` |
| 碰撞 | `src/chrono/collision/` |
| 有限元 | `src/chrono/fea/` |
| 车辆 | `src/chrono_vehicle/` |
| 多核 | `src/chrono_multicore/` |
| 离散元 | `src/chrono_dem/` |
| 流固耦合 | `src/chrono_fsi/` |
| 传感器 | `src/chrono_sensor/` |
| ROS | `src/chrono_ros/` |
| 分布式 | `src/chrono_synchrono/` |

### 模块依赖关系

| 模块 | 核心依赖 | 外部依赖 |
|------|---------|---------|
| Core | - | Eigen3 |
| Multicore | Core | OpenMP, Thrust, Blaze |
| Vehicle | Core | OpenCRG (可选), Irrklang (可选) |
| FSI | Core | - |
| DEM | Core | CUDA |
| Sensor | Core | CUDA, OptiX, GLM, GLEW, GLFW |
| SynChrono | Core, Vehicle | MPI |
| ROS | Core | ROS2 packages |

## 目标受众

- **工程师**：使用 Chrono 实现应用程序
- **研究人员**：理解架构以进行算法开发
- **学生**：学习多物理仿真架构
- **贡献者**：开发新模块或扩展现有模块

## 相关资源

- [Project Chrono 网站](http://projectchrono.org/)
- [API 文档](http://api.projectchrono.org/)
- [构建说明](https://api.projectchrono.org/install_guides.html)
- [GitHub 仓库](https://github.com/projectchrono/chrono)
