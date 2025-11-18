# Chrono Architecture Analysis

This directory contains architectural documentation for the Project Chrono codebase.

## Documents

### [Chrono架构分析文档.md](./Chrono架构分析文档.md) (Chinese)

A comprehensive architectural analysis document covering:

1. **Overview**: Introduction to Project Chrono and core design principles
2. **Directory Structure & Module Overview**: Top-level organization and 19+ optional modules
3. **Core Engine Architecture**: 
   - Core class hierarchies (ChSystem, ChBody, ChLink, etc.)
   - Solver, timestepper, and collision system details
   - Simulation timestep flow with diagrams
4. **Module Deep Dive**: Detailed analysis of 8 key modules:
   - Chrono::Multicore (parallel rigid/granular dynamics)
   - Chrono::Vehicle (wheeled/tracked vehicle simulation)
   - Chrono::FSI (fluid-solid interaction via SPH)
   - Chrono::DEM (GPU-accelerated discrete element method)
   - Chrono::Sensor (GPU sensor simulation)
   - Chrono::SynChrono (distributed multi-agent simulation)
   - Chrono::ROS (ROS2 interface)
   - Other modules (Irrlicht, VSG, Postprocess, etc.)
5. **Cross-Module Mechanisms**:
   - Solver architecture and comparison
   - Collision detection systems
   - Parallelization strategies (OpenMP, CUDA, MPI)
   - I/O and visualization pipelines
6. **End-to-End Scenarios**: 3 typical use cases with code examples:
   - Pure rigid multibody system
   - Vehicle on granular terrain
   - Robot/vehicle with sensors and ROS integration
7. **Summary & Insights**: Architecture strengths, limitations, evolution trends, and development guidance

## Document Statistics

- **Total Lines**: 1,408
- **Language**: Chinese (中文)
- **Version**: v1.0
- **Based on**: Chrono 9.0.1
- **Date**: 2025-11-18

## Key Features

- **Comprehensive Coverage**: From core physics engine to optional modules
- **Visual Diagrams**: Multiple Mermaid diagrams showing architecture and data flows
- **Code Examples**: Real code snippets for typical scenarios
- **Practical Guidance**: Development recommendations and best practices

## Quick Reference

### Core Components Location

| Component | Source Path |
|-----------|-------------|
| Core System | `src/chrono/physics/ChSystem*.h` |
| Rigid Bodies | `src/chrono/physics/ChBody*.h` |
| Constraints | `src/chrono/physics/ChLink*.h` |
| Solvers | `src/chrono/solver/ChSolver*.h` |
| Timesteppers | `src/chrono/timestepper/ChTimestepper*.h` |
| Collision | `src/chrono/collision/` |
| FEA | `src/chrono/fea/` |
| Vehicle | `src/chrono_vehicle/` |
| Multicore | `src/chrono_multicore/` |
| DEM | `src/chrono_dem/` |
| FSI | `src/chrono_fsi/` |
| Sensor | `src/chrono_sensor/` |
| ROS | `src/chrono_ros/` |
| SynChrono | `src/chrono_synchrono/` |

### Module Dependencies

| Module | Core Deps | External Deps |
|--------|-----------|---------------|
| Core | - | Eigen3 |
| Multicore | Core | OpenMP, Thrust, Blaze |
| Vehicle | Core | OpenCRG (opt), Irrklang (opt) |
| FSI | Core | - |
| DEM | Core | CUDA |
| Sensor | Core | CUDA, OptiX, GLM, GLEW, GLFW |
| SynChrono | Core, Vehicle | MPI |
| ROS | Core | ROS2 packages |

## Target Audience

- **Engineers**: Implementing applications using Chrono
- **Researchers**: Understanding architecture for algorithm development
- **Students**: Learning multiphysics simulation architecture
- **Contributors**: Developing new modules or extending existing ones

## Related Resources

- [Project Chrono Website](http://projectchrono.org/)
- [API Documentation](http://api.projectchrono.org/)
- [Build Instructions](https://api.projectchrono.org/install_guides.html)
- [GitHub Repository](https://github.com/projectchrono/chrono)
