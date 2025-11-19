# Multicore Math 模块架构分析

## 概述

`src/chrono/multicore_math` 文件夹提供多核和GPU计算所需的数学类型和操作，主要用于 Chrono::Multicore 模块。

## 文件结构

### 基本类型
```
real.h                      - 实数类型定义
real_single.h               - 单精度浮点
real_double.h               - 双精度浮点
real2.h/cpp                 - 2D向量
real3.h/cpp                 - 3D向量（主要）
real4.h/cpp                 - 4D向量（四元数）
matrix.h/cpp                - 矩阵类型
```

### SIMD支持
```
simd.h                      - SIMD抽象
simd_non.h                  - 无SIMD实现
simd_sse.h                  - SSE实现
simd_avx.h                  - AVX实现
```

### GPU支持
```
thrust.h                    - Thrust容器封装
vec3.cpp/h                  - GPU向量操作
```

### 工具
```
types.h                     - 类型定义
utility.h                   - 实用函数
```

## 设计特性

### SIMD优化
- 支持SSE、AVX指令集
- 自动向量化
- 对齐内存访问

### GPU兼容
- 可在CPU和GPU上编译
- Thrust容器封装
- CUDA内核友好

## 使用场景

### 基本向量操作
```cpp
real3 v1(1.0, 2.0, 3.0);
real3 v2(4.0, 5.0, 6.0);

// 向量运算
real3 sum = v1 + v2;
real3 diff = v1 - v2;
real3 scaled = v1 * 2.0;

// 点积和叉积
real dot = Dot(v1, v2);
real3 cross = Cross(v1, v2);

// 长度和归一化
real length = Length(v1);
real3 normalized = Normalize(v1);
```

### 四元数
```cpp
real4 q1(1, 0, 0, 0);  // 单位四元数
real4 q2 = QuatFromAngleAxis(CH_PI/4, real3(0, 1, 0));

// 四元数乘法
real4 q3 = Mult(q1, q2);

// 旋转向量
real3 v_rotated = Rotate(v1, q2);
```

### 矩阵操作
```cpp
ChMatrix33<real> mat;
mat.SetIdentity();

// 矩阵-向量乘法
real3 result = mat * v1;

// 矩阵-矩阵乘法
ChMatrix33<real> mat2;
ChMatrix33<real> product = mat * mat2;
```

## 与Chrono::Multicore集成

该模块主要为 Chrono::Multicore 提供数学基础：
- 并行碰撞检测
- 并行约束求解
- GPU加速计算

## 性能特点

### 优点
- SIMD加速
- 缓存友好
- GPU兼容
- 低级优化

### 使用场景
- 大规模并行计算
- GPU加速应用
- 高性能计算

## 总结

Multicore Math 模块提供高性能数学基础，主要用于并行和GPU计算场景。
