# Multicore Math 模块架构分析

## 概述

`src/chrono/multicore_math` 文件夹提供针对多核和并行执行优化的数学类型和操作。它包括 SIMD 友好的向量/矩阵类型、实数抽象以及用于高性能并行计算的实用程序。

## 主要功能

### 主要职责
1. **SIMD 类型**：用于 SIMD 操作的向量类型（real2、real3、real4）
2. **矩阵操作**：用于并行代码的高效矩阵数学运算
3. **实数抽象**：可切换的 float/double 精度
4. **Thrust 集成**：GPU 兼容的类型和操作
5. **实用函数**：并行内核的数学辅助函数

## 文件结构

```
real.h                  - 基础实数类型 (float or double)
real_single.h          - 单精度特化
real_double.h          - 双精度特化
real2.h                - 2D 向量类型
real3.h                - 3D 向量类型
real4.h                - 4D 向量类型 (or quaternion)
matrix.h               - 矩阵类型和操作
simd.h                 - SIMD 内建函数和操作
thrust.h               - Thrust 库集成
utility.h              - 数学实用函数
other_types.h          - 附加类型定义
```

## 架构图

```mermaid
graph TB
    subgraph "Real 类型"
        REAL[real]
        R1[real (single)]
        R2[real (double)]
    end
    
    subgraph "Vector 类型"
        VEC2[real2]
        VEC3[real3]
        VEC4[real4]
    end
    
    subgraph "Matrix 类型"
        MAT33[mat33]
        MAT[matrix]
    end
    
    subgraph "集成"
        SIMD[SIMD ops]
        THRUST[Thrust types]
    end
    
    subgraph "实用程序"
        UTIL[utility]
    end
    
    R1 -.-> REAL
    R2 -.-> REAL
    
    VEC2 --> REAL
    VEC3 --> REAL
    VEC4 --> REAL
    
    MAT33 --> VEC3
    MAT --> REAL
    
    SIMD --> VEC3
    SIMD --> VEC4
    
    THRUST --> VEC2
    THRUST --> VEC3
    THRUST --> VEC4
    
    style REAL fill:#e1f5ff
    style VEC3 fill:#ffe1f5
    style MAT33 fill:#fff5e1
    style SIMD fill:#e1ffe1
```

## Core Data 类型

### 1. Real Number (real.h)
```cpp
// Switchable precision
#ifdef CHRONO_MULTICORE_USE_DOUBLE
    using real = double;
#else
    using real = float;
#endif

// Constants
static const real C_REAL_MAX = /* ... */;
static const real C_REAL_MIN = /* ... */;
static const real C_REAL_EPSILON = /* ... */;
static const real C_PI = /* ... */;
```

### 2. Vector 类型 (real2.h, real3.h, real4.h)
```cpp
// 2D vector
struct real2 {
    real x, y;
    
    // Constructors
    __host__ __device__ real2() : x(0), y(0) {}
    __host__ __device__ real2(real x, real y) : x(x), y(y) {}
    
    // 操作
    __host__ __device__ real2 operator+(const real2& rhs) const;
    __host__ __device__ real2 operator*(real s) const;
    __host__ __device__ real length() const;
};

// 3D vector
struct real3 {
    real x, y, z;
    
    __host__ __device__ real3() : x(0), y(0), z(0) {}
    __host__ __device__ real3(real x, real y, real z) : x(x), y(y), z(z) {}
    
    // Vector operations
    __host__ __device__ real3 operator+(const real3& rhs) const;
    __host__ __device__ real3 operator-(const real3& rhs) const;
    __host__ __device__ real3 operator*(real s) const;
    __host__ __device__ real3 operator/(real s) const;
    
    // Dot and cross products
    __host__ __device__ real dot(const real3& rhs) const;
    __host__ __device__ real3 cross(const real3& rhs) const;
    
    // Length operations
    __host__ __device__ real length() const;
    __host__ __device__ real length2() const;  // Squared length
    __host__ __device__ real3 normalized() const;
};

// 4D vector / Quaternion
struct real4 {
    real w, x, y, z;  // Quaternion: w + xi + yj + zk
    
    __host__ __device__ real4() : w(0), x(0), y(0), z(0) {}
    __host__ __device__ real4(real w, real x, real y, real z)
        : w(w), x(x), y(y), z(z) {}
    
    // Quaternion operations
    __host__ __device__ real4 operator*(const real4& rhs) const;
    __host__ __device__ real4 conjugate() const;
    __host__ __device__ real4 inverse() const;
    __host__ __device__ real3 rotate(const real3& v) const;
};
```

### 3. Matrix 类型 (matrix.h)
```cpp
// 3x3 matrix
struct mat33 {
    real3 col1, col2, col3;  // Column-major
    
    __host__ __device__ mat33();
    __host__ __device__ mat33(const real3& c1, const real3& c2, const real3& c3);
    
    // Matrix operations
    __host__ __device__ mat33 operator+(const mat33& rhs) const;
    __host__ __device__ mat33 operator*(const mat33& rhs) const;
    __host__ __device__ real3 operator*(const real3& v) const;
    
    // 实用程序
    __host__ __device__ mat33 transpose() const;
    __host__ __device__ real determinant() const;
    __host__ __device__ mat33 inverse() const;
    
    // Special matrices
    __host__ __device__ static mat33 identity();
    __host__ __device__ static mat33 diagonal(real d1, real d2, real d3);
};

// Dynamic matrix
template <typename T>
class ChMulticoreMatrix {
    std::vector<T> data;
    int rows, cols;
    
public:
    ChMulticoreMatrix(int r, int c);
    
    T& operator()(int i, int j);
    const T& operator()(int i, int j) const;
    
    // 操作
    ChMulticoreMatrix operator*(const ChMulticoreMatrix& rhs) const;
    void multiply(const ChMulticoreMatrix& A,
                 const ChMulticoreMatrix& B);
};
```

## SIMD 操作 (simd.h)

```cpp
// SIMD-friendly operations
namespace simd {
    // Dot product (may use SIMD)
    __host__ __device__ inline real dot3(const real3& a, const real3& b);
    
    // Cross product
    __host__ __device__ inline real3 cross3(const real3& a, const real3& b);
    
    // Length
    __host__ __device__ inline real length3(const real3& v);
    
    // Normalize
    __host__ __device__ inline real3 normalize3(const real3& v);
    
    // Matrix-vector multiply
    __host__ __device__ inline real3 mult(const mat33& m, const real3& v);
    
    // Quaternion-vector rotation
    __host__ __device__ inline real3 rotate(const real4& q, const real3& v);
}
```

## Thrust 集成 (thrust.h)

```cpp
// Make types work with Thrust
namespace thrust {
    // Device vectors of custom types
    using device_real_vec = thrust::device_vector<real>;
    using device_real3_vec = thrust::device_vector<real3>;
    using device_real4_vec = thrust::device_vector<real4>;
    
    // Functors for parallel operations
    struct real3_add {
        __host__ __device__
        real3 operator()(const real3& a, const real3& b) const {
            return a + b;
        }
    };
    
    struct real3_scale {
        real factor;
        __host__ __device__
        real3_scale(real f) : factor(f) {}
        
        __host__ __device__
        real3 operator()(const real3& v) const {
            return v * factor;
        }
    };
}
```

## Utility Functions (utility.h)

```cpp
namespace utils {
    // Fast math functions
    __host__ __device__ inline real sqrt(real x);
    __host__ __device__ inline real rsqrt(real x);  // 1/sqrt(x)
    __host__ __device__ inline real abs(real x);
    __host__ __device__ inline real sign(real x);
    
    // Min/max
    __host__ __device__ inline real min(real a, real b);
    __host__ __device__ inline real max(real a, real b);
    __host__ __device__ inline real3 min(const real3& a, const real3& b);
    __host__ __device__ inline real3 max(const real3& a, const real3& b);
    
    // Clamp
    __host__ __device__ inline real clamp(real x, real lo, real hi);
    
    // Lerp (linear interpolation)
    __host__ __device__ inline real lerp(real a, real b, real t);
    __host__ __device__ inline real3 lerp(const real3& a, const real3& b, real t);
    
    // Quaternion utilities
    __host__ __device__ real4 quat_from_axis_angle(const real3& axis, real angle);
    __host__ __device__ real4 quat_slerp(const real4& q1, const real4& q2, real t);
    __host__ __device__ mat33 quat_to_mat33(const real4& q);
}
```

## 关键设计决策

### 1. GPU Compatibility
**决策**: __host__ __device__ annotations on all functions
**理由**:
- Code works on both CPU and GPU
- Single implementation for both
- CUDA compatibility
- Reduced code duplication

### 2. Struct-Based Vectors
**决策**: Plain structs instead of classes
**理由**:
- POD types for GPU kernels
- Fast pass-by-value
- No virtual functions
- Optimal memory layout

### 3. Switchable Precision
**决策**: 'real' typedef for float or double
**理由**:
- Choose precision at compile time
- Single codebase for both
- GPU often faster with float
- CPU may benefit from double

### 4. Column-Major Matrices
**决策**: Matrices stored column-major
**理由**:
- Compatible with 打开GL/graphics
- SIMD-friendly for column operations
- Standard in scientific computing
- Efficient for common operations

### 5. Thrust 集成
**决策**: 类型 compatible with Thrust library
**理由**:
- Leverage Thrust parallel algorithms
- GPU acceleration
- Standard C++ STL-like interface
- Proven performance

## 性能特性

### 优势
1. **SIMD Friendly**: Data layout optimized for vectorization
2. **GPU Compatible**: All code runs on GPU
3. **Cache Efficient**: Compact data structures
4. **Inlined 操作**: Small functions inlined
5. **Precision Control**: Choose speed vs accuracy

### 注意事项
1. **Alignment**: May need explicit alignment for SIMD
2. **Register Pressure**: Large structs may spill registers
3. **Precision Loss**: Float has limited precision
4. **Atomic 操作**: Limited atomic support for custom types

## 典型使用模式

### Vector 操作
```cpp
// 创建 vectors
real3 a(1.0, 2.0, 3.0);
real3 b(4.0, 5.0, 6.0);

// 操作
real3 c = a + b;
real3 d = a * 2.0;
real dot = a.dot(b);
real3 cross = a.cross(b);

// Normalize
real3 n = a.normalized();

// Length
real len = a.length();
real len2 = a.length2();  // Faster, no sqrt
```

### Quaternion Rotations
```cpp
// 创建 quaternion from axis-angle
real3 axis(0, 0, 1);  // Z axis
real angle = C_PI / 4;  // 45 degrees
real4 q = utils::quat_from_axis_angle(axis, angle);

// Rotate vector
real3 v(1, 0, 0);
real3 v_rotated = q.rotate(v);

// Quaternion multiplication
real4 q2 = /* ... */;
real4 q_combined = q * q2;
```

### Matrix 操作
```cpp
// 创建 matrix
mat33 R = mat33::identity();
mat33 S = mat33::diagonal(2.0, 2.0, 2.0);  // Scale

// Transform vector
real3 v(1, 2, 3);
real3 v_transformed = R * v;

// Matrix multiply
mat33 M = R * S;

// Transpose and inverse
mat33 Rt = R.transpose();
mat33 Rinv = R.inverse();
```

### Parallel 操作 with Thrust
```cpp
// Device vectors
thrust::device_vector<real3> d_positions(n);
thrust::device_vector<real3> d_velocities(n);
thrust::device_vector<real3> d_forces(n);

// Parallel transform: positions += velocities * dt
thrust::transform(d_positions.begin(), d_positions.end(),
                 d_velocities.begin(),
                 d_positions.begin(),
                 [dt] __device__ (const real3& p, const real3& v) {
                     return p + v * dt;
                 });

// Parallel reduction: sum all forces
real3 total_force = thrust::reduce(d_forces.begin(), d_forces.end(),
                                   real3(0, 0, 0),
                                   thrust::plus<real3>());
```

### SIMD 操作
```cpp
// Use SIMD namespace for potentially vectorized ops
real3 a(1, 2, 3);
real3 b(4, 5, 6);

real dot = simd::dot3(a, b);
real3 cross = simd::cross3(a, b);
real3 normalized = simd::normalize3(a);

// Matrix-vector multiply
mat33 M = /* ... */;
real3 v = /* ... */;
real3 result = simd::mult(M, v);
```

## 集成 with Chrono Multicore

The multicore_math 模块是 specifically designed for Chrono::Multicore, providing:
- GPU-compatible types for CUDA kernels
- Efficient data structures for parallel solvers
- SIMD-friendly layouts for 打开MP code
- 集成 with Thrust for GPU algorithms

## Best Practices

### 1. Use Appropriate Precision
```cpp
// Use float for GPU (faster)
#define CHRONO_MULTICORE_USE_FLOAT

// Use double for CPU (more accurate)
#define CHRONO_MULTICORE_USE_DOUBLE
```

### 2. Minimize Host-Device Transfers
```cpp
// Bad: transfer every iteration
for (int i = 0; i < n; i++) {
    real3 h_pos = d_positions[i];  // Copy to host
    // ...
}

// Good: bulk transfer
thrust::host_vector<real3> h_positions = d_positions;
```

### 3. Use Length2 When Possible
```cpp
// Comparing distances: avoid sqrt
if (v.length2() < threshold * threshold) {  // Good
    // ...
}

if (v.length() < threshold) {  // Slower (sqrt)
    // ...
}
```

### 4. Prefer SIMD Functions
```cpp
// Use simd:: namespace for better performance
real dot = simd::dot3(a, b);  // May use SIMD

// Instead of manual computation
real dot = a.x*b.x + a.y*b.y + a.z*b.z;
```

## 总结

The multicore_math 模块提供:
- GPU-compatible mathematical types (real2, real3, real4)
- SIMD-optimized vector and matrix operations
- 可切换 float/double 精度
- Thrust integration for parallel algorithms
- Efficient data structures for parallel computing

Its design emphasizes performance, GPU compatibility, and ease of use in parallel computing contexts, making it essential for Chrono::Multicore's high-performance simulations.
