# Output 模块架构分析

## 概述

`src/chrono/output` 文件夹提供数据输出功能，用于将仿真结果导出到各种文件格式。它支持 ASCII 文本文件和 HDF5，以实现仿真数据的高效存储和分析。

## 主要功能

### 主要职责
1. **数据导出**：将仿真状态导出到文件
2. **多种格式**：支持 ASCII 和 HDF5
3. **时间序列**：记录随时间变化的数据
4. **自定义数据**：灵活的数据结构导出
5. **高效存储**：压缩和二进制格式（HDF5）

## 文件结构

```
ChOutput.h/cpp          - 抽象基础输出类
ChOutputASCII.h/cpp     - ASCII 文本文件输出
ChOutputHDF5.h/cpp      - HDF5 二进制格式输出
```

## 架构图

```mermaid
graph TB
    subgraph "输出基类"
        OUT[ChOutput]
    end
    
    subgraph "格式实现"
        ASCII[ChOutputASCII]
        HDF5[ChOutputHDF5]
    end
    
    ASCII --|> OUT
    HDF5 --|> OUT
    
    style OUT fill:#e1f5ff
    style ASCII fill:#ffe1f5
    style HDF5 fill:#fff5e1
```

## 核心外部接口

### 1. 输出基类 (ChOutput.h)
```cpp
class ChApi ChOutput {
public:
    // 打开/关闭
    virtual bool Open(const std::string& filename) = 0;
    virtual void Close() = 0;
    
    // 写入数据
    virtual void WriteData(double time,
                          const std::string& name,
                          double value) = 0;
    
    virtual void WriteData(double time,
                          const std::string& name,
                          const ChVector3& value) = 0;
    
    virtual void WriteData(double time,
                          const std::string& name,
                          const ChQuaternion<>& value) = 0;
    
    // 刷新
    virtual void Flush() = 0;
};
```

### 2. ASCII 输出 (ChOutputASCII.h)
```cpp
class ChApi ChOutputASCII : public ChOutput {
public:
    ChOutputASCII(const std::string& delimiter = "\t");
    
    // 打开文件
    bool Open(const std::string& filename) override;
    void Close() override;
    
    // 写入表头
    void WriteHeader(const std::vector<std::string>& columns);
    
    // 写入行
    void WriteRow(const std::vector<double>& values);
    
    // 写入数据
    void WriteData(double time,
                  const std::string& name,
                  double value) override;
    
    // 配置
    void SetDelimiter(const std::string& delim);
    void SetPrecision(int precision);
};
```

### 3. HDF5 输出 (ChOutputHDF5.h)
```cpp
class ChApi ChOutputHDF5 : public ChOutput {
public:
    ChOutputHDF5();
    
    // 文件操作
    bool Open(const std::string& filename) override;
    void Close() override;
    
    // 数据集创建
    void CreateDataset(const std::string& name,
                      int dims,
                      const std::vector<int>& sizes);
    
    // 写入数据
    void WriteData(double time,
                  const std::string& name,
                  double value) override;
    
    void WriteArray(const std::string& name,
                   const std::vector<double>& data);
    
    // 属性
    void WriteAttribute(const std::string& dataset,
                       const std::string& attr_name,
                       const std::string& attr_value);
    
    // 压缩
    void SetCompression(bool enable);
};
```

## 依赖关系

### 外部依赖
- **HDF5 库**：用于 HDF5 格式支持（可选）

### 内部依赖
- **core**：数据类型的 ChVector3、ChQuaternion
- **serialization**：可能在内部使用归档

### 其他模块的使用
- **所有仿真模块**：导出结果
- **postprocess**：读取导出的数据进行分析

## 典型使用模式

### ASCII 输出
```cpp
// 创建 ASCII 输出
ChOutputASCII output("\t");  // 制表符分隔
output.Open("results.txt");

// 写入表头
output.WriteHeader({"Time", "Pos_X", "Pos_Y", "Pos_Z"});

// 仿真循环
while (simulating) {
    system.DoStepDynamics(dt);
    
    ChVector3 pos = body->GetPos();
    output.WriteRow({system.GetChTime(),
                    pos.x(), pos.y(), pos.z()});
}

output.Close();
```

### HDF5 输出
```cpp
// 创建 HDF5 输出
ChOutputHDF5 output;
output.Open("results.h5");
output.SetCompression(true);

// 创建数据集
output.CreateDataset("time", 1, {10000});
output.CreateDataset("position", 2, {10000, 3});

// 写入数据
for (int i = 0; i < num_steps; i++) {
    system.DoStepDynamics(dt);
    
    output.WriteData(system.GetChTime(), "time", system.GetChTime());
    output.WriteData(system.GetChTime(), "position", body->GetPos());
}

output.Close();
```

### 自定义数据导出
```cpp
// 导出多个量
ChOutputASCII output;
output.Open("detailed_results.csv");
output.SetDelimiter(",");
output.SetPrecision(10);

while (simulating) {
    system.DoStepDynamics(dt);
    
    // 写入各种量
    output.WriteData(system.GetChTime(), "time", system.GetChTime());
    output.WriteData(system.GetChTime(), "pos", body->GetPos());
    output.WriteData(system.GetChTime(), "vel", body->GetPosDt());
    output.WriteData(system.GetChTime(), "rot", body->GetRot());
    output.WriteData(system.GetChTime(), "omega", body->GetAngVelParent());
    output.WriteData(system.GetChTime(), "force", body->GetAppliedForce());
}
```

## 格式比较

### ASCII
- **优点**：人类可读、易于解析、通用
- **缺点**：文件大、I/O 较慢、精度有限
- **用途**：小型数据集、调试、快速分析

### HDF5
- **优点**：紧凑、快速、分层、压缩
- **缺点**：需要库、二进制格式
- **用途**：大型数据集、高效存储、复杂数据

## 关键设计决策

### 1. 抽象输出接口
**决策**：ChOutput 基类和格式实现
**理由**：
- 无论格式如何都有统一的 API
- 易于添加新格式
- 运行时格式选择
- 清晰的关注点分离

### 2. 多种数据类型
**决策**：为不同类型重载 WriteData
**理由**：
- 类型安全的接口
- 自动格式化
- 方便常见类型
- 可扩展用于自定义类型

### 3. HDF5 可选
**决策**：HDF5 支持是可选的
**理由**：
- 并非所有用户都需要 HDF5
- 减少依赖
- 基本使用的构建更简单
- 高级用户可以启用

## 总结

output 模块提供：
- 简单的文件格式数据导出
- 人类可读数据的 ASCII 输出
- 高效二进制存储的 HDF5 输出
- 灵活的时间序列数据记录
- 常见数据类型的类型安全接口

其设计强调简单性和效率，使记录仿真结果以进行后处理和分析变得容易。
