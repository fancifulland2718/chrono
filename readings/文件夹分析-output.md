# Output 模块架构分析

## 概述

`src/chrono/output` 文件夹提供数据输出功能，支持将仿真数据导出到各种格式用于后处理和可视化。

## 文件结构

```
ChOutput.h                  - 输出基类
ChOutputASCII.h/cpp         - ASCII文本输出
ChOutputHDF5.h/cpp          - HDF5格式输出（大数据）
```

## 使用场景

### ASCII输出
```cpp
ChOutputASCII output("output.dat");

// 在仿真循环中
while (time < end_time) {
    system->DoStepDynamics(dt);
    
    // 输出时间和数据
    output << system->GetChTime() << "\t"
           << body->GetPos() << "\t"
           << body->GetPosDt() << "\n";
}
```

### HDF5输出（需要HDF5库）
```cpp
#ifdef CHRONO_HDF5
ChOutputHDF5 output_hdf5("results.h5");

// 定义数据集
output_hdf5.DefineDataset("time", 1);
output_hdf5.DefineDataset("position", 3);
output_hdf5.DefineDataset("velocity", 3);

// 写入数据
while (time < end_time) {
    system->DoStepDynamics(dt);
    
    output_hdf5.WriteData("time", system->GetChTime());
    output_hdf5.WriteData("position", body->GetPos());
    output_hdf5.WriteData("velocity", body->GetPosDt());
}
#endif
```

## 总结

Output 模块提供了简单但有效的数据导出功能，适用于后处理分析。
