# Output Module Architecture Analysis

## Overview

The `src/chrono/output` folder provides data output functionality for exporting simulation results to various file formats. It supports ASCII text files and HDF5 for efficient storage and analysis of simulation data.

## Main Functionality

### Primary Responsibilities
1. **Data Export**: Export simulation state to files
2. **Multiple Formats**: ASCII and HDF5 support
3. **Time Series**: Record data over time
4. **Custom Data**: Flexible data structure export
5. **Efficient Storage**: Compressed and binary formats (HDF5)

## File Structure

```
ChOutput.h/cpp          - Abstract base output class
ChOutputASCII.h/cpp     - ASCII text file output
ChOutputHDF5.h/cpp      - HDF5 binary format output
```

## Architecture Diagram

```mermaid
graph TB
    subgraph "Output Base"
        OUT[ChOutput]
    end
    
    subgraph "Format Implementations"
        ASCII[ChOutputASCII]
        HDF5[ChOutputHDF5]
    end
    
    ASCII --|> OUT
    HDF5 --|> OUT
    
    style OUT fill:#e1f5ff
    style ASCII fill:#ffe1f5
    style HDF5 fill:#fff5e1
```

## Core External Interfaces

### 1. Output Base (ChOutput.h)
```cpp
class ChApi ChOutput {
public:
    // Open/close
    virtual bool Open(const std::string& filename) = 0;
    virtual void Close() = 0;
    
    // Write data
    virtual void WriteData(double time,
                          const std::string& name,
                          double value) = 0;
    
    virtual void WriteData(double time,
                          const std::string& name,
                          const ChVector3& value) = 0;
    
    virtual void WriteData(double time,
                          const std::string& name,
                          const ChQuaternion<>& value) = 0;
    
    // Flush
    virtual void Flush() = 0;
};
```

### 2. ASCII Output (ChOutputASCII.h)
```cpp
class ChApi ChOutputASCII : public ChOutput {
public:
    ChOutputASCII(const std::string& delimiter = "\t");
    
    // Open file
    bool Open(const std::string& filename) override;
    void Close() override;
    
    // Write header
    void WriteHeader(const std::vector<std::string>& columns);
    
    // Write row
    void WriteRow(const std::vector<double>& values);
    
    // Write data
    void WriteData(double time,
                  const std::string& name,
                  double value) override;
    
    // Configuration
    void SetDelimiter(const std::string& delim);
    void SetPrecision(int precision);
};
```

### 3. HDF5 Output (ChOutputHDF5.h)
```cpp
class ChApi ChOutputHDF5 : public ChOutput {
public:
    ChOutputHDF5();
    
    // File operations
    bool Open(const std::string& filename) override;
    void Close() override;
    
    // Dataset creation
    void CreateDataset(const std::string& name,
                      int dims,
                      const std::vector<int>& sizes);
    
    // Write data
    void WriteData(double time,
                  const std::string& name,
                  double value) override;
    
    void WriteArray(const std::string& name,
                   const std::vector<double>& data);
    
    // Attributes
    void WriteAttribute(const std::string& dataset,
                       const std::string& attr_name,
                       const std::string& attr_value);
    
    // Compression
    void SetCompression(bool enable);
};
```

## Dependencies

### External Dependencies
- **HDF5 library**: For HDF5 format support (optional)

### Internal Dependencies
- **core**: ChVector3, ChQuaternion for data types
- **serialization**: May use archives internally

### Usage by Other Modules
- **All simulation modules**: Export results
- **postprocess**: Read exported data for analysis

## Typical Usage Patterns

### ASCII Output
```cpp
// Create ASCII output
ChOutputASCII output("\t");  // Tab-delimited
output.Open("results.txt");

// Write header
output.WriteHeader({"Time", "Pos_X", "Pos_Y", "Pos_Z"});

// Simulation loop
while (simulating) {
    system.DoStepDynamics(dt);
    
    ChVector3 pos = body->GetPos();
    output.WriteRow({system.GetChTime(),
                    pos.x(), pos.y(), pos.z()});
}

output.Close();
```

### HDF5 Output
```cpp
// Create HDF5 output
ChOutputHDF5 output;
output.Open("results.h5");
output.SetCompression(true);

// Create datasets
output.CreateDataset("time", 1, {10000});
output.CreateDataset("position", 2, {10000, 3});

// Write data
for (int i = 0; i < num_steps; i++) {
    system.DoStepDynamics(dt);
    
    output.WriteData(system.GetChTime(), "time", system.GetChTime());
    output.WriteData(system.GetChTime(), "position", body->GetPos());
}

output.Close();
```

### Custom Data Export
```cpp
// Export multiple quantities
ChOutputASCII output;
output.Open("detailed_results.csv");
output.SetDelimiter(",");
output.SetPrecision(10);

while (simulating) {
    system.DoStepDynamics(dt);
    
    // Write various quantities
    output.WriteData(system.GetChTime(), "time", system.GetChTime());
    output.WriteData(system.GetChTime(), "pos", body->GetPos());
    output.WriteData(system.GetChTime(), "vel", body->GetPosDt());
    output.WriteData(system.GetChTime(), "rot", body->GetRot());
    output.WriteData(system.GetChTime(), "omega", body->GetAngVelParent());
    output.WriteData(system.GetChTime(), "force", body->GetAppliedForce());
}
```

## Format Comparison

### ASCII
- **Pros**: Human-readable, easy to parse, universal
- **Cons**: Large files, slower I/O, limited precision
- **Use**: Small datasets, debugging, quick analysis

### HDF5
- **Pros**: Compact, fast, hierarchical, compressed
- **Cons**: Requires library, binary format
- **Use**: Large datasets, efficient storage, complex data

## Key Design Decisions

### 1. Abstract Output Interface
**Decision**: ChOutput base class with format implementations
**Rationale**:
- Uniform API regardless of format
- Easy to add new formats
- Format selection at runtime
- Clean separation of concerns

### 2. Multiple Data Types
**Decision**: Overloaded WriteData for different types
**Rationale**:
- Type-safe interface
- Automatic formatting
- Convenient for common types
- Extensible for custom types

### 3. HDF5 Optional
**Decision**: HDF5 support is optional
**Rationale**:
- Not all users need HDF5
- Reduces dependencies
- Simpler builds for basic use
- Advanced users can enable

## Summary

The output module provides:
- Simple data export to file formats
- ASCII output for human-readable data
- HDF5 output for efficient binary storage
- Flexible time-series data recording
- Type-safe interfaces for common data types

Its design emphasizes simplicity and efficiency, making it easy to record simulation results for post-processing and analysis.
