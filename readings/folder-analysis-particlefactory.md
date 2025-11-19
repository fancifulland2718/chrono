# Particle Factory 模块架构分析

## 概述

`src/chrono/particlefactory` 文件夹提供了一个粒子生成和管理系统，用于在仿真中创建、发射和处理粒子。它支持各种发射模式、粒子形状和基于事件的处理。

## 主要功能

### 主要职责
1. **粒子发射**：以指定的速率和模式创建粒子
2. **形状生成**：随机粒子形状（球体、盒子等）
3. **位置/速度**：随机放置和初始速度
4. **事件触发器**：基于条件的粒子处理
5. **粒子移除**：自动清理粒子
6. **批量处理**：高效的多粒子操作

## 文件结构

```
ChParticleEmitter.h/cpp         - 主粒子发射器
ChRandomShapeCreator.h/cpp      - 随机形状生成
ChRandomParticlePosition.h/cpp  - 随机位置生成
ChRandomParticleVelocity.h/cpp  - 随机速度生成
ChParticleEventTrigger.h/cpp    - 基于事件的触发器
ChParticleProcessor.h/cpp       - 粒子处理基类
ChParticleProcessEvent.h/cpp    - 基于事件的处理
ChParticleRemover.h/cpp         - 粒子移除
```

## 架构图

```mermaid
graph TB
    subgraph "发射器核心"
        EMIT[ChParticleEmitter]
    end
    
    subgraph "随机生成器"
        RSHAPE[ChRandomShapeCreator]
        RPOS[ChRandomParticlePosition]
        RVEL[ChRandomParticleVelocity]
    end
    
    subgraph "处理"
        PROC[ChParticleProcessor]
        EVENT[ChParticleProcessEvent]
        TRIG[ChParticleEventTrigger]
        REM[ChParticleRemover]
    end
    
    EMIT --> RSHAPE
    EMIT --> RPOS
    EMIT --> RVEL
    
    PROC --> EVENT
    EVENT --> TRIG
    REM --|> PROC
    
    style EMIT fill:#e1f5ff
    style RSHAPE fill:#ffe1f5
    style PROC fill:#fff5e1
```

## 核心外部接口

### 1. 粒子发射器 (ChParticleEmitter.h)
```cpp
class ChApi ChParticleEmitter {
public:
    // 构造
    ChParticleEmitter();
    
    // 配置
    void SetParticlesPerSecond(double rate);
    void SetShapeCreator(std::shared_ptr<ChRandomShapeCreator> creator);
    void SetPositionGenerator(std::shared_ptr<ChRandomParticlePosition> gen);
    void SetVelocityGenerator(std::shared_ptr<ChRandomParticleVelocity> gen);
    
    // 材料
    void SetMaterial(std::shared_ptr<ChContactMaterial> material);
    
    // 发射
    void EmitParticles(ChSystem& system, double dt);
    
    // 统计信息
    int GetNumEmittedTotal() const;
    int GetNumEmittedLastStep() const;
};
```

### 2. 随机形状创建器 (ChRandomShapeCreator.h)
```cpp
class ChApi ChRandomShapeCreator {
public:
    // 添加形状分布
    void AddShape(ChCollisionShape::Type type,
                 double probability,
                 const ChVector3& size_min,
                 const ChVector3& size_max);
    
    void AddSphere(double probability,
                  double radius_min,
                  double radius_max);
    
    void AddBox(double probability,
               const ChVector3& size_min,
               const ChVector3& size_max);
    
    void AddCylinder(double probability,
                    double radius_min, double radius_max,
                    double height_min, double height_max);
    
    // 生成随机形状
    std::shared_ptr<ChBody> CreateRandomBody(ChSystem& system);
    
    // 密度
    void SetDensity(double density);
};
```

### 3. 随机位置 (ChRandomParticlePosition.h)
```cpp
class ChApi ChRandomParticlePosition {
public:
    virtual ChVector3 GetRandomPosition() = 0;
};

class ChApi ChRandomParticlePositionBox : public ChRandomParticlePosition {
public:
    ChRandomParticlePositionBox(const ChVector3& box_min,
                                const ChVector3& box_max);
    
    ChVector3 GetRandomPosition() override;
};

class ChApi ChRandomParticlePositionSphere : public ChRandomParticlePosition {
public:
    ChRandomParticlePositionSphere(const ChVector3& center,
                                   double radius);
    
    ChVector3 GetRandomPosition() override;
};

class ChApi ChRandomParticlePositionCylinder : public ChRandomParticlePosition {
public:
    ChRandomParticlePositionCylinder(const ChVector3& center,
                                     const ChVector3& axis,
                                     double radius,
                                     double height);
    
    ChVector3 GetRandomPosition() override;
};
```

### 4. 随机速度 (ChRandomParticleVelocity.h)
```cpp
class ChApi ChRandomParticleVelocity {
public:
    virtual ChVector3 GetRandomVelocity() = 0;
};

class ChApi ChRandomParticleVelocityConstant : public ChRandomParticleVelocity {
public:
    ChRandomParticleVelocityConstant(const ChVector3& velocity);
    
    ChVector3 GetRandomVelocity() override;
};

class ChApi ChRandomParticleVelocityBox : public ChRandomParticleVelocity {
public:
    ChRandomParticleVelocityBox(const ChVector3& vel_min,
                                const ChVector3& vel_max);
    
    ChVector3 GetRandomVelocity() override;
};

class ChApi ChRandomParticleVelocityNormal : public ChRandomParticleVelocity {
public:
    ChRandomParticleVelocityNormal(const ChVector3& direction,
                                   double speed_mean,
                                   double speed_stddev);
    
    ChVector3 GetRandomVelocity() override;
};
```

### 5. 事件触发器 (ChParticleEventTrigger.h)
```cpp
class ChApi ChParticleEventTrigger {
public:
    virtual bool CheckTrigger(std::shared_ptr<ChBody> particle,
                             ChSystem& system) = 0;
};

class ChApi ChParticleEventTriggerBox : public ChParticleEventTrigger {
public:
    // 粒子进入/离开盒子时触发
    enum Mode { INSIDE, OUTSIDE };
    
    ChParticleEventTriggerBox(const ChVector3& box_min,
                             const ChVector3& box_max,
                             Mode mode);
    
    bool CheckTrigger(std::shared_ptr<ChBody> particle,
                     ChSystem& system) override;
};

class ChApi ChParticleEventTriggerTime : public ChParticleEventTrigger {
public:
    // 指定时间后触发
    ChParticleEventTriggerTime(double time_threshold);
    
    bool CheckTrigger(std::shared_ptr<ChBody> particle,
                     ChSystem& system) override;
};
```

### 6. 粒子处理器 (ChParticleProcessor.h)
```cpp
class ChApi ChParticleProcessor {
public:
    // 处理所有粒子
    virtual void Process(ChSystem& system) = 0;
    
    // 过滤粒子
    void SetParticleFilter(std::function<bool(std::shared_ptr<ChBody>)> filter);
};

class ChApi ChParticleRemover : public ChParticleProcessor {
public:
    // 添加移除触发器
    void AddTrigger(std::shared_ptr<ChParticleEventTrigger> trigger);
    
    // 处理并移除触发的粒子
    void Process(ChSystem& system) override;
    
    // 统计信息
    int GetNumRemoved() const;
};
```

## 典型使用模式

### 基本粒子发射
```cpp
// 创建发射器
auto emitter = chrono_types::make_shared<ChParticleEmitter>();

// 配置发射速率
emitter->SetParticlesPerSecond(100);  // 100 粒子/秒

// 形状创建器
auto shape_creator = chrono_types::make_shared<ChRandomShapeCreator>();
shape_creator->AddSphere(0.7, 0.01, 0.05);  // 70% 球体，半径 1-5cm
shape_creator->AddBox(0.3, ChVector3(0.01, 0.01, 0.01),
                           ChVector3(0.05, 0.05, 0.05));  // 30% 盒子
shape_creator->SetDensity(1000);
emitter->SetShapeCreator(shape_creator);

// 位置生成器（从盒子区域发射）
auto pos_gen = chrono_types::make_shared<ChRandomParticlePositionBox>(
    ChVector3(-1, 10, -1),
    ChVector3(1, 11, 1)
);
emitter->SetPositionGenerator(pos_gen);

// 速度生成器
auto vel_gen = chrono_types::make_shared<ChRandomParticleVelocityBox>(
    ChVector3(-0.5, -2, -0.5),
    ChVector3(0.5, -1, 0.5)
);
emitter->SetVelocityGenerator(vel_gen);

// 材料
auto material = chrono_types::make_shared<ChContactMaterialSMC>();
material->SetFriction(0.5f);
emitter->SetMaterial(material);

// 仿真循环
while (simulating) {
    emitter->EmitParticles(system, dt);
    system.DoStepDynamics(dt);
}
```

### 粒子移除
```cpp
// 创建移除器
auto remover = chrono_types::make_shared<ChParticleRemover>();

// 移除盒子外的粒子
auto trigger_box = chrono_types::make_shared<ChParticleEventTriggerBox>(
    ChVector3(-5, -5, -5),
    ChVector3(5, 5, 5),
    ChParticleEventTriggerBox::OUTSIDE
);
remover->AddTrigger(trigger_box);

// 移除旧粒子（超过 10 秒）
auto trigger_time = chrono_types::make_shared<ChParticleEventTriggerTime>(10.0);
remover->AddTrigger(trigger_time);

// 在仿真循环中处理
while (simulating) {
    emitter->EmitParticles(system, dt);
    remover->Process(system);
    system.DoStepDynamics(dt);
    
    std::cout << "已移除：" << remover->GetNumRemoved() << std::endl;
}
```

### 高级发射模式
```cpp
// 从圆柱区域发射
auto pos_cylinder = chrono_types::make_shared<ChRandomParticlePositionCylinder>(
    ChVector3(0, 5, 0),    // 中心
    ChVector3(0, 1, 0),    // 轴（Y 方向）
    0.5,                    // 半径
    2.0                     // 高度
);

// 使用正态分布速度发射
auto vel_normal = chrono_types::make_shared<ChRandomParticleVelocityNormal>(
    ChVector3(0, -1, 0),   // 方向
    5.0,                    // 平均速度
    1.0                     // 标准偏差
);

emitter->SetPositionGenerator(pos_cylinder);
emitter->SetVelocityGenerator(vel_normal);
```

### 基于事件的处理
```cpp
// 进入区域的粒子的自定义处理器
class ChParticleCounter : public ChParticleProcessor {
    int count = 0;
    ChParticleEventTriggerBox trigger;
    
public:
    void Process(ChSystem& system) override {
        for (auto& body : system.GetBodies()) {
            if (trigger.CheckTrigger(body, system)) {
                count++;
            }
        }
    }
    
    int GetCount() const { return count; }
};

auto counter = std::make_shared<ChParticleCounter>();
// 在仿真中使用...
```

## 使用案例

### 1. 颗粒流
```cpp
// 模拟颗粒材料倾倒
emitter->SetParticlesPerSecond(1000);
shape_creator->AddSphere(1.0, 0.005, 0.01);  // 5-10mm 球体
// ... 在料斗上方配置位置 ...
// ... 在底部移除粒子 ...
```

### 2. 粒子床
```cpp
// 创建初始粒子床
emitter->SetParticlesPerSecond(10000);  // 快速填充
auto pos_bed = chrono_types::make_shared<ChRandomParticlePositionBox>(
    bed_min, bed_max);
// 短时间发射以填充床层
emitter->EmitParticles(system, 1.0);
// 然后停止发射
```

### 3. 输送系统
```cpp
// 移动输送带上的粒子
emitter->SetParticlesPerSecond(50);
// 位置：输送带上方
// 速度：初始匹配输送带速度
auto vel_conveyor = chrono_types::make_shared<ChRandomParticleVelocityConstant>(
    ChVector3(1.0, 0, 0));  // 带速
```

### 4. 喷雾/射流
```cpp
// 喷雾模式
auto pos_nozzle = chrono_types::make_shared<ChRandomParticlePositionSphere>(
    nozzle_pos, 0.01);  // 小喷嘴
auto vel_spray = chrono_types::make_shared<ChRandomParticleVelocityNormal>(
    spray_direction, 10.0, 2.0);  // 定向喷雾
```

## 关键设计决策

### 1. 模块化生成
**决策**：独立的形状、位置、速度生成器
**理由**：
- 独立配置
- 可重用组件
- 易于使用自定义生成器扩展
- 混合和匹配模式

### 2. 基于概率的形状
**决策**：形状创建器使用概率分布
**理由**：
- 真实的粒子混合
- 控制形状分布
- 简单配置
- 适用于不同应用的灵活性

### 3. 基于事件的处理
**决策**：基于触发器的粒子处理
**理由**：
- 灵活的条件处理
- 可组合触发器
- 高效过滤
- 可扩展用于自定义事件

### 4. 工厂模式
**决策**：粒子生成的创建者模式
**理由**：
- 一致的粒子创建
- 自动设置（质量、惯性、碰撞）
- 材料应用
- 清晰的 API

## 性能特性

### 优势
1. **批量创建**：高效的多粒子发射
2. **惰性求值**：仅在需要时创建粒子
3. **高效移除**：无需搜索的直接刚体移除
4. **随机生成**：快速随机数生成

### 注意事项
1. **大量粒子**：系统性能随粒子数量下降
2. **碰撞检测**：宽相位成本随粒子增加
3. **内存**：每个粒子都是有开销的刚体
4. **接触生成**：粒子间有许多接触

## 总结

particle factory 模块提供：
- 灵活的粒子发射系统
- 随机形状、位置和速度生成
- 基于事件的粒子处理
- 自动粒子移除
- 支持各种发射模式

其设计强调模块化和可扩展性，使为颗粒材料、DEM 仿真和粒子效果创建复杂的粒子系统变得容易。
