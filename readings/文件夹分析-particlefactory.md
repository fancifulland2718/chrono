# ParticleFactory 模块架构分析

## 概述

`src/chrono/particlefactory` 文件夹提供粒子系统的创建和管理工具，用于批量生成和处理大量粒子。

## 文件结构

```
ChParticleEmitter.h             - 粒子发射器
ChParticleProcessor.h           - 粒子处理器基类
ChParticleRemover.h             - 粒子移除器
ChParticleProcessEvent.h        - 事件驱动处理
ChParticleEventTrigger.h        - 事件触发器
ChRandomParticlePosition.h      - 随机位置生成
ChRandomParticleVelocity.h      - 随机速度生成
ChRandomParticleAlignment.h     - 随机方向生成
ChRandomShapeCreator.h          - 随机形状创建
```

## 使用场景

### 粒子发射器
```cpp
// 创建粒子发射器
ChParticleEmitter emitter;
emitter.SetParticlesPerSecond(100);

// 设置粒子属性
emitter.SetParticleMass(0.01);
emitter.SetParticleDensity(1000);
emitter.SetParticleRadius(0.02);

// 设置发射位置
ChRandomParticlePositionOnGeometry pos_generator;
pos_generator.SetGeometry(emitter_box);
emitter.SetParticlePositionGenerator(pos_generator);

// 设置速度分布
ChRandomParticleVelocity vel_generator;
vel_generator.SetModulusDistribution(
    chrono_types::make_shared<ChUniformDistribution>(1.0, 3.0)
);
emitter.SetParticleVelocityGenerator(vel_generator);

// 在仿真中发射
while (running) {
    emitter.EmitParticles(system, dt);
    system->DoStepDynamics(dt);
}
```

### 粒子移除器
```cpp
// 移除超出边界的粒子
ChParticleRemoverBox remover(
    ChVector3d(-10, -10, -10),
    ChVector3d(10, 10, 10),
    true  // 移除边界外的
);

// 每步检查并移除
while (running) {
    system->DoStepDynamics(dt);
    remover.ProcessParticles(system);
}
```

### 事件处理
```cpp
// 基于事件的粒子处理
class MyParticleProcessor : public ChParticleProcessor {
public:
    virtual void Process(std::shared_ptr<ChBody> particle, 
                        ChSystem& system) override {
        // 自定义处理逻辑
        if (particle->GetPos().y() < 0) {
            particle->SetBodyFixed(true);
        }
    }
};

MyParticleProcessor processor;
processor.ProcessParticles(system);
```

## 总结

ParticleFactory 模块简化了大规模粒子系统的创建和管理。
