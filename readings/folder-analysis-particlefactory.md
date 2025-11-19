# Particle Factory Module Architecture Analysis

## Overview

The `src/chrono/particlefactory` folder provides a particle generation and management system for creating, emitting, and processing particles in simulations. It supports various emission patterns, particle shapes, and event-based processing.

## Main Functionality

### Primary Responsibilities
1. **Particle Emission**: Create particles at specified rates and patterns
2. **Shape Generation**: Random particle shapes (spheres, boxes, etc.)
3. **Position/Velocity**: Random placement and initial velocities
4. **Event Triggers**: Condition-based particle processing
5. **Particle Removal**: Automatic cleanup of particles
6. **Batch Processing**: Efficient multi-particle operations

## File Structure

```
ChParticleEmitter.h/cpp         - Main particle emitter
ChRandomShapeCreator.h/cpp      - Random shape generation
ChRandomParticlePosition.h/cpp  - Random position generation
ChRandomParticleVelocity.h/cpp  - Random velocity generation
ChParticleEventTrigger.h/cpp    - Event-based triggers
ChParticleProcessor.h/cpp       - Particle processing base
ChParticleProcessEvent.h/cpp    - Event-based processing
ChParticleRemover.h/cpp         - Particle removal
```

## Architecture Diagram

```mermaid
graph TB
    subgraph "Emitter Core"
        EMIT[ChParticleEmitter]
    end
    
    subgraph "Random Generators"
        RSHAPE[ChRandomShapeCreator]
        RPOS[ChRandomParticlePosition]
        RVEL[ChRandomParticleVelocity]
    end
    
    subgraph "Processing"
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

## Core External Interfaces

### 1. Particle Emitter (ChParticleEmitter.h)
```cpp
class ChApi ChParticleEmitter {
public:
    // Construction
    ChParticleEmitter();
    
    // Configuration
    void SetParticlesPerSecond(double rate);
    void SetShapeCreator(std::shared_ptr<ChRandomShapeCreator> creator);
    void SetPositionGenerator(std::shared_ptr<ChRandomParticlePosition> gen);
    void SetVelocityGenerator(std::shared_ptr<ChRandomParticleVelocity> gen);
    
    // Material
    void SetMaterial(std::shared_ptr<ChContactMaterial> material);
    
    // Emission
    void EmitParticles(ChSystem& system, double dt);
    
    // Statistics
    int GetNumEmittedTotal() const;
    int GetNumEmittedLastStep() const;
};
```

### 2. Random Shape Creator (ChRandomShapeCreator.h)
```cpp
class ChApi ChRandomShapeCreator {
public:
    // Add shape distributions
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
    
    // Generate random shape
    std::shared_ptr<ChBody> CreateRandomBody(ChSystem& system);
    
    // Density
    void SetDensity(double density);
};
```

### 3. Random Position (ChRandomParticlePosition.h)
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

### 4. Random Velocity (ChRandomParticleVelocity.h)
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

### 5. Event Trigger (ChParticleEventTrigger.h)
```cpp
class ChApi ChParticleEventTrigger {
public:
    virtual bool CheckTrigger(std::shared_ptr<ChBody> particle,
                             ChSystem& system) = 0;
};

class ChApi ChParticleEventTriggerBox : public ChParticleEventTrigger {
public:
    // Trigger when particle enters/exits box
    enum Mode { INSIDE, OUTSIDE };
    
    ChParticleEventTriggerBox(const ChVector3& box_min,
                             const ChVector3& box_max,
                             Mode mode);
    
    bool CheckTrigger(std::shared_ptr<ChBody> particle,
                     ChSystem& system) override;
};

class ChApi ChParticleEventTriggerTime : public ChParticleEventTrigger {
public:
    // Trigger after specified time
    ChParticleEventTriggerTime(double time_threshold);
    
    bool CheckTrigger(std::shared_ptr<ChBody> particle,
                     ChSystem& system) override;
};
```

### 6. Particle Processor (ChParticleProcessor.h)
```cpp
class ChApi ChParticleProcessor {
public:
    // Process all particles
    virtual void Process(ChSystem& system) = 0;
    
    // Filter particles
    void SetParticleFilter(std::function<bool(std::shared_ptr<ChBody>)> filter);
};

class ChApi ChParticleRemover : public ChParticleProcessor {
public:
    // Add removal trigger
    void AddTrigger(std::shared_ptr<ChParticleEventTrigger> trigger);
    
    // Process and remove particles that trigger
    void Process(ChSystem& system) override;
    
    // Statistics
    int GetNumRemoved() const;
};
```

## Typical Usage Patterns

### Basic Particle Emission
```cpp
// Create emitter
auto emitter = chrono_types::make_shared<ChParticleEmitter>();

// Configure emission rate
emitter->SetParticlesPerSecond(100);  // 100 particles/second

// Shape creator
auto shape_creator = chrono_types::make_shared<ChRandomShapeCreator>();
shape_creator->AddSphere(0.7, 0.01, 0.05);  // 70% spheres, radius 1-5cm
shape_creator->AddBox(0.3, ChVector3(0.01, 0.01, 0.01),
                           ChVector3(0.05, 0.05, 0.05));  // 30% boxes
shape_creator->SetDensity(1000);
emitter->SetShapeCreator(shape_creator);

// Position generator (emit from box region)
auto pos_gen = chrono_types::make_shared<ChRandomParticlePositionBox>(
    ChVector3(-1, 10, -1),
    ChVector3(1, 11, 1)
);
emitter->SetPositionGenerator(pos_gen);

// Velocity generator
auto vel_gen = chrono_types::make_shared<ChRandomParticleVelocityBox>(
    ChVector3(-0.5, -2, -0.5),
    ChVector3(0.5, -1, 0.5)
);
emitter->SetVelocityGenerator(vel_gen);

// Material
auto material = chrono_types::make_shared<ChContactMaterialSMC>();
material->SetFriction(0.5f);
emitter->SetMaterial(material);

// Simulation loop
while (simulating) {
    emitter->EmitParticles(system, dt);
    system.DoStepDynamics(dt);
}
```

### Particle Removal
```cpp
// Create remover
auto remover = chrono_types::make_shared<ChParticleRemover>();

// Remove particles outside box
auto trigger_box = chrono_types::make_shared<ChParticleEventTriggerBox>(
    ChVector3(-5, -5, -5),
    ChVector3(5, 5, 5),
    ChParticleEventTriggerBox::OUTSIDE
);
remover->AddTrigger(trigger_box);

// Remove old particles (older than 10 seconds)
auto trigger_time = chrono_types::make_shared<ChParticleEventTriggerTime>(10.0);
remover->AddTrigger(trigger_time);

// Process in simulation loop
while (simulating) {
    emitter->EmitParticles(system, dt);
    remover->Process(system);
    system.DoStepDynamics(dt);
    
    std::cout << "Removed: " << remover->GetNumRemoved() << std::endl;
}
```

### Advanced Emission Patterns
```cpp
// Emit from cylindrical region
auto pos_cylinder = chrono_types::make_shared<ChRandomParticlePositionCylinder>(
    ChVector3(0, 5, 0),    // center
    ChVector3(0, 1, 0),    // axis (Y direction)
    0.5,                    // radius
    2.0                     // height
);

// Emit with normal distribution velocity
auto vel_normal = chrono_types::make_shared<ChRandomParticleVelocityNormal>(
    ChVector3(0, -1, 0),   // direction
    5.0,                    // mean speed
    1.0                     // std deviation
);

emitter->SetPositionGenerator(pos_cylinder);
emitter->SetVelocityGenerator(vel_normal);
```

### Event-Based Processing
```cpp
// Custom processor for particles entering a region
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
// Use in simulation...
```

## Use Cases

### 1. Granular Flow
```cpp
// Simulate granular material pouring
emitter->SetParticlesPerSecond(1000);
shape_creator->AddSphere(1.0, 0.005, 0.01);  // 5-10mm spheres
// ... configure position above hopper ...
// ... remove particles at bottom ...
```

### 2. Particle Bed
```cpp
// Create initial particle bed
emitter->SetParticlesPerSecond(10000);  // Fast filling
auto pos_bed = chrono_types::make_shared<ChRandomParticlePositionBox>(
    bed_min, bed_max);
// Emit for short time to fill bed
emitter->EmitParticles(system, 1.0);
// Then stop emission
```

### 3. Conveyor System
```cpp
// Particles on moving conveyor
emitter->SetParticlesPerSecond(50);
// Position: above conveyor belt
// Velocity: match conveyor speed initially
auto vel_conveyor = chrono_types::make_shared<ChRandomParticleVelocityConstant>(
    ChVector3(1.0, 0, 0));  // Belt speed
```

### 4. Spray/Jet
```cpp
// Spray pattern
auto pos_nozzle = chrono_types::make_shared<ChRandomParticlePositionSphere>(
    nozzle_pos, 0.01);  // Small nozzle
auto vel_spray = chrono_types::make_shared<ChRandomParticleVelocityNormal>(
    spray_direction, 10.0, 2.0);  // Directed spray
```

## Key Design Decisions

### 1. Modular Generation
**Decision**: Separate shape, position, velocity generators
**Rationale**:
- Independent configuration
- Reusable components
- Easy to extend with custom generators
- Mix and match patterns

### 2. Probability-Based Shapes
**Decision**: Shape creator uses probability distribution
**Rationale**:
- Realistic particle mixtures
- Control over shape distribution
- Simple configuration
- Flexible for different applications

### 3. Event-Based Processing
**Decision**: Trigger-based particle processing
**Rationale**:
- Flexible condition handling
- Composable triggers
- Efficient filtering
- Extensible for custom events

### 4. Factory Pattern
**Decision**: Creator pattern for particle generation
**Rationale**:
- Consistent particle creation
- Automatic setup (mass, inertia, collision)
- Material application
- Clean API

## Performance Characteristics

### Strengths
1. **Batch Creation**: Efficient multi-particle emission
2. **Lazy Evaluation**: Particles created only when needed
3. **Efficient Removal**: Direct body removal without search
4. **Random Generation**: Fast random number generation

### Considerations
1. **Many Particles**: System performance degrades with particle count
2. **Collision Detection**: Broadphase cost increases with particles
3. **Memory**: Each particle is a body with overhead
4. **Contact Generation**: Many contacts between particles

## Summary

The particle factory module provides:
- Flexible particle emission system
- Random shape, position, and velocity generation
- Event-based particle processing
- Automatic particle removal
- Support for various emission patterns

Its design emphasizes modularity and extensibility, making it easy to create complex particle systems for granular materials, DEM simulations, and particle effects.
