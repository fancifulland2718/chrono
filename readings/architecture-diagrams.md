# Chrono Architecture Diagrams

This document contains visual diagrams illustrating the Project Chrono architecture.

## Overall System Architecture

```mermaid
graph TB
    subgraph "User Interface Layer"
        CPP[C++ API]
        Python[Python/PyChrono]
        CSharp[C# Bindings]
        ROS2[ROS2 Interface]
    end
    
    subgraph "Core Engine Layer"
        System[ChSystem<br/>NSC/SMC]
        Assembly[ChAssembly<br/>Object Collection]
        Solver[Solver Layer<br/>VI/LS/Direct]
        Timestepper[Timestepper<br/>Integration]
        Collision[Collision System<br/>Bullet/Multicore]
    end
    
    subgraph "Physics Objects Layer"
        Bodies[Rigid Bodies<br/>ChBody]
        Joints[Joints/Links<br/>ChLink]
        FEA[FEA Elements<br/>ChMesh]
        Particles[Particles<br/>DEM/SPH]
    end
    
    subgraph "Optional Modules"
        Multicore[Multicore<br/>OpenMP Parallel]
        Vehicle[Vehicle<br/>Wheeled/Tracked]
        FSI[FSI<br/>Fluid-Solid]
        DEM[DEM<br/>GPU Granular]
        Sensor[Sensor<br/>GPU Sensors]
        SynChrono[SynChrono<br/>Distributed]
    end
    
    subgraph "Visualization & I/O"
        Irrlicht[Irrlicht<br/>OpenGL]
        VSG[VSG<br/>Vulkan]
        Postprocess[Postprocess<br/>Export]
    end
    
    CPP --> System
    Python --> System
    CSharp --> System
    ROS2 --> System
    
    System --> Assembly
    System --> Solver
    System --> Timestepper
    System --> Collision
    
    Assembly --> Bodies
    Assembly --> Joints
    Assembly --> FEA
    Assembly --> Particles
    
    System --> Multicore
    System --> Vehicle
    System --> FSI
    System --> DEM
    System --> Sensor
    System --> SynChrono
    
    System --> Irrlicht
    System --> VSG
    System --> Postprocess
    
    style System fill:#f96,stroke:#333,stroke-width:4px
    style Multicore fill:#9f9
    style Vehicle fill:#9f9
    style FSI fill:#9f9
    style DEM fill:#99f
    style Sensor fill:#99f
    style SynChrono fill:#f9f
```

## Core Physics Engine Stack

```mermaid
graph TB
    subgraph "Application Layer"
        UserCode[User Application Code]
    end
    
    subgraph "System Management Layer"
        ChSystem[ChSystem<br/>System Manager]
    end
    
    subgraph "Object Management Layer"
        ChAssembly[ChAssembly<br/>Physical Objects]
        ChBody[ChBody<br/>Rigid Bodies]
        ChLink[ChLink<br/>Constraints]
        ChMesh[ChMesh<br/>FEA Mesh]
    end
    
    subgraph "Computation Layer"
        Descriptor[ChSystemDescriptor<br/>DOF & Constraints]
        Solver[ChSolver<br/>Equation Solver]
        Timestepper[ChTimestepper<br/>Time Integration]
    end
    
    subgraph "Collision Layer"
        CollisionSys[ChCollisionSystem<br/>Detection]
        ContactContainer[ChContactContainer<br/>Contact Storage]
    end
    
    subgraph "Support Layer"
        Math[Math Library<br/>Eigen]
        Geometry[Geometry<br/>Shapes & Meshes]
        Serialization[Serialization<br/>Archive]
    end
    
    UserCode --> ChSystem
    ChSystem --> ChAssembly
    ChAssembly --> ChBody
    ChAssembly --> ChLink
    ChAssembly --> ChMesh
    
    ChSystem --> Descriptor
    ChSystem --> Solver
    ChSystem --> Timestepper
    ChSystem --> CollisionSys
    
    CollisionSys --> ContactContainer
    
    ChBody --> Math
    ChBody --> Geometry
    ChSystem --> Serialization
    
    style ChSystem fill:#f96,stroke:#333,stroke-width:3px
    style Solver fill:#bbf
    style CollisionSys fill:#bbf
```

## Module Dependency Graph

```mermaid
graph TB
    Core[Chrono Core<br/>Physics Engine]
    
    Core --> Multicore[Multicore<br/>OpenMP + Thrust]
    Core --> Vehicle[Vehicle<br/>Ground Vehicles]
    Core --> FSI[FSI<br/>SPH Fluid]
    Core --> DEM[DEM<br/>GPU Granular]
    Core --> Sensor[Sensor<br/>CUDA + OptiX]
    Core --> ROS[ROS<br/>ROS2 Bridge]
    Core --> Irrlicht[Irrlicht<br/>Visualization]
    Core --> VSG[VSG<br/>Vulkan Viz]
    Core --> Postprocess[Postprocess<br/>Export]
    Core --> Parsers[Parsers<br/>URDF/OpenSim]
    Core --> Modal[Modal<br/>Modal Analysis]
    Core --> FMI[FMI<br/>Co-simulation]
    
    Vehicle --> SynChrono[SynChrono<br/>Distributed MPI]
    Vehicle --> VehicleModels[Vehicle Models<br/>Predefined]
    
    Core --> MUMPS[MUMPS<br/>Sparse Solver]
    Core --> PardisoMKL[Pardiso MKL<br/>Sparse Solver]
    
    Sensor -.sensor data.-> ROS
    Vehicle -.can use.-> Multicore
    Vehicle -.can use.-> DEM
    Vehicle -.can use.-> FSI
    
    style Core fill:#f96,stroke:#333,stroke-width:4px
    style Multicore fill:#9f9
    style Vehicle fill:#9f9
    style DEM fill:#99f
    style Sensor fill:#99f
    style FSI fill:#9cf
    style SynChrono fill:#f9f
```

## Solver Hierarchy

```mermaid
graph TB
    ChSolver[ChSolver<br/>Abstract Base]
    
    ChSolver --> IterVI[ChIterativeSolverVI<br/>Variational Inequality]
    ChSolver --> IterLS[ChIterativeSolverLS<br/>Linear System]
    ChSolver --> DirectLS[ChDirectSolverLS<br/>Direct Solver]
    
    IterVI --> PSOR[ChSolverPSOR<br/>Projected SOR]
    IterVI --> PJacobi[ChSolverPJacobi<br/>Projected Jacobi]
    IterVI --> APGD[ChSolverAPGD<br/>Accelerated PGD]
    IterVI --> ADMM[ChSolverADMM<br/>ADMM]
    IterVI --> BB[ChSolverBB<br/>Barzilai-Borwein]
    
    IterLS --> PMINRES[ChSolverPMINRES<br/>Preconditioned MINRES]
    
    DirectLS --> SparseLU[Sparse LU]
    DirectLS --> SparseQR[Sparse QR]
    
    APGD -.used in.-> Multicore[Multicore Module]
    BB -.used in.-> Multicore
    DirectLS -.interface to.-> MUMPS[MUMPS Module]
    DirectLS -.interface to.-> Pardiso[Pardiso MKL]
    
    style ChSolver fill:#f96,stroke:#333,stroke-width:3px
    style IterVI fill:#9cf
    style IterLS fill:#9cf
    style DirectLS fill:#9cf
```

## Parallelization Strategy

```mermaid
graph LR
    subgraph "Serial Execution"
        Serial[Standard Chrono<br/>Single Thread]
    end
    
    subgraph "Shared Memory Parallel"
        OpenMP[Chrono::Multicore<br/>OpenMP Multi-thread<br/>100K+ contacts]
    end
    
    subgraph "GPU Acceleration"
        CUDA_DEM[Chrono::DEM<br/>CUDA Granular<br/>1M+ particles]
        CUDA_FSI[Chrono::FSI<br/>CUDA SPH<br/>100K+ fluid particles]
        CUDA_Sensor[Chrono::Sensor<br/>OptiX Ray Tracing<br/>Real-time rendering]
    end
    
    subgraph "Distributed Parallel"
        MPI[Chrono::SynChrono<br/>MPI Distributed<br/>Multi-agent simulation]
    end
    
    Serial --> OpenMP
    Serial --> CUDA_DEM
    Serial --> CUDA_FSI
    Serial --> CUDA_Sensor
    OpenMP --> MPI
    CUDA_DEM --> MPI
    
    style Serial fill:#ddd
    style OpenMP fill:#9f9
    style CUDA_DEM fill:#99f
    style CUDA_FSI fill:#99f
    style CUDA_Sensor fill:#99f
    style MPI fill:#f99
```

## Vehicle Module Architecture

```mermaid
graph TB
    ChVehicle[ChVehicle<br/>Base Class]
    
    ChVehicle --> Wheeled[ChWheeledVehicle]
    ChVehicle --> Tracked[ChTrackedVehicle]
    
    subgraph "Wheeled Subsystems"
        Chassis[Chassis]
        Suspension[Suspension<br/>DoubleWishbone<br/>MacPherson<br/>MultiLink]
        Tire[Tire<br/>Rigid<br/>Pac89<br/>TMeasy<br/>FEA]
        Driveline[Driveline<br/>2WD/4WD/AWD]
        Steering[Steering<br/>Pitman<br/>Rack-Pinion]
        Brake[Brake System]
        Wheel[Wheel]
    end
    
    subgraph "Tracked Subsystems"
        TrackAssembly[Track Assembly]
        TrackShoe[Track Shoe]
        Sprocket[Sprocket]
        Idler[Idler Wheel]
        RoadWheel[Road Wheel]
    end
    
    subgraph "Common Systems"
        Powertrain[Powertrain<br/>Engine + Transmission]
        Driver[Driver Model<br/>Interactive/Data-driven]
        Terrain[Terrain<br/>Rigid/SCM/Granular/FEA]
    end
    
    Wheeled --> Chassis
    Wheeled --> Suspension
    Wheeled --> Tire
    Wheeled --> Driveline
    Wheeled --> Steering
    Wheeled --> Brake
    Wheeled --> Wheel
    
    Tracked --> TrackAssembly
    TrackAssembly --> TrackShoe
    TrackAssembly --> Sprocket
    TrackAssembly --> Idler
    TrackAssembly --> RoadWheel
    
    ChVehicle --> Powertrain
    ChVehicle --> Driver
    ChVehicle --> Terrain
    
    style ChVehicle fill:#f96,stroke:#333,stroke-width:3px
    style Wheeled fill:#9f9
    style Tracked fill:#9f9
```

## Sensor Module Pipeline

```mermaid
graph LR
    subgraph "Scene Setup"
        Scene[3D Scene<br/>Meshes + Materials]
        Light[Lighting<br/>Point/Directional]
    end
    
    subgraph "Sensor Types"
        Camera[Camera<br/>RGB/Depth/Segmentation]
        Lidar[Lidar<br/>Point Cloud]
        Radar[Radar<br/>Detection]
        GPS[GPS<br/>Position]
        IMU[IMU<br/>Acceleration/Angular]
    end
    
    subgraph "Rendering"
        OptiX[OptiX Ray Tracing<br/>CUDA Kernels]
    end
    
    subgraph "Post-Processing"
        Filter1[Noise Filter]
        Filter2[Blur Filter]
        Filter3[Distortion]
        FilterN[Custom Filters]
    end
    
    subgraph "Output"
        Buffer[Data Buffer]
        File[Save to File]
        ROS[Publish to ROS]
        Display[Display Window]
    end
    
    Scene --> OptiX
    Light --> OptiX
    Camera --> OptiX
    Lidar --> OptiX
    Radar --> OptiX
    
    OptiX --> Filter1
    Filter1 --> Filter2
    Filter2 --> Filter3
    Filter3 --> FilterN
    
    FilterN --> Buffer
    Buffer --> File
    Buffer --> ROS
    Buffer --> Display
    
    GPS -.bypass rendering.-> Buffer
    IMU -.bypass rendering.-> Buffer
    
    style OptiX fill:#99f,stroke:#333,stroke-width:2px
    style Camera fill:#9cf
    style Lidar fill:#9cf
    style ROS fill:#f9f
```

## Typical Workflow: Vehicle on Granular Terrain

```mermaid
sequenceDiagram
    participant User as User Code
    participant Sys as ChSystem<br/>(Multicore/DEM)
    participant Veh as Vehicle
    participant Terr as Terrain
    participant Drv as Driver
    participant Vis as Visualization
    
    User->>Sys: Create System
    User->>Veh: Create Vehicle
    User->>Terr: Create Granular Terrain
    User->>Drv: Create Driver
    User->>Vis: Create Visualization
    
    Veh->>Sys: Add Vehicle Bodies
    Terr->>Sys: Add Terrain Particles
    
    loop Simulation Loop
        Drv->>Drv: GetInputs()
        
        Drv->>Veh: Synchronize(inputs)
        Veh->>Terr: Synchronize()
        Terr->>Veh: GetForces()
        
        Veh->>Sys: Update Bodies
        Drv->>Veh: Advance(dt)
        Veh->>Sys: Advance(dt)
        Terr->>Sys: Advance(dt)
        
        Sys->>Sys: Collision Detection
        Sys->>Sys: Solve Dynamics
        Sys->>Sys: Update States
        
        Sys->>Vis: Render()
    end
```

## ROS Integration Architecture

```mermaid
graph TB
    subgraph "Chrono Simulation"
        System[ChSystem]
        Vehicle[Vehicle System]
        Sensor[Sensor Manager]
        Bodies[Rigid Bodies]
        
        System --> Vehicle
        System --> Bodies
        Sensor --> System
    end
    
    subgraph "Chrono::ROS Bridge"
        ROSMgr[ChROSManager]
        ClockHandler[Clock Handler]
        BodyHandler[Body Handler]
        TFHandler[TF Handler]
        SensorHandler[Sensor Handler]
        
        ROSMgr --> ClockHandler
        ROSMgr --> BodyHandler
        ROSMgr --> TFHandler
        ROSMgr --> SensorHandler
    end
    
    subgraph "ROS2 Topics"
        ClockTopic[/clock]
        OdomTopic[/odom]
        TFTopic[/tf]
        ImageTopic[/camera/image]
        PCTopic[/lidar/points]
    end
    
    subgraph "ROS2 Applications"
        RViz[RViz2<br/>Visualization]
        Navigation[Nav2<br/>Navigation]
        Perception[Perception<br/>Node]
        Control[Control<br/>Node]
    end
    
    System --> ROSMgr
    Vehicle --> BodyHandler
    Sensor --> SensorHandler
    Bodies --> TFHandler
    
    ClockHandler --> ClockTopic
    BodyHandler --> OdomTopic
    TFHandler --> TFTopic
    SensorHandler --> ImageTopic
    SensorHandler --> PCTopic
    
    ClockTopic --> RViz
    OdomTopic --> Navigation
    TFTopic --> RViz
    ImageTopic --> Perception
    PCTopic --> Perception
    
    Control -.cmd_vel.-> ROSMgr
    
    style System fill:#f96
    style ROSMgr fill:#f9f
    style RViz fill:#9f9
    style Navigation fill:#9f9
```

---

**Note**: These diagrams use Mermaid syntax and can be rendered in:
- GitHub Markdown
- GitLab Markdown  
- Many Markdown editors (Typora, VS Code with Mermaid extension)
- Online Mermaid editors (mermaid.live)
