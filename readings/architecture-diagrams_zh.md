# Chrono 架构图表

本文档包含展示 Project Chrono 架构的可视化图表。

## 整体系统架构

```mermaid
graph TB
    subgraph "用户接口层"
        CPP[C++ API]
        Python[Python/PyChrono]
        CSharp[C# 绑定]
        ROS2[ROS2 接口]
    end
    
    subgraph "核心引擎层"
        System[ChSystem<br/>NSC/SMC]
        Assembly[ChAssembly<br/>对象集合]
        Solver[求解器层<br/>VI/LS/Direct]
        Timestepper[时间步进器<br/>积分]
        Collision[碰撞系统<br/>Bullet/Multicore]
    end
    
    subgraph "物理对象层"
        Bodies[刚体<br/>ChBody]
        Joints[关节/连接<br/>ChLink]
        FEA[有限元<br/>ChMesh]
        Particles[粒子<br/>DEM/SPH]
    end
    
    subgraph "可选模块"
        Multicore[Multicore<br/>OpenMP 并行]
        Vehicle[Vehicle<br/>轮式/履带式]
        FSI[FSI<br/>流固耦合]
        DEM[DEM<br/>GPU 颗粒]
        Sensor[Sensor<br/>GPU 传感器]
        SynChrono[SynChrono<br/>分布式]
    end
    
    subgraph "可视化与 I/O"
        Irrlicht[Irrlicht<br/>OpenGL]
        VSG[VSG<br/>Vulkan]
        Postprocess[Postprocess<br/>导出]
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

## 核心物理引擎栈

```mermaid
graph TB
    subgraph "应用层"
        UserCode[用户应用代码]
    end
    
    subgraph "系统管理层"
        ChSystem[ChSystem<br/>系统管理器]
    end
    
    subgraph "对象管理层"
        ChAssembly[ChAssembly<br/>物理对象]
        ChBody[ChBody<br/>刚体]
        ChLink[ChLink<br/>约束]
        ChMesh[ChMesh<br/>有限元网格]
    end
    
    subgraph "计算层"
        Descriptor[ChSystemDescriptor<br/>自由度与约束]
        Solver[ChSolver<br/>方程求解器]
        Timestepper[ChTimestepper<br/>时间积分]
    end
    
    subgraph "碰撞层"
        CollisionSys[ChCollisionSystem<br/>检测]
        ContactContainer[ChContactContainer<br/>接触存储]
    end
    
    subgraph "支持层"
        Math[数学库<br/>Eigen]
        Geometry[几何<br/>形状与网格]
        Serialization[序列化<br/>归档]
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

## 模块依赖图

```mermaid
graph TB
    Core[Chrono Core<br/>物理引擎]
    
    Core --> Multicore[Multicore<br/>OpenMP + Thrust]
    Core --> Vehicle[Vehicle<br/>地面车辆]
    Core --> FSI[FSI<br/>SPH 流体]
    Core --> DEM[DEM<br/>GPU 颗粒]
    Core --> Sensor[Sensor<br/>CUDA + OptiX]
    Core --> ROS[ROS<br/>ROS2 桥接]
    Core --> Irrlicht[Irrlicht<br/>OpenGL 可视化]
    Core --> VSG[VSG<br/>Vulkan 可视化]
    Core --> Postprocess[Postprocess<br/>I/O 后处理]
    Core --> Parsers[Parsers<br/>URDF/OpenSim]
    Core --> Modal[Modal<br/>模态分析]
    Core --> Peridynamics[Peridynamics<br/>近场动力学]
    
    Vehicle --> SynChrono[SynChrono<br/>MPI 分布式]
    Vehicle --> Sensor
    
    Multicore --> Granular[颗粒动力学]
    FSI --> SPH[SPH 求解器]
    DEM --> GPUCompute[GPU 计算]
    Sensor --> GPUCompute
    
    style Core fill:#f96,stroke:#333,stroke-width:4px
    style Multicore fill:#9f9
    style Vehicle fill:#9f9
    style FSI fill:#9f9
    style DEM fill:#99f
    style Sensor fill:#99f
    style SynChrono fill:#f9f
```

## 求解器层次结构

```mermaid
classDiagram
    ChSolver <|-- ChIterativeSolver
    ChSolver <|-- ChDirectSolver
    
    ChIterativeSolver <|-- ChSolverVI
    ChIterativeSolver <|-- ChSolverLS
    
    ChSolverVI <|-- ChSolverAPGD
    ChSolverVI <|-- ChSolverBB
    ChSolverVI <|-- ChSolverPMINRES
    
    ChSolverLS <|-- ChSolverGMRES
    ChSolverLS <|-- ChSolverBiCGSTAB
    ChSolverLS <|-- ChSolverMINRES
    
    ChDirectSolver <|-- ChSolverSparseQR
    ChDirectSolver <|-- ChSolverSparseLU
    ChDirectSolver <|-- ChSolverPardisoMKL
    ChDirectSolver <|-- ChSolverMumps
    
    class ChSolver{
        +Solve()
        +SetMaxIterations()
        +SetTolerance()
    }
    
    class ChIterativeSolver{
        +迭代求解器
        +适用于大规模问题
    }
    
    class ChSolverVI{
        +变分不等式求解器
        +处理接触和摩擦
    }
    
    class ChSolverLS{
        +线性系统求解器
        +光滑问题
    }
    
    class ChDirectSolver{
        +直接求解器
        +精确求解小规模问题
    }
```

## 并行化策略

```mermaid
graph LR
    Serial[串行<br/>单核 CPU]
    OpenMP[OpenMP<br/>多核 CPU]
    CUDA[CUDA<br/>单 GPU]
    MultiGPU[多 GPU<br/>DEM]
    MPI[MPI<br/>分布式集群]
    
    Serial -->|Multicore 模块| OpenMP
    Serial -->|DEM/Sensor 模块| CUDA
    CUDA -->|DEM 双 GPU| MultiGPU
    OpenMP -->|SynChrono 模块| MPI
    
    style Serial fill:#faa
    style OpenMP fill:#9f9
    style CUDA fill:#99f
    style MultiGPU fill:#99f
    style MPI fill:#f9f
```

## 车辆模块架构

```mermaid
graph TB
    subgraph "车辆系统"
        Vehicle[ChVehicle<br/>车辆基类]
        WheeledVehicle[ChWheeledVehicle<br/>轮式车辆]
        TrackedVehicle[ChTrackedVehicle<br/>履带式车辆]
    end
    
    subgraph "底盘子系统"
        Chassis[ChChassis<br/>车架]
        Suspension[ChSuspension<br/>悬挂]
        Steering[ChSteering<br/>转向]
        Driveline[ChDriveline<br/>传动系统]
        Brake[ChBrake<br/>制动]
    end
    
    subgraph "动力系统"
        Engine[ChEngine<br/>发动机]
        Transmission[ChTransmission<br/>变速器]
    end
    
    subgraph "轮胎/履带"
        Tire[ChTire<br/>轮胎模型]
        TrackShoe[ChTrackShoe<br/>履带板]
    end
    
    subgraph "地形"
        Terrain[ChTerrain<br/>地形基类]
        RigidTerrain[RigidTerrain<br/>刚性地形]
        GranularTerrain[GranularTerrain<br/>颗粒地形]
        SCMTerrain[SCMTerrain<br/>SCM 地形]
    end
    
    Vehicle --> WheeledVehicle
    Vehicle --> TrackedVehicle
    
    WheeledVehicle --> Chassis
    WheeledVehicle --> Suspension
    WheeledVehicle --> Steering
    WheeledVehicle --> Driveline
    WheeledVehicle --> Brake
    WheeledVehicle --> Engine
    WheeledVehicle --> Transmission
    WheeledVehicle --> Tire
    
    TrackedVehicle --> Chassis
    TrackedVehicle --> Driveline
    TrackedVehicle --> Brake
    TrackedVehicle --> Engine
    TrackedVehicle --> TrackShoe
    
    Tire --> Terrain
    TrackShoe --> Terrain
    
    Terrain --> RigidTerrain
    Terrain --> GranularTerrain
    Terrain --> SCMTerrain
    
    style Vehicle fill:#9f9,stroke:#333,stroke-width:3px
    style Terrain fill:#fc9
```

## 传感器模块管线

```mermaid
graph LR
    Scene[场景配置<br/>ChSystem]
    
    subgraph "传感器管理器"
        SensorManager[ChSensorManager<br/>GPU 场景]
    end
    
    subgraph "传感器类型"
        Camera[ChCameraSensor<br/>相机]
        Lidar[ChLidarSensor<br/>激光雷达]
        Radar[ChRadarSensor<br/>毫米波雷达]
        GPS[ChGPSSensor<br/>GPS]
        IMU[ChIMUSensor<br/>IMU]
    end
    
    subgraph "渲染管线"
        Optix[OptiX<br/>光线追踪]
        Raster[栅格化<br/>OpenGL]
    end
    
    subgraph "后处理"
        Filters[滤波器<br/>噪声/失真]
    end
    
    subgraph "输出"
        Data[传感器数据<br/>图像/点云]
    end
    
    Scene --> SensorManager
    
    SensorManager --> Camera
    SensorManager --> Lidar
    SensorManager --> Radar
    SensorManager --> GPS
    SensorManager --> IMU
    
    Camera --> Raster
    Lidar --> Optix
    Radar --> Optix
    
    Optix --> Filters
    Raster --> Filters
    
    GPS --> Data
    IMU --> Data
    Filters --> Data
    
    style SensorManager fill:#99f,stroke:#333,stroke-width:3px
```

## 典型工作流：颗粒地形上的车辆

```mermaid
sequenceDiagram
    participant User as 用户代码
    participant Sys as ChSystem
    participant Veh as ChVehicle
    participant Terr as GranularTerrain
    participant Multi as Multicore
    participant Solver as ChSolver
    
    User->>Sys: 创建系统
    User->>Veh: 初始化车辆
    User->>Terr: 创建颗粒地形
    User->>Sys: SetMulticoreMode()
    
    loop 仿真循环
        User->>Veh: Synchronize(time)
        Veh->>Terr: GetForces()
        Terr-->>Veh: 地形反力
        
        User->>Sys: DoStepDynamics(dt)
        
        Sys->>Multi: 碰撞检测
        Multi-->>Sys: 接触对
        
        Sys->>Solver: 求解约束
        Solver-->>Sys: 加速度
        
        Sys->>Sys: 时间积分
        
        User->>Veh: Advance(dt)
    end
```

## ROS 集成架构

```mermaid
graph TB
    subgraph "Chrono 端"
        ChronoSys[ChSystem<br/>物理仿真]
        ChronoSensor[ChSensorManager<br/>传感器]
        ChronoVehicle[ChVehicle<br/>车辆]
    end
    
    subgraph "ROS 桥接层"
        ROSManager[ChROSManager<br/>ROS 管理器]
        Publishers[发布器]
        Subscribers[订阅器]
        TFBroadcaster[TF 广播器]
    end
    
    subgraph "ROS2 端"
        ROSTopics[ROS Topics]
        RVIZ[RViz<br/>可视化]
        Controllers[控制节点]
    end
    
    ChronoSys --> ROSManager
    ChronoSensor --> ROSManager
    ChronoVehicle --> ROSManager
    
    ROSManager --> Publishers
    ROSManager --> Subscribers
    ROSManager --> TFBroadcaster
    
    Publishers --> ROSTopics
    Subscribers --> ROSTopics
    TFBroadcaster --> ROSTopics
    
    ROSTopics --> RVIZ
    ROSTopics --> Controllers
    Controllers --> ROSTopics
    
    style ROSManager fill:#9cf,stroke:#333,stroke-width:3px
```

---

**说明**：所有图表使用 Mermaid 格式，可在 GitHub、GitLab、VS Code（需扩展）或 [mermaid.live](https://mermaid.live) 上查看。
