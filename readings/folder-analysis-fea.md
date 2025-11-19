# FEA 模块架构分析

## 概述

`src/chrono/fea` folder implements the Finite Element Analysis (FEA) capabilities in Chrono, providing structural mechanics simulation including beams, shells, solids, and cables. It supports both small and large deformations with various element formulations.

## 主要功能

### 主要职责
1. **FEA Mesh Management**: Container for nodes and elements
2. **Node 类型**: 3D position, rotation, shell nodes, etc.
3. **Element 类型**: Beams, shells, solids, cables, springs
4. **Material Models**: Linear elastic, hyperelastic, viscoelastic
5. **Section 属性**: Beam and shell cross-sections
6. **Contact**: Node-node, node-face, surface contact
7. **Loads**: Distributed loads, body forces, surface loads
8. **集成**: Gauss quadrature and numerical integration

## File Structure and Relationships

### Core FEA Classes
```
ChMesh .h/cpp                - FEA mesh container
ChNodeFEAbase .h/cpp         - Base FEA node class
ChElementBase .h/cpp         - Base FEA element class
ChContinuumMaterial .h/cpp   - Material models
```

### Node 类型
```
ChNodeFEAxyz .h/cpp          - 3D position node (3 DOF)
ChNodeFEAxyzrot .h/cpp       - Position + small rotation (6 DOF)
ChNodeFEAxyzD .h/cpp         - Position + gradient (6 DOF)
ChNodeFEAxyzDD .h/cpp        - Position + 2 gradients (9 DOF)
ChNodeFEAxyzP .h/cpp         - Position + pressure (4 DOF)
```

### Beam Elements
```
ChElementBeam.h             - Base beam element
ChElementBeamEuler .h/cpp    - Euler-Bernoulli beam
ChElementBeamANCF_3243.h    - ANCF beam (3 nodes, 24/43 DOF)
ChElementBeamANCF_3333.h    - ANCF beam (3 nodes, 33 DOF)
ChElementBeamIGA .h/cpp      - Isogeometric beam
ChElementBeamTaperedTimoshenko .h/cpp - Tapered Timoshenko beam
ChElementCableANCF .h/cpp    - Cable element (no bending)
```

### Shell Elements
```
ChElementShell.h            - Base shell element
ChElementShellANCF_3423.h   - ANCF shell (4 nodes, 24 DOF)
ChElementShellANCF_3443.h   - ANCF shell (4 nodes, 36 DOF)
ChElementShellReissner4.h   - Reissner-Mindlin shell (4 nodes)
ChElementShellBST.h         - BST shell element
```

### Solid Elements
```
ChElementTetrahedron.h      - Tetrahedral element
ChElementHexahedron.h       - Hexahedral element (brick)
ChElementBrick.h            - 8-node brick element
ChElementBrick_9.h          - 9-node brick
```

### Special Elements
```
ChElementBar .h/cpp          - 2-node bar/truss
ChElementSpring .h/cpp       - Spring element
ChElementGeneric.h          - Generic element base
ChElementCorotational.h     - Corotational formulation wrapper
```

### Sections and Materials
```
ChBeamSection .h/cpp         - Beam cross-section properties
ChBeamSectionCable.h        - Cable section
ChBeamSectionEuler.h        - Euler beam section
ChBeamSectionTimoshenko.h   - Timoshenko beam section
ChBeamSectionShape.h        - Arbitrary cross-sections

ChContinuumElastic.h        - Linear elastic material
ChContinuumPlasticVonMises.h - Plasticity
ChContinuumPoisson3D.h      - Poisson material
ChMaterialShellANCF.h       - ANCF shell material
ChMaterialShellReissner.h   - Reissner shell material
```

### Contact and Constraints
```
ChContactSurface .h/cpp      - Base contact surface
ChContactSurfaceMesh .h/cpp  - Triangle mesh contact
ChContactSurfaceNodeCloud.h - Node cloud contact
ChLinkNodeFrame .h/cpp       - Link node to body
ChLinkNodeNode .h/cpp        - Link two nodes
ChLinkNodeFace .h/cpp        - Link node to element face
```

### Loads
```
ChLoad.h                    - Base load (from physics)
ChLoadsBeam .h/cpp           - Distributed beam loads
ChLoadsNodeXYZ.h            - Point loads on nodes
ChLoadBodyMesh.h            - Distributed surface loads
```

### Mesh 操作
```
ChMeshFileLoader .h/cpp      - Load mesh from file (Abaqus, etc.)
ChMeshExporter .h/cpp        - Export mesh to file
ChMeshSurface .h/cpp         - Extract mesh surface
```

## 架构图

```mermaid
graph TB
    subgraph "Mesh Container"
        MESH[ChMesh]
    end
    
    subgraph "Nodes"
        NODE[ChNodeFEAbase]
        NXYZ[ChNodeFEAxyz]
        NROT[ChNodeFEAxyzrot]
        ND[ChNodeFEAxyzD]
    end
    
    subgraph "Elements"
        ELEM[ChElementBase]
        BEAM[ChElementBeam]
        SHELL[ChElementShell]
        SOLID[ChElementTetrahedron]
        CABLE[ChElementCableANCF]
    end
    
    subgraph "Materials & Sections"
        MAT[ChContinuumMaterial]
        SEC[ChBeamSection]
    end
    
    subgraph "Contact"
        CONT[ChContactSurface]
        CONTM[ChContactSurfaceMesh]
    end
    
    subgraph "Loads"
        LOAD[ChLoad]
        LBEAM[ChLoadsBeam]
    end
    
    MESH --> NODE
    MESH --> ELEM
    
    NXYZ --|> NODE
    NROT --|> NODE
    ND --|> NODE
    
    BEAM --|> ELEM
    SHELL --|> ELEM
    SOLID --|> ELEM
    CABLE --|> ELEM
    
    ELEM --> NODE
    ELEM --> MAT
    BEAM --> SEC
    
    MESH --> CONT
    CONTM --|> CONT
    
    LOAD --> ELEM
    LBEAM --|> LOAD
    
    style MESH fill:#e1f5ff
    style NODE fill:#ffe1f5
    style ELEM fill:#fff5e1
    style MAT fill:#e1ffe1
```

## 核心外部接口

### 1. FEA Mesh (ChMesh.h)
```cpp
class ChApi ChMesh : public ChIndexedNodes, public ChPhysicsItem {
public:
    // Add nodes
    void AddNode(std::shared_ptr<ChNodeFEAbase> node);
    
    // Add elements
    void AddElement(std::shared_ptr<ChElementBase> elem);
    
    // 访问
    size_t GetNumNodes() const;
    size_t GetNumElements() const;
    std::shared_ptr<ChNodeFEAbase> GetNode(size_t i);
    std::shared_ptr<ChElementBase> GetElement(size_t i);
    
    // Contact surface
    void AddContactSurface(std::shared_ptr<ChContactSurface> surf);
    
    // Automatic DOF setup
    void SetupInitial() override;
    
    // Mass/stiffness matrices
    void ComputeMassMatrix(ChSparseMatrix& M);
    void ComputeStiffnessMatrix(ChSparseMatrix& K);
};
```

### 2. FEA Node (ChNodeFEAbase.h, ChNodeFEAxyz.h)
```cpp
class ChApi ChNodeFEAbase : public ChNodeBase {
public:
    // DOF information
    virtual int GetNumCoordsPosLevel() const = 0;
    virtual int GetNumCoordsVelLevel() const = 0;
    
    // 状态
    virtual void NodeIntStateGather(int offset_x,
                                    ChState& x,
                                    int offset_v,
                                    ChStateDelta& v) = 0;
    
    virtual void NodeIntStateScatter(int offset_x,
                                    const ChState& x,
                                    int offset_v,
                                    const ChStateDelta& v) = 0;
};

// 3D position node
class ChApi ChNodeFEAxyz : public ChNodeFEAbase {
public:
    ChNodeFEAxyz(const ChVector3& pos = VNULL);
    
    // Position
    void SetPos(const ChVector3& pos);
    const ChVector3& GetPos() const;
    
    // Velocity
    void SetPosDt(const ChVector3& vel);
    const ChVector3& GetPosDt() const;
    
    // Force
    void SetForce(const ChVector3& force);
    const ChVector3& GetForce() const;
    
    // Mass
    void SetMass(double mass);
    double GetMass() const;
    
    // 3 DOF
    int GetNumCoordsPosLevel() const override { return 3; }
};
```

### 3. FEA Element (ChElementBase.h)
```cpp
class ChApi ChElementBase {
public:
    // Number of nodes
    virtual int GetNumNodes() = 0;
    
    // Node access
    virtual std::shared_ptr<ChNodeFEAbase> GetNode(int n) = 0;
    
    // Matrices
    virtual void ComputeKRMmatricesGlobal(ChMatrixRef H,
                                         double Kfactor,
                                         double Rfactor = 0,
                                         double Mfactor = 0) = 0;
    
    virtual void ComputeInternalForces(ChVectorDynamic<>& Fi) = 0;
    
    // Strain/stress
    virtual void EvaluateStrain(const ChVector3& loc,
                               ChVectorDynamic<>& strain);
    virtual void EvaluateStress(const ChVector3& loc,
                               ChVectorDynamic<>& stress);
};
```

### 4. Beam Element (ChElementBeamEuler.h)
```cpp
class ChApi ChElementBeamEuler : public ChElementBeam {
public:
    ChElementBeamEuler();
    
    // Setup
    void SetNodes(std::shared_ptr<ChNodeFEAxyzrot> nodeA,
                 std::shared_ptr<ChNodeFEAxyzrot> nodeB);
    
    void SetSection(std::shared_ptr<ChBeamSectionEuler> section);
    
    // 2 nodes, 6 DOF each = 12 DOF total
    int GetNumNodes() override { return 2; }
    int GetNumCoordsPosLevel() override { return 12; }
    
    // Evaluation
    void EvaluateSectionFrame(double eta,
                             ChVector3& pos,
                             ChQuaternion<>& rot);
    
    void EvaluateSectionStrain(double eta,
                              ChVector3& strain_e,
                              ChVector3& strain_k);
};
```

### 5. Material Model (ChContinuumMaterial.h)
```cpp
class ChApi ChContinuumElastic : public ChContinuumMaterial {
public:
    // Set material properties
    void SetYoungModulus(double E);
    void SetPoissonRatio(double nu);
    void SetDensity(double rho);
    
    double GetYoungModulus() const;
    double GetPoissonRatio() const;
    
    // Derived properties
    double GetShearModulus() const;  // G = E / (2*(1+nu))
    double GetBulkModulus() const;   // K = E / (3*(1-2*nu))
    
    // Stress computation
    virtual void ComputeStress(ChVectorDynamic<>& stress,
                              const ChVectorDynamic<>& strain);
    
    // Tangent modulus
    virtual void ComputeTangent(ChMatrixDynamic<>& tangent);
};
```

### 6. Beam Section (ChBeamSection.h)
```cpp
class ChApi ChBeamSectionEuler : public ChBeamSection {
public:
    // Geometric properties
    void SetArea(double A);
    void SetIyy(double Iyy);  // Moment of inertia about y
    void SetIzz(double Izz);  // Moment of inertia about z
    void SetJ(double J);       // Polar moment (torsion)
    
    double GetArea() const;
    double GetIyy() const;
    
    // Material
    void SetYoungModulus(double E);
    void SetShearModulus(double G);
    void SetDensity(double rho);
    
    // Stiffness matrix (6x6 for beam)
    void ComputeStiffnessMatrix(ChMatrixDynamic<>& K);
};
```

### 7. Contact Surface (ChContactSurface.h)
```cpp
class ChApi ChContactSurfaceMesh : public ChContactSurface {
public:
    // Setup from mesh faces
    void AddFacesFromBoundary(double sphere_swept = 0.0,
                             bool ccw = true);
    
    void AddFacesFromNodeSet(std::vector<std::shared_ptr<ChNodeFEAbase>>& nodes);
    
    // Material
    void SetMaterial(std::shared_ptr<ChContactMaterial> material);
    
    // Collision
    size_t GetNumTriangles() const;
    ChVector3 GetTriangleVertex(size_t i, int vertex) const;
};
```

## 依赖关系

### 外部依赖
- **Eigen3**: Linear algebra for element matrices
- **MKL (optional)**: Fast linear algebra

### 内部依赖
- **core**: ChVector3, ChMatrix for math
- **physics**: ChPhysicsItem, ChLoadable interfaces
- **solver**: ChVariables, ChConstraint for DOFs
- **collision**: ChContactMaterial for contact

### 其他模块的使用
- **vehicle**: Flexible tires, cables
- **sensor**: Deformable object visualization
- **postprocess**: FEA result export

## 典型使用模式

### 创建 Simple Beam
```cpp
// 创建 mesh
auto mesh = chrono_types::make_shared<ChMesh>();

// 创建 nodes
auto nodeA = chrono_types::make_shared<ChNodeFEAxyzrot>(
    ChFrame<>(ChVector3(0, 0, 0)));
auto nodeB = chrono_types::make_shared<ChNodeFEAxyzrot>(
    ChFrame<>(ChVector3(1, 0, 0)));

mesh->AddNode(nodeA);
mesh->AddNode(nodeB);

// 创建 beam element
auto element = chrono_types::make_shared<ChElementBeamEuler>();
element->SetNodes(nodeA, nodeB);

// Section properties
auto section = chrono_types::make_shared<ChBeamSectionEuler>();
section->SetYoungModulus(2.1e11);  // Steel
section->SetDensity(7800);
section->SetArea(0.01);
section->SetIyy(1e-6);
section->SetIzz(1e-6);
element->SetSection(section);

mesh->AddElement(element);

// Add to system
system.Add(mesh);

// Fix one end
nodeA->SetFixed(true);

// Apply load to other end
nodeB->SetForce(ChVector3(0, -1000, 0));
```

### 创建 Shell Mesh
```cpp
auto mesh = chrono_types::make_shared<ChMesh>();

// 创建 nodes in grid
for (int i = 0; i <= nx; i++) {
    for (int j = 0; j <= ny; j++) {
        auto node = chrono_types::make_shared<ChNodeFEAxyzD>(
            ChVector3(i * dx, j * dy, 0),
            ChVector3(0, 0, 1)  // Initial normal
        );
        mesh->AddNode(node);
    }
}

// 创建 shell elements
for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
        auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
        element->SetNodes(
            GetNode(i, j),
            GetNode(i+1, j),
            GetNode(i+1, j+1),
            GetNode(i, j+1)
        );
        
        auto material = chrono_types::make_shared<ChMaterialShellANCF>();
        material->SetYoungModulus(2.1e11);
        material->SetPoissonRatio(0.3);
        material->SetDensity(7800);
        element->AddLayer(0.01, 0, material);  // 1cm thickness
        
        mesh->AddElement(element);
    }
}
```

### Add Contact Surface
```cpp
// 创建 contact surface from mesh boundary
auto contact_surface = chrono_types::make_shared<ChContactSurfaceMesh>();
mesh->AddContactSurface(contact_surface);

// Material
auto contact_mat = chrono_types::make_shared<ChContactMaterialSMC>();
contact_mat->SetYoungModulus(1e7);
contact_mat->SetFriction(0.5f);
contact_surface->SetMaterial(contact_mat);

// Add faces from boundary
contact_surface->AddFacesFromBoundary(0.001);  // 1mm sphere swept
```

### Apply Distributed Load
```cpp
// Gravity load on beam
auto load = chrono_types::make_shared<ChLoadBeamWrench>(element);
load->SetForce(ChVector3(0, -9.81 * mass_per_length, 0));
auto load_container = chrono_types::make_shared<ChLoadContainer>();
load_container->Add(load);
system.Add(load_container);
```

### Load Mesh from File
```cpp
// Load from Abaqus file
ChMeshFileLoader::ANCFShellFromAbaqusInput(
    mesh,
    "mesh.inp",
    material,
    shell_thickness
);

// Or from gmsh
ChMeshFileLoader::FromGmshFile(
    mesh,
    "mesh.msh",
    material
);
```

## 关键设计决策

### 1. Modular Element Library
**决策**: Wide variety of element types
**理由**:
- Different elements for different applications
- ANCF for large deformations
- Corotational for moderate deformations
- Euler beams for small deformations
- User can choose appropriate formulation

### 2. Separate Nodes and Elements
**决策**: Nodes and elements are separate objects
**理由**:
- Flexibility in mesh topology
- Nodes shared between elements
- Clear ownership model
- Standard FEA architecture

### 3. Material/Section Separation
**决策**: Separate material and geometric properties
**理由**:
- Reusable materials
- Section properties independent
- Clear organization
- Standard in structural analysis

### 4. 集成 with Physics Engine
**决策**: FEA elements are ChPhysicsItems
**理由**:
- Seamless integration with rigid bodies
- Unified solver for everything
- Contacts between FEA and rigid
- Single time integration

## 性能特性

### 优势
1. **Sparse Matrices**: Efficient for large systems
2. **Parallel Assembly**: Elements assembled in parallel
3. **Selective 集成**: Reduced integration for efficiency
4. **Corotational**: Efficient for moderate rotations

### 注意事项
1. **Element Count**: Many elements = large system
2. **Nonlinearity**: Geometric/material nonlinearity expensive
3. **Contact**: FEA contact is computationally intensive
4. **Time Steps**: Small steps needed for stability

## 总结

The FEA 模块提供:
- Comprehensive finite element library
- Beams, shells, solids, and cables
- Small and large deformation formulations
- Material models (elastic, plastic, hyperelastic)
- FEA-rigid body interaction
- Contact on FEA surfaces

Its design emphasizes flexibility and integration with the broader Chrono physics engine, enabling complex multi-physics simulations combining flexible and rigid bodies.
