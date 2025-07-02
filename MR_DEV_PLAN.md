# Multi-Robot Development Plan - Hybrid Approach

## Overview

This document outlines the development plan for enabling multi-robot planning in VAMP using a **hybrid approach** that combines:
1. **Template-based SIMD optimization** for maximum performance
2. **Object-oriented interfaces** for multi-robot algorithm compatibility
3. **Factory pattern** for creating robot instances from templates

This approach preserves SIMD benefits while enabling algorithms like Conflict-Based Search (CBS) and Prioritized Planning.

## Design Philosophy

### Performance vs. Flexibility Trade-off
- **Template Approach**: Maximum SIMD performance, compile-time optimization
- **OO Approach**: Algorithm flexibility, runtime polymorphism
- **Hybrid Approach**: Best of both worlds - SIMD for computation, OO for algorithms

### Multi-Robot Algorithm Compatibility
- **Conflict-Based Search (CBS)**: Requires heterogeneous robot collections
- **Prioritized Planning**: Needs dynamic robot priority reordering
- **Centralized Planning**: Benefits from unified robot interface
- **Decentralized Planning**: Can still use individual template variants

## Architecture Design

### 1. Template Layer (SIMD Optimized)
```cpp
// Pre-compiled grid variants with full SIMD optimization
template<int BaseX, int BaseY, int BaseZ>  // Integer template params (cm * 100)
struct PandaBase : BaseRobot<BaseX, BaseY, BaseZ> {
    // All existing SIMD-optimized functionality
    template<std::size_t rake>
    static void sphere_fk(const ConfigurationBlock<rake>& q, Spheres<rake>& out);
    
    template<std::size_t rake>
    static bool validate_motion(const Configuration& start, const Configuration& goal, 
                               const Environment<FloatVector<rake>>& env);
};

// Grid variants (compile-time optimized)
using Panda_0_0 = PandaBase<0, 0, 5>;    // (0.0, 0.0, 0.05)
using Panda_0_1 = PandaBase<0, 100, 5>;  // (0.0, 1.0, 0.05)
using Panda_2_2 = PandaBase<200, 200, 5>; // (2.0, 2.0, 0.05)
// ... all 9 grid variants
```

### 2. OO Interface Layer (Algorithm Compatible)
```cpp
// Base interface for multi-robot algorithms
class RobotInterface {
public:
    virtual ~RobotInterface() = default;
    
    // Core robot functionality
    virtual std::vector<Sphere> fk(const Configuration& config) = 0;
    virtual bool validate(const Configuration& config, const Environment& env) = 0;
    virtual std::array<float, 3> get_base_position() const = 0;
    virtual std::string get_name() const = 0;
    
    // Multi-robot specific methods
    virtual int get_dimension() const = 0;
    virtual int get_n_spheres() const = 0;
    virtual float get_resolution() const = 0;
    
    // Planning methods
    virtual PlanningResult rrtc(const Configuration& start, const Configuration& goal,
                               const Environment& env, const RRTCSettings& settings,
                               std::shared_ptr<RNG> rng) = 0;
    // ... other planning methods
};

// Template wrapper that preserves SIMD
template<typename RobotType>
class RobotWrapper : public RobotInterface {
private:
    RobotType robot_; // The templated robot (SIMD optimized)
    
public:
    std::vector<Sphere> fk(const Configuration& config) override {
        return RobotType::fk(config); // Uses templated SIMD code
    }
    
    bool validate(const Configuration& config, const Environment& env) override {
        return RobotType::validate(config, env); // Uses templated SIMD code
    }
    
    std::array<float, 3> get_base_position() const override {
        return {RobotType::base_x / 100.0f, RobotType::base_y / 100.0f, RobotType::base_z / 100.0f};
    }
    
    std::string get_name() const override {
        return RobotType::name;
    }
    
    // ... other interface implementations
};
```

### 3. Factory Layer (Creation & Management)
```cpp
// Factory for creating robot instances
class RobotFactory {
public:
    // Create robot from position (maps to nearest grid variant)
    static std::unique_ptr<RobotInterface> create_panda(float x, float y, float z) {
        // Round to nearest grid position
        int grid_x = std::round(x * 100);
        int grid_y = std::round(y * 100);
        int grid_z = std::round(z * 100);
        
        // Map to template variant
        if (grid_x == 0 && grid_y == 0 && grid_z == 5) 
            return std::make_unique<RobotWrapper<Panda_0_0>>();
        if (grid_x == 200 && grid_y == 200 && grid_z == 5) 
            return std::make_unique<RobotWrapper<Panda_2_2>>();
        // ... etc for all grid variants
        
        throw std::runtime_error("Position not supported in grid");
    }
    
    // Create robot from grid variant name
    static std::unique_ptr<RobotInterface> create_panda_grid(const std::string& variant) {
        if (variant == "panda_0_0") return std::make_unique<RobotWrapper<Panda_0_0>>();
        if (variant == "panda_2_2") return std::make_unique<RobotWrapper<Panda_2_2>>();
        // ... etc
        
        throw std::runtime_error("Unknown grid variant: " + variant);
    }
    
    // Get all available grid variants
    static std::vector<std::string> get_available_variants() {
        return {"panda_0_0", "panda_0_1", "panda_0_2", 
                "panda_1_0", "panda_1_1", "panda_1_2",
                "panda_2_0", "panda_2_1", "panda_2_2"};
    }
};
```

## Implementation Plan

### Phase 1: Core Infrastructure (Week 1)

#### Step 1.1: Create Base Robot Interface
**File**: `src/impl/vamp/robots/base_robot_interface.hh`
```cpp
class RobotInterface {
    // Define pure virtual interface for multi-robot algorithms
    // Core methods: fk, validate, get_base_position, get_name
    // Planning methods: rrtc, prm, fcit, aorrtc
};
```

#### Step 1.2: Create Template Wrapper
**File**: `src/impl/vamp/robots/robot_wrapper.hh`
```cpp
template<typename RobotType>
class RobotWrapper : public RobotInterface {
    // Implement all interface methods by delegating to RobotType
    // Preserve SIMD optimizations from template layer
};
```

#### Step 1.3: Create Robot Factory
**File**: `src/impl/vamp/robots/robot_factory.hh`
```cpp
class RobotFactory {
    // Factory methods for creating robot instances
    // Position mapping to grid variants
    // Grid variant name mapping
};
```

### Phase 2: Multi-Robot Algorithm Infrastructure (Week 2)

#### Step 2.1: Multi-Robot Environment
**File**: `src/impl/vamp/collision/multi_robot_environment.hh`
```cpp
class MultiRobotEnvironment {
private:
    std::vector<std::unique_ptr<RobotInterface>> robots_;
    Environment base_environment_;
    
public:
    // Add/remove robots
    void add_robot(std::unique_ptr<RobotInterface> robot);
    void remove_robot(size_t index);
    
    // Inter-robot collision checking
    bool check_inter_robot_collisions(const std::vector<Configuration>& configs);
    
    // Environment management
    void add_obstacle(const Sphere& sphere);
    void add_cuboid(const Cuboid& cuboid);
};
```

#### Step 2.2: Multi-Robot Planning Base Classes
**File**: `src/impl/vamp/planning/multi_robot_planner.hh`
```cpp
class MultiRobotPlanner {
protected:
    std::vector<std::unique_ptr<RobotInterface>> robots_;
    MultiRobotEnvironment environment_;
    
public:
    virtual PlanningResult plan(const std::vector<Configuration>& starts,
                               const std::vector<Configuration>& goals) = 0;
};

class PrioritizedPlanner : public MultiRobotPlanner {
    // Implement prioritized planning
    // Reorder robot priorities dynamically
};

class CBSPlanner : public MultiRobotPlanner {
    // Implement Conflict-Based Search
    // Handle constraints and conflicts
};
```

### Phase 3: Python Bindings (Week 3)

#### Step 3.1: Template API (Performance)
**File**: `src/impl/vamp/bindings/panda.cc`
```cpp
void vamp::binding::init_panda(nanobind::module_ &pymodule) {
    // Register main panda module (backward compatibility)
    vamp::binding::init_robot<vamp::robots::Panda>(pymodule);
    
    // Create grid variant submodules that alias main types
    auto create_grid_variant = [&pymodule](const char* name, const char* description) {
        auto submodule = pymodule.def_submodule(name, description);
        // Alias all classes and functions from main module
        // This avoids type registration conflicts
    };
    
    // Create all 9 grid variants
    create_grid_variant("panda_0_0", "Panda robot at position (0.0, 0.0, 0.05)");
    // ... etc
}
```

#### Step 3.2: OO API (Multi-Robot Algorithms)
**File**: `src/impl/vamp/bindings/multi_robot.cc`
```cpp
void vamp::binding::init_multi_robot(nanobind::module_ &pymodule) {
    // RobotFactory bindings
    nb::class_<RobotFactory>(pymodule, "RobotFactory")
        .def_static("create_panda", &RobotFactory::create_panda)
        .def_static("create_panda_grid", &RobotFactory::create_panda_grid)
        .def_static("get_available_variants", &RobotFactory::get_available_variants);
    
    // RobotInterface bindings
    nb::class_<RobotInterface>(pymodule, "Robot")
        .def("fk", &RobotInterface::fk)
        .def("validate", &RobotInterface::validate)
        .def("get_base_position", &RobotInterface::get_base_position)
        .def("get_name", &RobotInterface::get_name)
        .def("rrtc", &RobotInterface::rrtc);
    
    // MultiRobotEnvironment bindings
    nb::class_<MultiRobotEnvironment>(pymodule, "MultiRobotEnvironment")
        .def(nb::init<>())
        .def("add_robot", &MultiRobotEnvironment::add_robot)
        .def("remove_robot", &MultiRobotEnvironment::remove_robot)
        .def("check_inter_robot_collisions", &MultiRobotEnvironment::check_inter_robot_collisions);
    
    // Planner bindings
    nb::class_<PrioritizedPlanner>(pymodule, "PrioritizedPlanner")
        .def(nb::init<std::vector<std::unique_ptr<RobotInterface>>>())
        .def("plan", &PrioritizedPlanner::plan);
    
    nb::class_<CBSPlanner>(pymodule, "CBSPlanner")
        .def(nb::init<std::vector<std::unique_ptr<RobotInterface>>>())
        .def("plan", &CBSPlanner::plan);
}
```

### Phase 4: Testing and Validation (Week 4)

#### Step 4.1: Unit Tests
**File**: `test_multi_robot.py`
```python
def test_robot_factory():
    # Test creating robots from positions
    robot1 = vamp.RobotFactory.create_panda(0.0, 0.0, 0.05)
    robot2 = vamp.RobotFactory.create_panda(2.0, 2.0, 0.05)
    
    assert robot1.get_base_position() == [0.0, 0.0, 0.05]
    assert robot2.get_base_position() == [2.0, 2.0, 0.05]

def test_multi_robot_planning():
    # Test CBS planning
    robots = [
        vamp.RobotFactory.create_panda(0.0, 0.0, 0.05),
        vamp.RobotFactory.create_panda(2.0, 2.0, 0.05)
    ]
    
    planner = vamp.CBSPlanner(robots)
    result = planner.plan(start_configs, goal_configs, environment)
    assert result.solved
```

#### Step 4.2: Performance Tests
**File**: `test_performance.py`
```python
def test_simd_performance():
    # Compare template vs OO performance
    # Template approach should be faster for single robot
    # OO approach should be comparable for multi-robot
```

## Final API Design

### Python Usage Examples

#### 1. Template API (Performance)
```python
import vamp._core as vamp

# Direct template access (maximum SIMD performance)
config = vamp.panda_2_2.Configuration([0, 0, 0, 0, 0, 0, 0])
spheres = vamp.panda_2_2.fk(config)  # Full SIMD optimization

# Multi-robot with templates (homogeneous)
robots = [vamp.panda_2_2, vamp.panda_2_2, vamp.panda_2_2]  # Same type = SIMD batch
```

#### 2. OO API (Multi-Robot Algorithms)
```python
from vamp.multi_robot import RobotFactory, CBSPlanner, PrioritizedPlanner

# Create heterogeneous robot collection
robots = [
    RobotFactory.create_panda(0.0, 0.0, 0.05),  # Maps to Panda_0_0
    RobotFactory.create_panda(2.0, 2.0, 0.05),  # Maps to Panda_2_2
    RobotFactory.create_panda(1.0, 1.0, 0.05)   # Maps to Panda_1_1
]

# Use multi-robot algorithms
cbs_planner = CBSPlanner(robots)
result = cbs_planner.plan(start_configs, goal_configs, environment)

# Prioritized planning
priority_planner = PrioritizedPlanner(robots)
result = priority_planner.plan(start_configs, goal_configs, environment)
```

#### 3. Mixed Usage
```python
# Use templates for performance-critical single-robot work
single_robot = vamp.panda_2_2
fast_result = single_robot.rrtc(start, goal, env, settings, rng)

# Use OO for multi-robot algorithms
multi_robots = [RobotFactory.create_panda(0, 0, 0.05),
                RobotFactory.create_panda(2, 2, 0.05)]
cbs_result = CBSPlanner(multi_robots).plan(starts, goals, env)
```

## Performance Characteristics

| Scenario | Template API | OO API | Performance |
|----------|-------------|--------|-------------|
| Single robot | ✅ | ✅ | Template: Excellent, OO: Good |
| Multi-robot, homogeneous | ✅ | ✅ | Template: Excellent (SIMD batch), OO: Good |
| Multi-robot, heterogeneous | ❌ | ✅ | OO: Good (scalar + internal SIMD) |
| CBS algorithm | ❌ | ✅ | OO: Good |
| Prioritized planning | ❌ | ✅ | OO: Good |

## Benefits of Hybrid Approach

1. **Maximum SIMD Performance**: Template layer preserves all vectorization
2. **Algorithm Compatibility**: OO layer enables CBS, prioritized planning, etc.
3. **Flexible API**: Users choose based on their needs
4. **Backward Compatibility**: Existing `vamp.panda` still works
5. **Future-Proof**: Easy to add new multi-robot algorithms
6. **Minimal Overhead**: OO wrapper adds only one virtual function call

## Implementation Timeline

- **Week 1**: Core infrastructure (interfaces, wrappers, factory)
- **Week 2**: Multi-robot algorithm infrastructure
- **Week 3**: Python bindings for both APIs
- **Week 4**: Testing, validation, and documentation

## Future Extensions

### Phase 5: Extend to Other Robots
- Apply same pattern to UR5, Fetch, Baxter
- Each robot gets template variants + OO interface

### Phase 6: Advanced Multi-Robot Algorithms
- Decentralized planning
- Formation control
- Swarm robotics

### Phase 7: Dynamic Robot Creation
- Runtime robot parameter modification
- Custom robot configurations
- Learning-based robot adaptation 