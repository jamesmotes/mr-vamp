# Templated FK Functions with Base Position Parameters

## Overview

This document outlines a solution to modify the FK functions in `src/impl/vamp/robots/panda/fk.hh` to be templated on base position parameters, allowing the compiler to optimize the base position additions at compile time while maintaining SIMD efficiency.

## Problem Statement

Currently, the base position is hardcoded in the FK calculations:

```cpp
// Current hardcoded positions in fk.hh
out.x[0] = 0.0;     // (0, 0)
out.y[0] = 0.0;     // (0, 0)
out.z[0] = 0.05;    // (0, 0)
```

The base position is accessible via template parameters (`Robot::base_x`, `Robot::base_y`, `Robot::base_z`) but is not being applied to the sphere positions, leading to incorrect world coordinates.

## Solution: Template FK Functions on Base Position

### 1. Modified FK Function Signature

```cpp
// Original function
template <std::size_t rake>
inline void sphere_fk(const ConfigurationBlock<rake> &q, Spheres<rake> &out) noexcept

// New templated function with base position parameters
template <std::size_t rake, float base_x, float base_y, float base_z>
inline void sphere_fk(const ConfigurationBlock<rake> &q, Spheres<rake> &out) noexcept
```

### 2. Compile-Time Base Position Addition

```cpp
// Instead of hardcoded positions:
out.x[0] = 0.0;     // (0, 0)
out.y[0] = 0.0;     // (0, 0)
out.z[0] = 0.05;    // (0, 0)

// Use compile-time constant addition:
out.x[0] = 0.0f + base_x;     // (0, 0)
out.y[0] = 0.0f + base_y;     // (0, 0)
out.z[0] = 0.05f + base_z;    // (0, 0)
```

### 3. Backward Compatibility

```cpp
// Original function calls the templated version with default base position
template <std::size_t rake>
inline void sphere_fk(const ConfigurationBlock<rake> &q, Spheres<rake> &out) noexcept
{
    sphere_fk<rake, 0.0f, 0.0f, 0.0f>(q, out);
}
```

## Implementation Details

### 1. Template Parameter Propagation

```cpp
// Base robot template (already exists)
template<int BaseX100, int BaseY100, int BaseZ100>
struct BaseRobot {
    static constexpr float base_x = static_cast<float>(BaseX100) / 100.0f;
    static constexpr float base_y = static_cast<float>(BaseY100) / 100.0f;
    static constexpr float base_z = static_cast<float>(BaseZ100) / 100.0f;
};

// Robot instantiation (already exists)
struct Panda_2_2 : PandaBase<200, 200, 5> {
    static constexpr auto name = "panda_2_2";
};

// Modified panda base to use templated FK
template<int BaseX100, int BaseY100, int BaseZ100>
struct PandaBase : BaseRobot<BaseX100, BaseY100, BaseZ100>
{
    // Use templated FK functions with base position parameters
    template <std::size_t rake>
    static constexpr auto sphere_fk = panda::sphere_fk<rake, base_x, base_y, base_z>;
    
    template <std::size_t rake>
    static constexpr auto fkcc = panda::interleaved_sphere_fk<rake, base_x, base_y, base_z>;
};
```

### 2. Compiler Optimization Benefits

#### Compile-Time Constant Folding
When `base_x = 2.0f`, `base_y = 2.0f`, `base_z = 0.05f`:
- `out.x[0] = 0.0f + 2.0f` becomes `out.x[0] = 2.0f`
- `out.y[0] = 0.0f + 2.0f` becomes `out.y[0] = 2.0f`
- `out.z[0] = 0.05f + 0.05f` becomes `out.z[0] = 0.1f`

#### SIMD Vectorization
- No runtime addition operations for base position
- Compiler can vectorize FK calculations without branching
- Maintains full SIMD efficiency

#### Memory Access Patterns
- All sphere positions computed in one pass
- No separate loop to add base positions
- Better cache locality

## File Modifications Required

### 1. `src/impl/vamp/robots/panda/fk.hh`

```cpp
// Add templated version of sphere_fk
template <std::size_t rake, float base_x, float base_y, float base_z>
inline void sphere_fk(const ConfigurationBlock<rake> &q, Spheres<rake> &out) noexcept
{
    // Set sphere radii (unchanged)
    out.r[0] = 0.08;
    // ... (all radius assignments)
    
    // Set base positions with compile-time constant addition
    out.x[0] = 0.0f + base_x;
    out.y[0] = 0.0f + base_y;
    out.z[0] = 0.05f + base_z;
    
    // Continue with FK calculations, adding base position to each coordinate
    auto INPUT_0 = q[0];
    // ... (FK calculations)
    out.x[1] = MUL_1615 + base_x;  // Add base_x to computed position
    out.x[2] = MUL_1639 + base_x;  // Add base_x to computed position
    // ... (continue for all sphere positions)
}

// Add templated version of interleaved_sphere_fk
template <std::size_t rake, float base_x, float base_y, float base_z>
inline bool interleaved_sphere_fk(
    const vamp::collision::Environment<FloatVector<rake>> &environment,
    const ConfigurationBlock<rake> &q) noexcept
{
    // Similar modifications for collision checking
    // Add base position to all sphere_environment_in_collision calls
}

// Keep original functions for backward compatibility
template <std::size_t rake>
inline void sphere_fk(const ConfigurationBlock<rake> &q, Spheres<rake> &out) noexcept
{
    sphere_fk<rake, 0.0f, 0.0f, 0.0f>(q, out);
}
```

### 2. `src/impl/vamp/robots/panda_base.hh`

```cpp
template<int BaseX100, int BaseY100, int BaseZ100>
struct PandaBase : BaseRobot<BaseX100, BaseY100, BaseZ100>
{
    // Use templated FK functions with base position parameters
    template <std::size_t rake>
    static constexpr auto sphere_fk = panda::sphere_fk<rake, base_x, base_y, base_z>;
    
    template <std::size_t rake>
    static constexpr auto fkcc = panda::interleaved_sphere_fk<rake, base_x, base_y, base_z>;
    
    template <std::size_t rake>
    static constexpr auto fkcc_attach = panda::interleaved_sphere_fk_attachment<rake, base_x, base_y, base_z>;
};
```

### 3. `src/impl/vamp/bindings/common.hh`

```cpp
template <typename Robot>
struct Helper
{
    inline static auto fk(const ConfigurationArray &configuration)
        -> std::vector<vamp::collision::Sphere<float>>
    {
        typename Robot::template Spheres<1> out;
        typename Robot::template ConfigurationBlock<1> block;
        
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            block[i] = configuration[i];
        }

        // Use templated sphere_fk (base position handled at compile time)
        Robot::template sphere_fk<1>(block, out);
        
        std::vector<vamp::collision::Sphere<float>> result;
        result.reserve(Robot::n_spheres);

        for (auto i = 0U; i < Robot::n_spheres; ++i)
        {
            // No need to add base position here - it's already included
            result.emplace_back(out.x[{i, 0}], out.y[{i, 0}], out.z[{i, 0}], out.r[{i, 0}]);
        }

        return result;
    }
};
```

## Benefits

### 1. Performance
- **Zero runtime overhead** for base position addition
- **Compile-time constant folding** eliminates addition operations
- **Maintained SIMD efficiency** through vectorization
- **Better cache locality** with single-pass computation

### 2. Correctness
- **Proper world coordinates** for all sphere positions
- **Type safety** through template parameters
- **No runtime errors** from missing base position addition

### 3. Maintainability
- **Single source of truth** for base position
- **Compile-time validation** of base position usage
- **Clear separation** between local and world coordinates

## Example Usage

```cpp
// Robot variants automatically use correct base positions
Panda_0_0 robot_00;  // base_x=0.0, base_y=0.0, base_z=0.05
Panda_2_2 robot_22;  // base_x=2.0, base_y=2.0, base_z=0.05

// FK calls automatically include base position
auto spheres_00 = robot_00.fk(config);  // Spheres at (0,0,0.05) + local positions
auto spheres_22 = robot_22.fk(config);  // Spheres at (2,2,0.05) + local positions
```

## Migration Strategy

1. **Phase 1**: Add templated FK functions alongside existing ones
2. **Phase 2**: Update robot base classes to use templated versions
3. **Phase 3**: Update helper templates to remove runtime base position addition
4. **Phase 4**: Remove original non-templated functions (after testing)

This approach ensures backward compatibility while providing the performance and correctness benefits of compile-time base position optimization. 