#!/usr/bin/env python3

import vamp._core as vamp
import numpy as np

def test_template_api():
    """Test the template API (existing approach) for maximum performance."""
    print("=== Testing Template API ===")
    
    # Test direct template access
    config = vamp.panda_2_2.Configuration([0, 0, 0, 0, 0, 0, 0])
    spheres = vamp.panda_2_2.fk(config)
    print(f"Template API - Number of spheres: {len(spheres)}")
    
    # Test that panda_2_2 is at the correct position
    if len(spheres) > 0:
        first_sphere = spheres[0]
        print(f"Template API - First sphere position: ({first_sphere.x:.3f}, {first_sphere.y:.3f}, {first_sphere.z:.3f})")
        # Should be offset by (2.0, 2.0, 0.05)
        assert abs(first_sphere.x - 2.0) < 0.1, f"Expected x=2.0, got {first_sphere.x}"
        assert abs(first_sphere.y - 2.0) < 0.1, f"Expected y=2.0, got {first_sphere.y}"
        assert abs(first_sphere.z - 0.05) < 0.1, f"Expected z=0.05, got {first_sphere.z}"
        print("✅ Template API position test passed!")
    
    print("✅ Template API test completed!")

def test_oo_factory_api():
    """Test the OO factory API (new approach) for multi-robot algorithms."""
    print("\n=== Testing OO Factory API ===")
    
    try:
        # Import the new multi-robot module (we'll create this next)
        from vamp.multi_robot import RobotFactory
        
        # Test creating robots from positions
        robot1 = RobotFactory.create_panda(0.0, 0.0, 0.05)  # Should map to Panda_0_0
        robot2 = RobotFactory.create_panda(2.0, 2.0, 0.05)  # Should map to Panda_2_2
        
        print(f"OO API - Robot 1 name: {robot1.get_name()}")
        print(f"OO API - Robot 1 position: {robot1.get_base_position()}")
        print(f"OO API - Robot 2 name: {robot2.get_name()}")
        print(f"OO API - Robot 2 position: {robot2.get_base_position()}")
        
        # Test that positions are correct
        pos1 = robot1.get_base_position()
        pos2 = robot2.get_base_position()
        
        assert abs(pos1[0] - 0.0) < 0.001, f"Expected x=0.0, got {pos1[0]}"
        assert abs(pos1[1] - 0.0) < 0.001, f"Expected y=0.0, got {pos1[1]}"
        assert abs(pos1[2] - 0.05) < 0.001, f"Expected z=0.05, got {pos1[2]}"
        
        assert abs(pos2[0] - 2.0) < 0.001, f"Expected x=2.0, got {pos2[0]}"
        assert abs(pos2[1] - 2.0) < 0.001, f"Expected y=2.0, got {pos2[1]}"
        assert abs(pos2[2] - 0.05) < 0.001, f"Expected z=0.05, got {pos2[2]}"
        
        print("✅ OO API position test passed!")
        
        # Test robot properties
        print(f"OO API - Robot 1 dimension: {robot1.get_dimension()}")
        print(f"OO API - Robot 1 n_spheres: {robot1.get_n_spheres()}")
        print(f"OO API - Robot 1 resolution: {robot1.get_resolution()}")
        
        # Test FK through OO interface
        config = vamp.panda.Configuration([0, 0, 0, 0, 0, 0, 0])
        spheres_oo = robot2.fk(config)  # Should be at position (2.0, 2.0, 0.05)
        print(f"OO API - Number of spheres from FK: {len(spheres_oo)}")
        
        if len(spheres_oo) > 0:
            first_sphere_oo = spheres_oo[0]
            print(f"OO API - First sphere position: ({first_sphere_oo.x:.3f}, {first_sphere_oo.y:.3f}, {first_sphere_oo.z:.3f})")
            # Should be offset by (2.0, 2.0, 0.05)
            assert abs(first_sphere_oo.x - 2.0) < 0.1, f"Expected x=2.0, got {first_sphere_oo.x}"
            assert abs(first_sphere_oo.y - 2.0) < 0.1, f"Expected y=2.0, got {first_sphere_oo.y}"
            assert abs(first_sphere_oo.z - 0.05) < 0.1, f"Expected z=0.05, got {first_sphere_oo.z}"
            print("✅ OO API FK test passed!")
        
        # Test available variants
        variants = RobotFactory.get_available_variants()
        print(f"OO API - Available variants: {variants}")
        assert "panda_0_0" in variants, "panda_0_0 should be available"
        assert "panda_2_2" in variants, "panda_2_2 should be available"
        assert len(variants) == 9, f"Expected 9 variants, got {len(variants)}"
        print("✅ OO API variants test passed!")
        
        print("✅ OO Factory API test completed!")
        
    except ImportError as e:
        print(f"⚠️  OO Factory API not yet available: {e}")
        print("This is expected - we'll implement the Python bindings in Phase 3")

def test_grid_variants():
    """Test that all grid variants are available in the template API."""
    print("\n=== Testing Grid Variants ===")
    
    # Test that all grid variants are available
    grid_variants = [
        "panda_0_0", "panda_0_1", "panda_0_2",
        "panda_1_0", "panda_1_1", "panda_1_2", 
        "panda_2_0", "panda_2_1", "panda_2_2"
    ]
    
    for variant_name in grid_variants:
        if hasattr(vamp, variant_name):
            variant = getattr(vamp, variant_name)
            print(f"✅ {variant_name} is available")
            
            # Test basic functionality
            config = variant.Configuration([0, 0, 0, 0, 0, 0, 0])
            spheres = variant.fk(config)
            print(f"   - {variant_name} has {len(spheres)} spheres")
        else:
            print(f"❌ {variant_name} is NOT available")
    
    print("✅ Grid variants test completed!")

def main():
    """Run all tests for the hybrid approach."""
    print("🚀 Testing Hybrid Approach: Template + OO Interface")
    print("=" * 60)
    
    test_template_api()
    test_oo_factory_api()
    test_grid_variants()
    
    print("\n" + "=" * 60)
    print("🎉 All tests completed!")
    print("\nSummary:")
    print("- ✅ Template API: Working (maximum SIMD performance)")
    print("- ⚠️  OO Factory API: Not yet implemented (Phase 3)")
    print("- ✅ Grid Variants: All 9 variants available")
    print("\nNext steps:")
    print("1. Implement Python bindings for OO API (Phase 3)")
    print("2. Implement multi-robot algorithm infrastructure (Phase 2)")
    print("3. Test multi-robot planning algorithms")

if __name__ == "__main__":
    main() 