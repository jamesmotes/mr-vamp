#!/usr/bin/env python3

import vamp._core as vamp
import numpy as np

def test_panda_2_2():
    """Test the panda_2_2 variant to ensure it's positioned correctly."""
    
    print("Testing panda_2_2 variant...")
    
    # Access the panda_2_2 module
    panda_2_2 = vamp.panda_2_2
    
    print(f"Robot dimension: {panda_2_2.dimension}")
    print(f"Resolution: {panda_2_2.resolution}")
    print(f"Number of spheres: {panda_2_2.n_spheres}")
    
    # Create a default configuration (all zeros)
    config = panda_2_2.Configuration()
    print(f"Default config: {config.to_list()}")
    
    # Test forward kinematics to see sphere positions
    spheres = panda_2_2.fk(config)
    print(f"Number of spheres from FK: {len(spheres)}")
    
    # Check the first few sphere positions to verify base offset
    print("\nFirst 5 sphere positions (should be offset by (2.0, 2.0, 0.05)):")
    for i, sphere in enumerate(spheres[:5]):
        print(f"Sphere {i}: pos=({sphere.x:.3f}, {sphere.y:.3f}, {sphere.z:.3f}), radius={sphere.r:.3f}")
    
    # Test end-effector forward kinematics
    eef_pos, eef_quat = panda_2_2.eefk(config)
    print(f"\nEnd-effector position: {eef_pos}")
    print(f"End-effector orientation (xyzw): {eef_quat}")
    
    # Test validation
    is_valid = panda_2_2.validate(config)
    print(f"Default configuration valid: {is_valid}")
    
    print("\nPanda_2_2 test completed successfully!")

if __name__ == "__main__":
    test_panda_2_2() 