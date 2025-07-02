#!/usr/bin/env python3
"""
Simple test for basic animation functionality.
"""

import vamp
from vamp import pybullet_interface as vpb
from pathlib import Path

def main():
    """Test basic animation functionality."""
    
    print("Testing basic animation...")
    
    # Create simulator
    robot_dir = Path(__file__).parent.parent / 'resources' / 'panda'
    sim = vpb.PyBulletSimulator(
        str(robot_dir / "panda_spherized.urdf"), 
        vamp.ROBOT_JOINTS['panda'], 
        visualize=True
    )
    
    # Create a simple test path
    from vamp.panda import Path as PandaPath, Configuration as PandaConfiguration
    
    test_path = PandaPath()
    test_path.append(PandaConfiguration([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]))
    test_path.append(PandaConfiguration([0.0, 0.785, 0.0, -0.785, 0.0, 1.571, -0.785]))
    
    print(f"Created test path with {len(test_path)} waypoints")
    print(f"Start: {test_path[0].to_list()}")
    print(f"End: {test_path[len(test_path)-1].to_list()}")
    
    print("\nStarting animation...")
    print("Press space to start/stop animation")
    print("Use left/right arrow keys to move through individual states")
    print("Press Q to quit")
    
    # Test basic animation
    sim.animate(test_path)

if __name__ == "__main__":
    main() 