#!/usr/bin/env python3
"""
Simple test script to verify the animation system works.
"""

import vamp
from vamp import pybullet_interface as vpb
from pathlib import Path

def main():
    """Test basic animation functionality."""
    
    print("Testing basic animation functionality...")
    
    # Create simulator
    robot_dir = Path(__file__).parent.parent / 'resources' / 'panda'
    sim = vpb.PyBulletSimulator(
        str(robot_dir / "panda_spherized.urdf"), 
        vamp.ROBOT_JOINTS['panda'], 
        visualize=True
    )
    
    # Set up first robot at origin
    first_robot_id = sim.skel_id
    sim.set_robot_base_position((0.0, 0.0, 0.05))
    print(f"Main robot at position (0.0, 0.0, 0.05)")
    
    # Load additional robots at different positions
    robot_ids = [first_robot_id]
    base_positions = [
        (0.0, 0.0, 0.05),   # Main robot
        (2.0, 0.0, 0.05),   # Robot 2
        (0.0, 2.0, 0.05),   # Robot 3
    ]
    
    for i, (x, y, z) in enumerate(base_positions[1:], 2):
        new_robot_id = sim.client.loadURDF(
            str(robot_dir / "panda_spherized.urdf"),
            basePosition=(x, y, z),
            baseOrientation=(0, 0, 0, 1),
            useFixedBase=True,
            flags=sim.client.URDF_MAINTAIN_LINK_ORDER | sim.client.URDF_USE_SELF_COLLISION
        )
        robot_ids.append(new_robot_id)
        sim.add_robot(new_robot_id, sim.joints, f"Robot_{i}")
        print(f"Robot {i} at position ({x}, {y}, {z})")
    
    # Create a simple test path
    from vamp.panda import Path as PandaPath, Configuration as PandaConfiguration
    
    test_path = PandaPath()
    test_path.append(PandaConfiguration([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]))
    test_path.append(PandaConfiguration([0.0, 0.785, 0.0, -0.785, 0.0, 1.571, -0.785]))
    
    print(f"\nCreated test path with {len(test_path)} waypoints")
    print(f"Start: {test_path[0].to_list()}")
    print(f"End: {test_path[len(test_path)-1].to_list()}")
    
    print("\nStarting multi-robot animation...")
    print("Controls:")
    print("  Space: Start/stop global animation")
    print("  Tab: Select next robot for individual control")
    print("  0-9: Select specific robot by number")
    print("  Left/Right arrows: Step through states (when paused)")
    print("  R: Reset all robots")
    print("  Q: Quit animation")
    
    # Test 1: Single robot animation
    print("\nTest 1: Single robot animation")
    sim.animate(test_path)
    
    # Test 2: Multi-robot animation with same plan
    print("\nTest 2: Multi-robot animation with same plan")
    sim.animate_multi(robot_ids, plan=test_path)
    
    # Test 3: Multi-robot animation with different plans
    print("\nTest 3: Multi-robot animation with different plans")
    robot_plans = {}
    for i, robot_id in enumerate(robot_ids):
        robot_plans[robot_id] = test_path
    
    sim.animate_multi(robot_plans)

if __name__ == "__main__":
    main() 