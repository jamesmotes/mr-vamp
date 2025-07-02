#!/usr/bin/env python3
"""
Simple test script to demonstrate multi-robot animation functionality.
This script creates multiple robots at different base positions and animates them
all following the same path to show the multi-robot system working.
"""

import vamp
from vamp import pybullet_interface as vpb
from pathlib import Path
from fire import Fire

def main(visualize: bool = True):
    """Demonstrate multi-robot animation with visualization."""
    
    print("Multi-robot animation demonstration")
    print("=" * 40)
    
    # Simple start and goal configurations
    start_config = [0., -0.785, 0., -2.356, 0., 1.571, 0.785]
    goal_config = [2.35, 1., 0., -0.8, 0, 2.5, 0.785]
    
    # Configure planner
    (vamp_module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs("panda", "rrtc")
    
    sampler = vamp_module.halton()
    
    if visualize:
        # Create simulator
        robot_dir = Path(__file__).parent / 'resources' / 'panda'
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
            (-2.0, 0.0, 0.05),  # Robot 4
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
        
        # Create a simple environment (empty for this demo)
        e = vamp.Environment()
        
        # Plan a path for the main robot
        print("\nPlanning path for main robot...")
        result = planner_func(start_config, goal_config, e, plan_settings, sampler)
        
        if result.solved:
            simple = vamp_module.simplify(result.path, e, simp_settings, sampler)
            simple.path.interpolate_to_resolution(vamp.panda.resolution())
            print(f"✓ Path planned successfully ({len(simple.path)} states)")
            
            # Use the same path for all robots
            print("\nStarting multi-robot animation...")
            print("Controls:")
            print("  Space: Start/stop global animation")
            print("  Tab: Select next robot for individual control")
            print("  0-9: Select specific robot by number")
            print("  Left/Right arrows: Step through states (when paused)")
            print("  R: Reset all robots")
            print("  Q: Quit animation")
            
            # Animate all robots with the same path
            sim.animate_multi(robot_ids, plan=simple.path)
            
        else:
            print("✗ Failed to plan path, creating simple fallback path")
            
            # Create a simple fallback path
            fallback_path = vamp_module.Path()
            fallback_path.append(vamp_module.Configuration(start_config))
            fallback_path.append(vamp_module.Configuration(goal_config))
            
            print("\nStarting multi-robot animation with fallback path...")
            sim.animate_multi(robot_ids, plan=fallback_path)
    
    else:
        print("Visualization disabled. Set --visualize=True to see the animation.")

if __name__ == "__main__":
    Fire(main) 