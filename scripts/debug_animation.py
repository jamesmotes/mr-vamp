#!/usr/bin/env python3
"""
Comprehensive debug script to isolate the animation issue.
"""

import vamp
from vamp import pybullet_interface as vpb
from pathlib import Path
import time
import sys

def test_pybullet_connection():
    """Test basic PyBullet connection."""
    print("=== Testing PyBullet Connection ===")
    
    try:
        import pybullet as pb
        from pybullet_utils.bullet_client import BulletClient
        
        print("✓ PyBullet imports successful")
        
        # Test basic connection
        client = BulletClient(connection_mode=pb.GUI)
        print("✓ PyBullet GUI connection successful")
        
        # Test basic functionality
        client.setRealTimeSimulation(0)
        print("✓ Real-time simulation disabled")
        
        # Test keyboard events
        keys = client.getKeyboardEvents()
        print(f"✓ Keyboard events accessible: {len(keys)} keys")
        
        # Test constants
        left_arrow = client.B3G_LEFT_ARROW
        right_arrow = client.B3G_RIGHT_ARROW
        print(f"✓ Arrow key constants: left={left_arrow}, right={right_arrow}")
        
        client.disconnect()
        print("✓ PyBullet connection test completed")
        return True
        
    except Exception as e:
        print(f"✗ PyBullet connection test failed: {e}")
        return False

def test_simulator_creation():
    """Test simulator creation."""
    print("\n=== Testing Simulator Creation ===")
    
    try:
        robot_dir = Path(__file__).parent.parent / 'resources' / 'panda'
        sim = vpb.PyBulletSimulator(
            str(robot_dir / "panda_spherized.urdf"), 
            vamp.ROBOT_JOINTS['panda'], 
            visualize=True
        )
        
        print("✓ Simulator creation successful")
        print(f"✓ Robot ID: {sim.skel_id}")
        print(f"✓ Joints: {len(sim.joints)} joints")
        print(f"✓ Joint names: {sim.joints}")
        
        return sim
        
    except Exception as e:
        print(f"✗ Simulator creation failed: {e}")
        return None

def test_path_creation():
    """Test path creation."""
    print("\n=== Testing Path Creation ===")
    
    try:
        # Use the correct imports from vamp.panda
        Path = vamp.panda.Path
        Configuration = vamp.panda.Configuration
        
        test_path = Path()
        test_path.append(Configuration([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]))
        test_path.append(Configuration([0.0, 0.785, 0.0, -0.785, 0.0, 1.571, -0.785]))
        
        print(f"✓ Path creation successful: {len(test_path)} waypoints")
        print(f"✓ Start: {test_path[0].to_list()}")
        print(f"✓ End: {test_path[len(test_path)-1].to_list()}")
        
        return test_path
        
    except Exception as e:
        print(f"✗ Path creation failed: {e}")
        import traceback
        traceback.print_exc()
        return None

def test_animation_loop(sim, test_path):
    """Test the animation loop with detailed debugging."""
    print("\n=== Testing Animation Loop ===")
    
    try:
        print("Starting animation loop...")
        print("Press space to start/stop animation")
        print("Use left/right arrow keys to move through individual states")
        print("Press Ctrl+C to exit")
        
        plan_idx = 0
        playing = False
        moved = True
        left = sim.client.B3G_LEFT_ARROW
        right = sim.client.B3G_RIGHT_ARROW
        space_code = ord(' ')
        
        iteration = 0
        max_iterations = 1000  # Prevent infinite loop
        
        while iteration < max_iterations:
            iteration += 1
            
            # Get current state
            c = test_path[plan_idx]
            if hasattr(c, 'to_list'):
                c_list = c.to_list()
            else:
                c_list = c
            
            # Set joint positions
            sim.set_joint_positions(c_list)
            
            # Get keyboard events
            keys = sim.client.getKeyboardEvents()
            
            # Debug: Print iteration info every 100 iterations
            if iteration % 100 == 0:
                print(f"Iteration {iteration}: plan_idx={plan_idx}, playing={playing}, keys={len(keys)}")
            
            # Handle keyboard input
            if space_code in keys and keys[space_code] & sim.client.KEY_WAS_TRIGGERED:
                playing = not playing
                print(f"Space pressed: playing = {playing}")
            
            elif not playing and left in keys and keys[left] & sim.client.KEY_WAS_TRIGGERED:
                plan_idx -= 1
                if plan_idx < 0:
                    plan_idx = len(test_path) - 1
                print(f"Left arrow: plan_idx = {plan_idx}")
                moved = True
            
            elif not playing and right in keys and keys[right] & sim.client.KEY_WAS_TRIGGERED:
                plan_idx += 1
                if plan_idx >= len(test_path):
                    plan_idx = 0
                print(f"Right arrow: plan_idx = {plan_idx}")
                moved = True
            
            elif playing:
                plan_idx += 1
                if plan_idx >= len(test_path):
                    plan_idx = 0
                moved = True
            
            if moved:
                if plan_idx >= len(test_path):
                    plan_idx = 0
                moved = False
            
            time.sleep(0.016)  # ~60 FPS
            
        print("Animation loop completed (max iterations reached)")
        return True
        
    except KeyboardInterrupt:
        print("\nAnimation loop interrupted by user")
        return True
    except Exception as e:
        print(f"✗ Animation loop failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_simple_animation(sim, test_path):
    """Test the simple animate function."""
    print("\n=== Testing Simple Animation ===")
    
    try:
        print("Calling sim.animate()...")
        sim.animate(test_path)
        print("✓ Animation completed successfully")
        return True
        
    except Exception as e:
        print(f"✗ Simple animation failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Main debug function."""
    print("PyBullet Animation Debug Script")
    print("=" * 50)
    
    # Test 1: PyBullet connection
    if not test_pybullet_connection():
        print("PyBullet connection failed, exiting")
        return
    
    # Test 2: Simulator creation
    sim = test_simulator_creation()
    if sim is None:
        print("Simulator creation failed, exiting")
        return
    
    # Test 3: Path creation
    test_path = test_path_creation()
    if test_path is None:
        print("Path creation failed, exiting")
        return
    
    # Test 4: Custom animation loop
    print("\n" + "="*50)
    print("Testing custom animation loop...")
    success = test_animation_loop(sim, test_path)
    
    if not success:
        print("Custom animation loop failed, trying simple animation...")
        test_simple_animation(sim, test_path)

if __name__ == "__main__":
    main() 