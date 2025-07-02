#!/usr/bin/env python3
"""
Minimal test to just open a PyBullet window.
"""

import vamp
from vamp import pybullet_interface as vpb
from pathlib import Path
import time

def main():
    """Test basic PyBullet window functionality."""
    
    print("Testing PyBullet window...")
    
    # Create simulator
    robot_dir = Path(__file__).parent.parent / 'resources' / 'panda'
    sim = vpb.PyBulletSimulator(
        str(robot_dir / "panda_spherized.urdf"), 
        vamp.ROBOT_JOINTS['panda'], 
        visualize=True
    )
    
    print("PyBullet window should be open now.")
    print("Press Ctrl+C to exit.")
    
    try:
        # Just keep the window open
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == "__main__":
    main() 