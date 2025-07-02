#!/usr/bin/env python3
"""
Check what modules and classes are available in the vamp package.
"""

import vamp
import inspect

def main():
    """Check vamp package contents."""
    print("VAMP Package Contents")
    print("=" * 50)
    
    # List all attributes in vamp
    print("Available attributes in vamp:")
    for attr_name in dir(vamp):
        if not attr_name.startswith('_'):
            attr = getattr(vamp, attr_name)
            attr_type = type(attr).__name__
            print(f"  {attr_name}: {attr_type}")
    
    print("\n" + "=" * 50)
    print("Checking for specific modules:")
    
    # Check for panda module
    try:
        panda_module = vamp.panda
        print("✓ vamp.panda exists")
        print(f"  Type: {type(panda_module)}")
        print(f"  Attributes: {[attr for attr in dir(panda_module) if not attr.startswith('_')]}")
    except AttributeError:
        print("✗ vamp.panda does not exist")
    
    # Check for other potential modules
    potential_modules = ['panda', 'mr_planning', 'pybullet_interface', 'Environment', 'FloatVector7']
    for module_name in potential_modules:
        try:
            module = getattr(vamp, module_name)
            print(f"✓ vamp.{module_name} exists: {type(module)}")
        except AttributeError:
            print(f"✗ vamp.{module_name} does not exist")
    
    print("\n" + "=" * 50)
    print("Checking for Path and Configuration classes:")
    
    # Try different ways to import Path and Configuration
    try:
        from vamp import Path
        print("✓ vamp.Path exists")
    except ImportError:
        print("✗ vamp.Path does not exist")
    
    try:
        from vamp import Configuration
        print("✓ vamp.Configuration exists")
    except ImportError:
        print("✗ vamp.Configuration does not exist")
    
    try:
        from vamp import FloatVector7
        print("✓ vamp.FloatVector7 exists")
    except ImportError:
        print("✗ vamp.FloatVector7 does not exist")
    
    # Check if they exist in mr_planning
    try:
        from vamp.mr_planning import Path, Configuration, FloatVector7
        print("✓ Path, Configuration, FloatVector7 exist in vamp.mr_planning")
    except ImportError as e:
        print(f"✗ Import from vamp.mr_planning failed: {e}")

if __name__ == "__main__":
    main() 