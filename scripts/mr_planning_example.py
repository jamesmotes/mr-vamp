#!/usr/bin/env python3
"""
Multi-robot planning example using the MR planning framework.

This script demonstrates how to use the multi-robot planning framework
to plan for multiple Panda robots with different base positions.
"""

import numpy as np
import vamp
from vamp.mr_planning import (
    MRSettings, MRPlanningResult, FloatVector7,
    create_mr_problem, create_dummy_planner
)

def create_float_vector_7(values):
    """Create a FloatVector7 from a list of 7 values."""
    if len(values) != 7:
        raise ValueError("Expected 7 values for Panda robot configuration")
    return FloatVector7(values)

def main(visualize: bool = True):
    print("Multi-Robot Planning Example")
    print("=" * 40)
    
    # Create environment (empty for this example)
    env = vamp.Environment()
    
    # Define robot base positions (in meters)
    # These correspond to the grid positions supported by the template variants
    base_positions = [
        [0.0, 0.0, 0.05],    # Panda_0_0
        [1.0, 0.0, 0.05],    # Panda_1_0  
        [2.0, 0.0, 0.05],    # Panda_2_0
    ]
    
    print(f"Planning for {len(base_positions)} robots")
    for i, pos in enumerate(base_positions):
        print(f"  Robot {i}: base position {pos}")
    
    # Define start and goal configurations for each robot
    # Using simple configurations that should be reachable
    starts = [
        create_float_vector_7([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),  # Robot 0
        create_float_vector_7([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),  # Robot 1
        create_float_vector_7([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),  # Robot 2
    ]
    
    goals = [
        create_float_vector_7([0.0, 0.785, 0.0, -0.785, 0.0, 1.571, -0.785]),   # Robot 0
        create_float_vector_7([0.0, 0.785, 0.0, -0.785, 0.0, 1.571, -0.785]),   # Robot 1
        create_float_vector_7([0.0, 0.785, 0.0, -0.785, 0.0, 1.571, -0.785]),   # Robot 2
    ]
    
    print("\nStart configurations:")
    for i, start in enumerate(starts):
        print(f"  Robot {i}: {start.to_list()}")
    
    print("\nGoal configurations:")
    for i, goal in enumerate(goals):
        print(f"  Robot {i}: {goal.to_list()}")
    
    # Create multi-robot planning settings
    settings = MRSettings()
    settings.algorithm = "dummy"
    settings.enable_inter_robot_collision_checking = False  # Ignore inter-robot collisions for demo
    settings.enable_parallel_roadmap_construction = True
    
    print(f"\nUsing algorithm: {settings.algorithm}")
    print(f"Inter-robot collision checking: {settings.enable_inter_robot_collision_checking}")
    print(f"Parallel roadmap construction: {settings.enable_parallel_roadmap_construction}")
    
    try:
        # Method 1: Use the helper function
        print("\n" + "="*40)
        print("Method 1: Using create_mr_problem and solve_mr_problem helper functions")
        print("="*40)
        
        # Create the problem first
        problem = create_mr_problem(base_positions, starts, goals, env, settings)
        print(f"Created problem with {problem.num_robots()} robots")
        
        # Solve the problem
        from vamp.mr_planning import solve_mr_problem
        result = solve_mr_problem(problem, "dummy", settings)
        
        print(f"Planning successful: {result.success}")
        print(f"Algorithm used: {result.algorithm_name}")
        print(f"Total cost: {result.total_cost}")
        print(f"Planning time: {result.nanoseconds / 1e6:.2f} ms")
        print(f"Iterations: {result.iterations}")
        
        for i, path in enumerate(result.robot_paths):
            print(f"  Robot {i}: {len(path)} waypoints")
            if path:
                print(f"    Start: {path[0].to_list()}")
                print(f"    End: {path[len(path)-1].to_list()}")
        
        # Method 2: Create planner directly
        print("\n" + "="*40)
        print("Method 2: Creating planner directly")
        print("="*40)
        
        planner = create_dummy_planner(base_positions, env, settings)
        
        # Build roadmaps first
        print("Building roadmaps...")
        planner.build_roadmaps(starts, goals)
        
        if planner.are_roadmaps_built():
            print("Roadmaps built successfully")
            print(f"Roadmap build time: {planner.get_roadmap_build_time_ns() / 1e6:.2f} ms")
        else:
            print("Failed to build roadmaps")
            return
        
        # Solve the planning problem
        print("Solving planning problem...")
        result2 = planner.solve(starts, goals)
        
        print(f"Planning successful: {result2.success}")
        print(f"Algorithm used: {result2.algorithm_name}")
        print(f"Total cost: {result2.total_cost}")
        print(f"Planning time: {result2.nanoseconds / 1e6:.2f} ms")
        print(f"Iterations: {result2.iterations}")
        
        for i, path in enumerate(result2.robot_paths):
            print(f"  Robot {i}: {len(path)} waypoints")
            if path:
                print(f"    Start: {path[0].to_list()}")
                print(f"    End: {path[len(path)-1].to_list()}")
        
        # Verify results are the same
        print("\n" + "="*40)
        print("Verification")
        print("="*40)
        
        if result.success == result2.success and len(result.robot_paths) == len(result2.robot_paths):
            print("✓ Both methods produced consistent results")
            
            # Check if paths are identical
            paths_match = True
            for i, (path1, path2) in enumerate(zip(result.robot_paths, result2.robot_paths)):
                if len(path1) != len(path2):
                    paths_match = False
                    break
                for j, (wp1, wp2) in enumerate(zip(path1, path2)):
                    if wp1.to_list() != wp2.to_list():
                        paths_match = False
                        break
                if not paths_match:
                    break
            
            if paths_match:
                print("✓ Paths are identical between methods")
            else:
                print("⚠ Paths differ between methods (this is expected for some algorithms)")
        else:
            print("✗ Results are inconsistent between methods")
        
        # Visualization
        if visualize and result.success:
            print("\n" + "="*40)
            print("Visualization")
            print("="*40)
            
            from vamp import pybullet_interface as vpb
            from pathlib import Path
            
            # Create simulator
            robot_dir = Path(__file__).parent.parent / 'resources' / 'panda'
            sim = vpb.PyBulletSimulator(
                str(robot_dir / "panda_spherized.urdf"), 
                vamp.ROBOT_JOINTS['panda'], 
                visualize=True
            )
            
            # Set up first robot at origin
            first_robot_id = sim.skel_id
            sim.set_robot_base_position(base_positions[0])
            print(f"Main robot at position {base_positions[0]}")
            
            # Load additional robots at different positions
            robot_ids = [first_robot_id]
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
            
            # Debug: Check the original paths
            print("\nDebug: Original paths from multi-robot planning:")
            any_empty = False
            for i, path in enumerate(result.robot_paths):
                print(f"  Robot {i}: {len(path)} waypoints")
                if len(path) == 0:
                    print(f"  WARNING: Robot {i} has an empty plan!")
                    any_empty = True
                elif path:
                    print(f"    Start: {path[0].to_list()}")
                    print(f"    End: {path[len(path)-1].to_list()}")
            if any_empty:
                print("\nERROR: One or more robot plans are empty. Multi-robot animation will not run.")
                return
            
            # Convert paths to the format expected by the visualization system
            # The visualization expects paths with Configuration objects, not FloatVector7
            Path = vamp.panda.Path
            Configuration = vamp.panda.Configuration
            
            robot_plans = {}
            for i, (robot_id, path) in enumerate(zip(robot_ids, result.robot_paths)):
                print(f"Converting path for robot {i} (ID: {robot_id})")
                
                # Convert FloatVector7 path to Configuration path
                panda_path = Path()
                for waypoint in path:
                    panda_path.append(Configuration(waypoint.to_list()))
                
                robot_plans[robot_id] = panda_path
                print(f"  Converted: {len(panda_path)} waypoints")
            
            print(f"\nStarting multi-robot animation with {len(robot_plans)} robots...")
            print("Controls: Space=pause/play, Left/Right=step, Q=quit")
            
            # Start the animation
            sim.animate_multi(robot_plans)
        
    except Exception as e:
        print(f"Error during planning: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    import fire
    fire.Fire(main) 