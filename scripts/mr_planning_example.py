#!/usr/bin/env python3
"""
Multi-robot planning example using the MR planner framework.

This script demonstrates how to use the multi-robot planning framework
to generate plans for multiple Panda robots at different base positions.
No obstacles are included in this example.
"""

import numpy as np
from pathlib import Path
import vamp
from fire import Fire

# Different start and goal configurations for each robot
start_configs = [
    [0., -0.785, 0., -2.356, 0., 1.571, 0.785],      # Robot 1: neutral pose
    [1.5, -1.0, 0.5, -1.5, 0.5, 2.0, 1.0],           # Robot 2: different pose
    [-0.5, -0.5, -0.5, -2.0, -0.5, 1.0, 0.5],        # Robot 3: different pose
]

goal_configs = [
    [2.35, 1., 0., -0.8, 0, 2.5, 0.785],             # Robot 1: original goal
    [-1.0, 1.5, 0.5, -1.0, -0.5, 1.5, 1.5],          # Robot 2: different goal
    [1.5, -1.5, -0.5, -2.5, 0.5, 0.5, -0.5],         # Robot 3: different goal
]

def main(
    visualize: bool = True,
    planner: str = "dummy",
    n_robots: int = 3,
    base_positions: list = None,
    **kwargs,
):
    """
    Run multi-robot planning example.
    
    Args:
        visualize: Whether to visualize the results
        planner: Planner type to use ("dummy" for now)
        n_robots: Number of robots to use
        base_positions: List of (x, y, z) base positions for each robot
        **kwargs: Additional planner settings
    """
    
    # Default base positions if none provided
    if base_positions is None:
        base_positions = [
            (0.0, 0.0, 0.05),   # Robot 1 at origin
            (2.0, 0.0, 0.05),   # Robot 2 at x=2
            (0.0, 2.0, 0.05),   # Robot 3 at y=2
        ]
        # Add more robots if requested
        if n_robots > 3:
            for i in range(3, n_robots):
                x = (i % 3) * 2.0
                y = (i // 3) * 2.0
                base_positions.append((x, y, 0.05))
    
    # Limit to requested number of robots
    base_positions = base_positions[:n_robots]
    
    print(f"Multi-robot planning with {len(base_positions)} robots at positions: {base_positions}")
    print(f"Using planner: {planner}")

    # Create start and goal configurations for each robot
    starts = []
    goals = []
    for i in range(len(base_positions)):
        if i < len(start_configs):
            starts.append(start_configs[i])
            goals.append(goal_configs[i])
        else:
            # For additional robots, use variations of the first config
            starts.append([c + 0.1 * i for c in start_configs[0]])
            goals.append([c + 0.1 * i for c in goal_configs[0]])
    
    print(f"Robot configurations:")
    for i, (start, goal) in enumerate(zip(starts, goals)):
        print(f"  Robot {i}: start={start[:3]}..., goal={goal[:3]}...")
    
    # Create empty environment (no obstacles)
    env = vamp.Environment()
    
    # Create multi-robot problem
    print("\nCreating multi-robot problem...")
    problem = vamp.mr_planning.create_mr_problem(
        starts=starts,
        goals=goals, 
        base_positions=base_positions,
        environment=env,
        problem_name="mr_planning_example"
    )
    
    print(f"Problem created with {problem.num_robots()} robots")
    print(f"Problem valid: {problem.is_valid()}")
    
    # Configure settings
    settings = vamp.mr_planning.MRSettings()
    settings.algorithm = planner
    settings.max_roadmap_size = 1000
    settings.max_roadmap_iterations = 5000
    settings.enable_inter_robot_collision_checking = False  # No obstacles in this example
    
    # Solve the multi-robot problem
    print(f"\nSolving multi-robot problem with {planner} planner...")
    result = vamp.mr_planning.solve_mr_problem(
        problem=problem,
        planner_type=planner,
        settings=settings
    )
    
    # Print results
    print(f"\nPlanning completed!")
    print(f"Success: {result.success}")
    print(f"Algorithm: {result.algorithm_name}")
    print(f"Planning time: {result.nanoseconds / 1e6:.2f} ms")
    print(f"Iterations: {result.iterations}")
    print(f"Total cost: {result.total_cost:.3f}")
    print(f"Number of robot paths: {result.num_robots()}")
    
    for i, path in enumerate(result.robot_paths):
        print(f"  Robot {i}: {len(path)} waypoints, cost: {path.cost():.3f}")
    
    if visualize:
        from vamp import pybullet_interface as vpb
        
        robot_dir = Path(__file__).parent.parent / 'resources' / 'panda'
        
        # Create simulator
        sim = vpb.PyBulletSimulator(
            str(robot_dir / "panda_spherized.urdf"), 
            vamp.ROBOT_JOINTS['panda'], 
            visualize=True
        )
        
        # Set up first robot at its position
        first_robot_id = sim.skel_id
        sim.set_robot_base_position(base_positions[0])
        print(f"\nRobot 1 at position {base_positions[0]}")
        
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
        
        # Prepare paths for animation
        print("\nPreparing paths for animation...")
        animated_paths = []
        valid_paths = []
        
        for i, path in enumerate(result.robot_paths):
            if len(path) > 0:
                # Interpolate path to resolution for smooth animation
                path.interpolate_to_resolution(vamp.panda.resolution())
                animated_paths.append(path)
                valid_paths.append(i)
                print(f"  Robot {i}: {len(path)} waypoints ready for animation")
            else:
                print(f"  Robot {i}: No path available")
        
        if animated_paths:
            print(f"\nStarting multi-robot animation with {len(animated_paths)} robots...")
            print("Controls:")
            print("  Space: Start/stop global animation")
            print("  Tab: Select next robot for individual control")
            print("  0-9: Select specific robot by number")
            print("  Left/Right arrows: Step through states (when paused)")
            print("  R: Reset all robots")
            print("  Q: Quit animation")
            
            # Create robot plans dictionary with individual paths
            robot_plans = {}
            for i, robot_id in enumerate(robot_ids):
                if i < len(animated_paths):
                    robot_plans[robot_id] = animated_paths[i]
                else:
                    # If no path for this robot, use the first available path
                    robot_plans[robot_id] = animated_paths[0]
            
            # Animate robots with their individual paths
            sim.animate_multi(robot_plans)
            
        else:
            print("\nNo valid paths to animate!")
            print("Creating fallback animation with start/goal configurations...")
            
            # Create simple fallback paths from start to goal
            fallback_paths = {}
            for i, robot_id in enumerate(robot_ids):
                # Create a simple path with just start and goal for this specific robot
                fallback_path = vamp.panda.Path()
                fallback_path.append(vamp.panda.Configuration(starts[i]))
                fallback_path.append(vamp.panda.Configuration(goals[i]))
                fallback_path.interpolate_to_resolution(vamp.panda.resolution())
                fallback_paths[robot_id] = fallback_path
            
            print("Starting fallback animation...")
            sim.animate_multi(fallback_paths)
    
    return result

if __name__ == "__main__":
    Fire(main) 