#!/usr/bin/env python3
"""
Multi-robot planning test script using the MR planning framework.

This script tests the multi-robot planning framework with different scenarios:
- Different base positions for robots
- Different start/goal configurations for each robot
- Optional obstacles for testing collision avoidance
- Multiple test scenarios
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

def create_test_scenarios():
    """Create different test scenarios for multi-robot planning."""
    
    scenarios = {
        "simple_3_robots": {
            "name": "Simple 3-Robot Test",
            "base_positions": [
                [0.0, 0.0, 0.05],    # Robot 0 at origin
                [1.0, 0.0, 0.05],    # Robot 1 at x=1
                [2.0, 0.0, 0.05],    # Robot 2 at x=2
            ],
            "starts": [
                create_float_vector_7([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),   # Robot 0
                create_float_vector_7([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),   # Robot 1
                create_float_vector_7([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),   # Robot 2
            ],
            "goals": [
                create_float_vector_7([0.0, 0.785, 0.0, -0.785, 0.0, 1.571, -0.785]),    # Robot 0
                create_float_vector_7([0.0, 0.785, 0.0, -0.785, 0.0, 1.571, -0.785]),    # Robot 1
                create_float_vector_7([0.0, 0.785, 0.0, -0.785, 0.0, 1.571, -0.785]),    # Robot 2
            ],
            "obstacles": False,
            "description": "Basic test with 3 robots in a line, same start/goal for all"
        },
        
        "varied_configs": {
            "name": "Varied Configurations Test",
            "base_positions": [
                [0.0, 0.0, 0.05],    # Robot 0 at origin
                [1.0, 0.0, 0.05],    # Robot 1 at x=1 (fixed: was 1.5)
                [0.0, 1.0, 0.05],    # Robot 2 at y=1 (fixed: was 1.5)
            ],
            "starts": [
                create_float_vector_7([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),   # Robot 0: low arm
                create_float_vector_7([1.57, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),  # Robot 1: rotated base
                create_float_vector_7([0.0, 0.785, 0.0, -0.785, 0.0, 1.571, -0.785]),   # Robot 2: high arm
            ],
            "goals": [
                create_float_vector_7([0.0, 0.785, 0.0, -0.785, 0.0, 1.571, -0.785]),    # Robot 0: high arm
                create_float_vector_7([-1.57, 0.785, 0.0, -0.785, 0.0, 1.571, -0.785]),  # Robot 1: opposite rotation
                create_float_vector_7([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),    # Robot 2: low arm
            ],
            "obstacles": False,
            "description": "Different start/goal configurations for each robot"
        },
        
        "with_obstacles": {
            "name": "Obstacle Avoidance Test",
            "base_positions": [
                [0.0, 0.0, 0.05],    # Robot 0 at origin
                [1.0, 0.0, 0.05],    # Robot 1 at x=1
                [2.0, 0.0, 0.05],    # Robot 2 at x=2
            ],
            "starts": [
                create_float_vector_7([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),   # Robot 0
                create_float_vector_7([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),   # Robot 1
                create_float_vector_7([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),   # Robot 2
            ],
            "goals": [
                create_float_vector_7([0.0, 0.785, 0.0, -0.785, 0.0, 1.571, -0.785]),    # Robot 0
                create_float_vector_7([0.0, 0.785, 0.0, -0.785, 0.0, 1.571, -0.785]),    # Robot 1
                create_float_vector_7([0.0, 0.785, 0.0, -0.785, 0.0, 1.571, -0.785]),    # Robot 2
            ],
            "obstacles": True,
            "description": "Same as simple test but with obstacles in the workspace"
        },
        
        "complex_4_robots": {
            "name": "Complex 4-Robot Test",
            "base_positions": [
                [0.0, 0.0, 0.05],    # Robot 0 at origin
                [1.0, 0.0, 0.05],    # Robot 1 at x=1
                [0.0, 1.0, 0.05],    # Robot 2 at y=1
                [1.0, 1.0, 0.05],    # Robot 3 at (1,1)
            ],
            "starts": [
                create_float_vector_7([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),   # Robot 0
                create_float_vector_7([1.57, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),  # Robot 1
                create_float_vector_7([0.0, 0.785, 0.0, -0.785, 0.0, 1.571, -0.785]),   # Robot 2
                create_float_vector_7([-1.57, 0.785, 0.0, -0.785, 0.0, 1.571, -0.785]), # Robot 3
            ],
            "goals": [
                create_float_vector_7([0.0, 0.785, 0.0, -0.785, 0.0, 1.571, -0.785]),    # Robot 0
                create_float_vector_7([-1.57, 0.785, 0.0, -0.785, 0.0, 1.571, -0.785]),  # Robot 1
                create_float_vector_7([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),    # Robot 2
                create_float_vector_7([1.57, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]),   # Robot 3
            ],
            "obstacles": False,
            "description": "4 robots in a square formation with varied configurations"
        }
    }
    
    return scenarios

def create_obstacle_environment():
    """Create an environment with obstacles for testing."""
    env = vamp.Environment()
    
    # Add some spheres as obstacles
    obstacles = [
        vamp.Sphere([0.5, 0.0, 0.3], 0.2),    # Center obstacle
        vamp.Sphere([1.5, 0.0, 0.3], 0.2),    # Right obstacle
        vamp.Sphere([0.0, 0.5, 0.3], 0.15),   # Back obstacle
    ]
    
    for obstacle in obstacles:
        env.add_sphere(obstacle)
    
    return env

def interpolate_path_to_resolution(path, resolution=32):
    """Interpolate path to the specified resolution (integer)."""
    if len(path) < 2:
        return path
    
    # Convert to vamp planning Path type
    vamp_path = vamp.planning.Path[7]()  # 7 for Panda dimension
    for config in path:
        vamp_path.append(config)
    
    # Interpolate to resolution
    vamp_path.interpolate_to_resolution(resolution)
    
    # Convert back to list
    return [vamp_path[i] for i in range(len(vamp_path))]

def run_scenario(scenario_name, scenario_data, visualize=True):
    """Run a specific test scenario."""
    print(f"\n{'='*60}")
    print(f"Running Scenario: {scenario_data['name']}")
    print(f"{'='*60}")
    print(f"Description: {scenario_data['description']}")
    
    # Create environment
    if scenario_data['obstacles']:
        env = create_obstacle_environment()
        print("Environment: With obstacles")
    else:
        env = vamp.Environment()
        print("Environment: Empty")
    
    base_positions = scenario_data['base_positions']
    starts = scenario_data['starts']
    goals = scenario_data['goals']
    
    print(f"Number of robots: {len(base_positions)}")
    for i, pos in enumerate(base_positions):
        print(f"  Robot {i}: base position {pos}")
    
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
    
    print(f"\nSettings:")
    print(f"  Algorithm: {settings.algorithm}")
    print(f"  Inter-robot collision checking: {settings.enable_inter_robot_collision_checking}")
    print(f"  Parallel roadmap construction: {settings.enable_parallel_roadmap_construction}")
    
    try:
        # Create and solve the problem
        print(f"\n{'='*40}")
        print("Planning")
        print(f"{'='*40}")
        
        # Create the problem
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
        
        print("\nPath details:")
        for i, path in enumerate(result.robot_paths):
            print(f"  Robot {i}: {len(path)} waypoints")
            if path:
                print(f"    Start: {path[0].to_list()}")
                print(f"    End: {path[len(path)-1].to_list()}")
        
        # Store result for visualization
        result_for_viz = result
        
        # Visualize results
        if visualize:
            print(f"\nVisualizing {scenario_name}...")
            
            # Convert paths to visualization format and interpolate
            robot_paths = []
            for i, path in enumerate(result_for_viz.robot_paths):
                if path:
                    # Interpolate path to robot resolution for smooth visualization
                    interpolated_path = interpolate_path_to_resolution(path, resolution=32)
                    robot_paths.append(interpolated_path)
                else:
                    robot_paths.append([])
            
            # Create multi-robot animation
            vamp.animate_multiple_robots(
                robot_paths=robot_paths,
                robot_types=['panda'] * len(robot_paths),
                base_positions=scenario_data['base_positions'],
                environment=env,
                title=f"Multi-Robot Planning: {scenario_name}"
            )
        
        return result.success
        
    except Exception as e:
        print(f"Error during planning: {e}")
        import traceback
        traceback.print_exc()
        return False

def main(scenario_name=None, visualize=True):
    """Main function to run multi-robot planning tests."""
    print("Multi-Robot Planning Test Suite")
    print("=" * 60)
    
    # Get all available scenarios
    scenarios = create_test_scenarios()
    
    if scenario_name is None:
        # Run all scenarios
        print(f"Running all {len(scenarios)} scenarios...")
        results = {}
        
        for name, data in scenarios.items():
            success = run_scenario(name, data, visualize)
            results[name] = success
            
            if not success:
                print(f"⚠ Scenario '{name}' failed!")
            else:
                print(f"✓ Scenario '{name}' completed successfully!")
        
        # Summary
        print(f"\n{'='*60}")
        print("Test Summary")
        print(f"{'='*60}")
        for name, success in results.items():
            status = "✓ PASS" if success else "✗ FAIL"
            print(f"{status} {scenarios[name]['name']}")
        
        passed = sum(results.values())
        total = len(results)
        print(f"\nOverall: {passed}/{total} scenarios passed")
        
    elif scenario_name in scenarios:
        # Run specific scenario
        success = run_scenario(scenario_name, scenarios[scenario_name], visualize)
        if success:
            print(f"\n✓ Scenario '{scenario_name}' completed successfully!")
        else:
            print(f"\n✗ Scenario '{scenario_name}' failed!")
    else:
        print(f"Unknown scenario: {scenario_name}")
        print(f"Available scenarios: {list(scenarios.keys())}")

if __name__ == "__main__":
    import sys
    
    # Parse command line arguments
    scenario = None
    visualize = True
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "--help" or sys.argv[1] == "-h":
            print("Usage: python mr_planning_example.py [scenario_name] [--no-viz]")
            print("\nAvailable scenarios:")
            scenarios = create_test_scenarios()
            for name, data in scenarios.items():
                print(f"  {name}: {data['name']}")
            print("\nExamples:")
            print("  python mr_planning_example.py                    # Run all scenarios")
            print("  python mr_planning_example.py simple_3_robots   # Run specific scenario")
            print("  python mr_planning_example.py --no-viz          # Run without visualization")
            sys.exit(0)
        elif sys.argv[1] == "--no-viz":
            visualize = False
        else:
            scenario = sys.argv[1]
    
    if len(sys.argv) > 2 and sys.argv[2] == "--no-viz":
        visualize = False
    
    main(scenario, visualize) 