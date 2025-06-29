import numpy as np
from pathlib import Path
import pandas as pd
import random
import copy
import vamp
from fire import Fire

# Starting configuration
a = [0., -0.785, 0., -2.356, 0., 1.571, 0.785]

# Goal configuration
b = [2.35, 1., 0., -0.8, 0, 2.5, 0.785]

# Problem specification: a list of sphere centers
problem = [
    [0.55, 0, 0.25],
    [0.35, 0.35, 0.25],
    [0, 0.55, 0.25],
    [-0.55, 0, 0.25],
    [-0.35, -0.35, 0.25],
    [0, -0.55, 0.25],
    [0.35, -0.35, 0.25],
    [0.35, 0.35, 0.8],
    [0, 0.55, 0.8],
    [-0.35, 0.35, 0.8],
    [-0.55, 0, 0.8],
    [-0.35, -0.35, 0.8],
    [0, -0.55, 0.8],
    [0.35, -0.35, 0.8],
    ]


def main(
    variation: float = 0.01,
    benchmark: bool = True,
    n_trials: int = 100,
    radius: float = 0.2,
    visualize: bool = False,
    planner: str = "rrtc",
    sampler_name: str = "halton",  # Sampler to use.
    skip_rng_iterations: int = 0,  # Skip a number of RNG iterations
    **kwargs,
    ):

    (vamp_module, planner_func, plan_settings,
     simp_settings) = vamp.configure_robot_and_planner_with_kwargs("panda", planner, **kwargs)

    sampler = getattr(vamp_module, sampler_name)()
    sampler.skip(skip_rng_iterations)

    # Test validation in empty environment
    empty_env = vamp.Environment()
    start_valid_empty = vamp.panda.validate(a, empty_env)
    goal_valid_empty = vamp.panda.validate(b, empty_env)
    print(f"Start config valid in empty env: {start_valid_empty}")
    print(f"Goal config valid in empty env: {goal_valid_empty}")

    if benchmark:
        random.seed(0)
        np.random.seed(0)

        results = []
        spheres = [np.array(sphere) for sphere in problem]
        for trial in range(n_trials):
            random.shuffle(spheres)
            spheres_copy = copy.deepcopy(spheres)

            e = vamp.Environment()
            for sphere in spheres_copy:
                sphere += np.random.uniform(low = -variation, high = variation, size = (3, ))
                e.add_sphere(vamp.Sphere(sphere, radius))

            # Debug: Check validation
            start_valid = vamp.panda.validate(a, e)
            goal_valid = vamp.panda.validate(b, e)
            
            if not start_valid:
                print(f"Trial {trial}: Start configuration invalid")
                continue
            if not goal_valid:
                print(f"Trial {trial}: Goal configuration invalid")
                continue

            if start_valid and goal_valid:
                result = planner_func(a, b, e, plan_settings, sampler)
                simple = vamp_module.simplify(result.path, e, simp_settings, sampler)
                results.append(vamp.results_to_dict(result, simple))
                print(f"Trial {trial}: Success - added result")
            else:
                print(f"Trial {trial}: Validation failed - start: {start_valid}, goal: {goal_valid}")

        df = pd.DataFrame.from_dict(results)

        # Debug: Print the results to see what's happening
        print(f"Number of results: {len(results)}")
        if results:
            print(f"First result keys: {list(results[0].keys())}")
            print(f"First result: {results[0]}")
        else:
            print("No results found!")
            print("This means all trials failed validation or planning.")
            print("Try reducing the sphere radius or adjusting the start/goal configurations.")
            return

        # Convert to microseconds
        df["planning_time"] = df["planning_time"].dt.microseconds
        df["simplification_time"] = df["simplification_time"].dt.microseconds

        # Get summary statistics
        stats = df[[
            "planning_time",
            "simplification_time",
            "initial_path_cost",
            "simplified_path_cost",
            "planning_iterations"
            ]].describe()

        print(stats)

    if visualize:
        from vamp import pybullet_interface as vpb

        robot_dir = Path(__file__).parent.parent / 'resources' / 'panda'
        sim = vpb.PyBulletSimulator(
            str(robot_dir / f"panda_spherized.urdf"), vamp.ROBOT_JOINTS['panda'], True
            )

        e = vamp.Environment()
        for sphere in problem:
            e.add_sphere(vamp.Sphere(sphere, radius))
            sim.add_sphere(radius, sphere)

        result = planner_func(a, b, e, plan_settings, sampler)
        simple = vamp_module.simplify(result.path, e, simp_settings, sampler)

        simple.path.interpolate_to_resolution(vamp.panda.resolution())

        # Use the new multi-robot system for single robot (demonstrates backward compatibility)
        print("Single robot animation using new multi-robot system:")
        print("Press space to start/stop animation, use left/right arrows to step through states")
        
        # Method 1: Use the original animate function (backward compatible)
        sim.animate(simple.path)
        
        # Method 2: Use the new multi-robot system for single robot
        # This demonstrates how the new system can handle single robots too
        # sim.animate_multi([sim.skel_id], plan=simple.path)


if __name__ == "__main__":
    Fire(main)
