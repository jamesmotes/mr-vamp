import numpy as np
from pathlib import Path
import random
import copy
import vamp
from fire import Fire

# Starting configuration for each robot
a = [0., -0.785, 0., -2.356, 0., 1.571, 0.785]

# Goal configuration for each robot
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
    base_positions: list = None,  # List of (x, y, z) base positions for each robot
    **kwargs,
    ):
    
    # Default base positions if none provided
    if base_positions is None:
        base_positions = [
            (0.0, 0.0, 0.05),   # Robot 1 at origin
            (2.0, 0.0, 0.05),   # Robot 2 at x=2
            (0.0, 2.0, 0.05),   # Robot 3 at y=2
        ]
    
    print(f"Multi-robot setup with {len(base_positions)} robots at positions: {base_positions}")

    # Use the new multi-robot configuration function
    (vamp_module, planner_func, plan_settings,
     simp_settings, robot_positions) = vamp.configure_multi_robot_and_planner_with_kwargs(
        "panda", planner, base_positions, **kwargs
    )

    sampler = getattr(vamp_module, sampler_name)()
    sampler.skip(skip_rng_iterations)

    if benchmark:
        try:
            import pandas as pd
        except ImportError:
            print("pandas not available, skipping benchmark")
            benchmark = False
            
        if benchmark:
            random.seed(0)
            np.random.seed(0)

            results = []
            spheres = [np.array(sphere) for sphere in problem]
            for _ in range(n_trials):
                random.shuffle(spheres)
                spheres_copy = copy.deepcopy(spheres)

                e = vamp.Environment()
                for sphere in spheres_copy:
                    sphere += np.random.uniform(low = -variation, high = variation, size = (3, ))
                    e.add_sphere(vamp.Sphere(sphere, radius))

                # For multi-robot, we'll just test the first robot's start/goal
                # In a real multi-robot scenario, you'd have different start/goal for each robot
                if vamp.panda.validate(a, e) and vamp.panda.validate(b, e):
                    result = planner_func(a, b, e, plan_settings, sampler)
                    simple = vamp_module.simplify(result.path, e, simp_settings, sampler)
                    results.append(vamp.results_to_dict(result, simple))

            df = pd.DataFrame.from_dict(results)

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
        
        # Create a single simulator and load multiple robots
        sim = vpb.PyBulletSimulator(
            str(robot_dir / f"panda_spherized.urdf"), vamp.ROBOT_JOINTS['panda'], True
        )
        
        # Store the first robot's ID and set its position
        first_robot_id = sim.skel_id
        sim.set_robot_base_position(robot_positions[0])
        print(f"Robot 1 at position {robot_positions[0]}")
        
        # Load additional robots at different positions and register them
        robot_ids = [first_robot_id]
        for i, (x, y, z) in enumerate(robot_positions[1:], 2):
            # Create a new robot body at the desired position
            new_robot_id = sim.client.loadURDF(
                str(robot_dir / f"panda_spherized.urdf"),
                basePosition=(x, y, z),
                baseOrientation=(0, 0, 0, 1),
                useFixedBase=True,
                flags=sim.client.URDF_MAINTAIN_LINK_ORDER | sim.client.URDF_USE_SELF_COLLISION
            )
            robot_ids.append(new_robot_id)
            
            # Register the robot with the simulator for multi-robot animation
            sim.add_robot(new_robot_id, sim.joints, f"Robot_{i}")
            print(f"Robot {i} at position ({x}, {y}, {z})")

        # Create environment with obstacles
        e = vamp.Environment()
        for sphere in problem:
            e.add_sphere(vamp.Sphere(sphere, radius))
            sim.add_sphere(radius, sphere)

        # Plan the path (for this example, we'll use the same path for all robots)
        # In a real scenario, you'd plan different paths for each robot
        result = planner_func(a, b, e, plan_settings, sampler)
        simple = vamp_module.simplify(result.path, e, simp_settings, sampler)
        simple.path.interpolate_to_resolution(vamp.panda.resolution())

        # Use the new multi-robot animation system
        print("Multi-robot animation using new system:")
        print("Press space to start/stop global animation")
        print("Press Tab to select different robots for individual control")
        print("Press 0-9 to select specific robots")
        print("Press R to reset all robots")
        print("Press Q to quit")
        
        # Method 1: Simple multi-robot animation with same plan for all robots
        sim.animate_multi(robot_ids, plan=simple.path)
        
        # Method 2: Advanced multi-robot animation with different plans per robot
        # This demonstrates how you could have different paths for each robot
        # robot_plans = {
        #     robot_ids[0]: simple.path,
        #     robot_ids[1]: simple.path,  # Could be different path
        #     robot_ids[2]: simple.path,  # Could be different path
        # }
        # sim.animate_multi(robot_plans)
        
        # Method 3: Advanced configuration with custom callbacks
        # from vamp.pybullet_interface import RobotAnimationConfig
        # configs = [
        #     RobotAnimationConfig(
        #         robot_id=robot_ids[0],
        #         plan=simple.path,
        #         callback=lambda state, robot_id: print(f"Robot {robot_id} at state {state}"),
        #         name="Main Robot"
        #     ),
        #     RobotAnimationConfig(
        #         robot_id=robot_ids[1],
        #         plan=simple.path,
        #         speed=1.5,  # This robot moves 50% faster
        #         name="Fast Robot"
        #     ),
        #     RobotAnimationConfig(
        #         robot_id=robot_ids[2],
        #         plan=simple.path,
        #         start_time=0.5,  # This robot starts halfway through
        #         name="Delayed Robot"
        #     )
        # ]
        # sim.animate_multi_advanced(configs)


if __name__ == "__main__":
    Fire(main) 