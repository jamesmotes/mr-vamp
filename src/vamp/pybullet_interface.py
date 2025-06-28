from dataclasses import dataclass
from pathlib import Path

import sys
import time
import numpy as np
import xmltodict
from typing import Any, Dict, List, Optional, Union, Callable
from zlib import crc32
from colorsys import hsv_to_rgb

import pybullet as pb
from pybullet_utils.bullet_client import BulletClient
from .redirect_stream import RedirectStream
from .disable_rendering import DisableRendering
from .typing import *


def string_to_01(b: str) -> float:
    return float(crc32(b.encode('utf-8')) & 0xffffffff) / 2**32


def name_to_color(name: str) -> List[float]:
    return [*hsv_to_rgb(string_to_01(name), 0.5, 0.9), 1.]


def handle_color(name: Optional[str], color: Optional[Union[List[float], str]]) -> List[float]:
    if not color and name:
        return name_to_color(name)
    elif isinstance(color, str):
        return name_to_color(color)
    elif not color:
        return [0.5, 0.5, 0.5, 1.]
    else:
        return color


@dataclass
class RobotAnimationConfig:
    """Configuration for a single robot's animation."""
    robot_id: int
    plan: Any
    callback: Optional[Callable] = None
    speed: float = 1.0
    start_time: float = 0.0
    name: Optional[str] = None


@dataclass
class PyBulletSimulator:
    client: BulletClient

    def __init__(self, urdf: str, joints: List[str], visualize: bool = True):
        with RedirectStream(sys.stdout):
            if visualize:
                self.client = BulletClient(connection_mode = pb.GUI)
                self.client.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
                self.client.configureDebugVisualizer(pb.COV_ENABLE_MOUSE_PICKING, 0)
            else:
                self.client = BulletClient(connection_mode = pb.DIRECT)

        self.client.setRealTimeSimulation(0)
        self.objects = []
        self.robots = {}  # Registry for additional robots

        if urdf:
            self.urdf = urdf
            with DisableRendering(self.client), RedirectStream(sys.stdout), RedirectStream(sys.stderr):
                self.skel_id = self.client.loadURDF(
                    urdf,
                    basePosition = (0, 0, 0),
                    baseOrientation = (0, 0, 0, 1),
                    useFixedBase = True,
                    flags = pb.URDF_MAINTAIN_LINK_ORDER | pb.URDF_USE_SELF_COLLISION
                    )

        if urdf and joints:

            jtu = [
                [
                    i.decode() if isinstance(i, bytes) else i
                    for i in self.client.getJointInfo(self.skel_id, j)
                    ]
                for j in range(self.client.getNumJoints(self.skel_id))
                ]
            jt = sorted(filter(lambda ji: ji[1] in joints, jtu), key = lambda ji: joints.index(ji[1]))

            self.joints = [ji[0] for ji in jt]
            self.lows = [ji[8] for ji in jt]
            self.highs = [ji[9] for ji in jt]

            self.link_map = {ji[12]: ji[0] for ji in jtu}

            if urdf:
                srdffile = list(Path(urdf).parent.glob('*.srdf'))[0]
                if srdffile:
                    with open(srdffile, 'r') as f:
                        srdf = xmltodict.parse(f.read())

                    for disabled in srdf['robot']['disable_collisions']:
                        link1, link2 = disabled['@link1'], disabled['@link2']
                        l1x = self.link_map[link1] if link1 in self.link_map else -1
                        l2x = self.link_map[link2] if link2 in self.link_map else -1
                        self.client.setCollisionFilterPair(0, 0, l1x, l2x, False)

    def add_robot(self, robot_id: int, joints: List[int], name: str = None):
        """Register an additional robot for multi-robot animation."""
        self.robots[robot_id] = {
            'joints': joints,
            'name': name or f"Robot_{robot_id}"
        }
        print(f"Registered robot {robot_id} ({self.robots[robot_id]['name']}) with {len(joints)} joints")

    def set_joint_positions(self, positions: List[float], robot_id: Optional[int] = None):
        """Set joint positions for a specific robot or the main robot."""
        if robot_id is None:
            # Use main robot
            for joint, value in zip(self.joints, positions):
                self.client.resetJointState(self.skel_id, joint, value, targetVelocity = 0)
        else:
            # Use specific robot
            if robot_id == self.skel_id:
                # Main robot - use main robot joints
                for joint, value in zip(self.joints, positions):
                    self.client.resetJointState(self.skel_id, joint, value, targetVelocity = 0)
            elif robot_id in self.robots:
                # Registered robot - use its joints
                robot_joints = self.robots[robot_id]['joints']
                for joint, value in zip(robot_joints, positions):
                    self.client.resetJointState(robot_id, joint, value, targetVelocity = 0)
            else:
                raise ValueError(f"Robot {robot_id} not registered. Use add_robot() first.")

    def set_robot_base_position(self, position: Position, orientation: XYZWQuaternion = [0, 0, 0, 1], robot_id: Optional[int] = None):
        """Set the robot's base position and orientation."""
        target_id = robot_id if robot_id is not None else self.skel_id
        self.client.resetBasePositionAndOrientation(target_id, position, orientation)

    def in_collision(self) -> bool:
        self.client.performCollisionDetection()
        dists = list(point[8] for point in self.client.getContactPoints())
        return bool(dists) and min(dists) < 0

    def set_camera(self, position: Position, look_at: Position):
        dx = position[0] - look_at[0]
        dy = position[1] - look_at[1]
        dz = position[2] - look_at[2]

        import math

        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        yaw = math.atan2(dz, dx)
        pitch = math.atan2(math.sqrt(dz * dz + dx * dx), dy) + math.pi

        self.client.resetDebugVisualizerCamera(
            cameraDistance = distance,
            cameraYaw = math.degrees(yaw),
            cameraPitch = math.degrees(pitch),
            cameraTargetPosition = look_at
            )

    def add_capsule(
        self,
        radius: float,
        length: float,
        position: Position,
        rot_xyzw: XYZWQuaternion,
        name: Optional[str] = None,
        color: Optional[Union[List[float], str]] = None,
        ):
        with DisableRendering(self.client):
            vis_shape_id = self.client.createVisualShape(
                pb.GEOM_CAPSULE,
                radius = radius,
                length = length,
                rgbaColor = handle_color(name, color),
                )
            col_shape_id = self.client.createCollisionShape(pb.GEOM_CAPSULE, radius = radius, height = length)

            multibody_id = self.client.createMultiBody(
                baseVisualShapeIndex = vis_shape_id,
                baseCollisionShapeIndex = col_shape_id,
                basePosition = position,
                baseOrientation = rot_xyzw,
                )

            if name:
                self.client.addUserDebugText(
                    text = name,
                    textPosition = position,
                    textColorRGB = [0., 0., 0.],
                    )

                return multibody_id

    def add_cylinder(
        self,
        radius: float,
        length: float,
        position: Position,
        rot_xyzw: XYZWQuaternion,
        name: Optional[str] = None,
        color: Optional[Union[List[float], str]] = None,
        ):
        with DisableRendering(self.client):
            vis_shape_id = self.client.createVisualShape(
                pb.GEOM_CYLINDER,
                radius = radius,
                length = length,
                rgbaColor = handle_color(name, color),
                )
            col_shape_id = self.client.createCollisionShape(
                pb.GEOM_CYLINDER, radius = radius, height = length
                )

            multibody_id = self.client.createMultiBody(
                baseVisualShapeIndex = vis_shape_id,
                baseCollisionShapeIndex = col_shape_id,
                basePosition = position,
                baseOrientation = rot_xyzw,
                )

            if name:
                self.client.addUserDebugText(
                    text = name,
                    textPosition = position,
                    textColorRGB = [0., 0., 0.],
                    )

                return multibody_id

    def add_cuboid(
        self,
        half_extents: Vector3,
        position: Position,
        rot_xyzw: XYZWQuaternion,
        name: Optional[str] = None,
        color: Optional[Union[List[float], str]] = None,
        ):

        with DisableRendering(self.client):
            vis_shape_id = self.client.createVisualShape(
                pb.GEOM_BOX,
                halfExtents = half_extents,
                rgbaColor = handle_color(name, color),
                )
            col_shape_id = self.client.createCollisionShape(pb.GEOM_BOX, halfExtents = half_extents)

            multibody_id = self.client.createMultiBody(
                baseVisualShapeIndex = vis_shape_id,
                baseCollisionShapeIndex = col_shape_id,
                basePosition = position,
                baseOrientation = rot_xyzw,
                )

            if name:
                self.client.addUserDebugText(
                    text = name,
                    textPosition = position,
                    textColorRGB = [0., 0., 0.],
                    )

                return multibody_id

    def add_sphere(
        self,
        radius: float,
        position: Position,
        name: Optional[str] = None,
        color: Optional[Union[List[float], str]] = None,
        ):

        with DisableRendering(self.client):
            vis_shape_id = self.client.createVisualShape(
                pb.GEOM_SPHERE,
                radius = radius,
                rgbaColor = handle_color(name, color),
                )
            col_shape_id = self.client.createCollisionShape(pb.GEOM_SPHERE, radius = radius)

            multibody_id = self.client.createMultiBody(
                baseVisualShapeIndex = vis_shape_id,
                baseCollisionShapeIndex = col_shape_id,
                basePosition = position,
                )

            if name:
                self.client.addUserDebugText(
                    text = name,
                    textPosition = position,
                    textColorRGB = [0., 0., 0.],
                    )

            return multibody_id

    def update_object_position(
            self, multibody_id: int, position: Position, rot_xywz: XYZWQuaternion = [0, 0, 0, 1]
        ):
        self.client.resetBasePositionAndOrientation(multibody_id, position, rot_xywz)

    def add_height_map(
            self,
            height_file: Path,
            texture_file: Optional[Path] = None,
            scale: Vector3 = [1, 1, 1],
            center: Position = [0., 0., 0.]
        ):
        with DisableRendering(self.client), RedirectStream(sys.stdout), RedirectStream(sys.stderr):
            shape_id = self.client.createCollisionShape(
                shapeType = pb.GEOM_HEIGHTFIELD, meshScale = scale, fileName = str(height_file)
                )

            terrain = self.client.createMultiBody(baseCollisionShapeIndex = shape_id, basePosition = center)

            if texture_file:
                texture_id = self.client.loadTexture(str(texture_file))
                self.client.changeVisualShape(terrain, -1, textureUniqueId = texture_id)

            self.client.changeVisualShape(terrain, -1, rgbaColor = [1, 1, 1, 1])

    def add_environment_from_problem_dict(self, problem: Dict[str, Any], add_names: bool = True):
        with DisableRendering(self.client), RedirectStream(sys.stdout), RedirectStream(sys.stderr):
            for obj in problem['sphere']:
                self.add_sphere(
                    obj['radius'],
                    obj['position'],
                    obj['name'] if add_names and 'name' in obj else None,
                    obj['name'],
                    )

            for obj in problem['cylinder']:
                self.add_capsule(
                    obj['radius'],
                    obj['length'],
                    obj['position'],
                    obj['orientation_quat_xyzw'],
                    obj['name'] if add_names and 'name' in obj else None,
                    obj['name'],
                    )

            for obj in problem['box']:
                self.add_cuboid(
                    obj['half_extents'],
                    obj['position'],
                    obj['orientation_quat_xyzw'],
                    obj['name'] if add_names and 'name' in obj else None,
                    obj['name'],
                    )

    def draw_roadmap(self, fk_function, roadmap):
        with DisableRendering(self.client), RedirectStream(sys.stdout), RedirectStream(sys.stderr):
            for i, edge_list in enumerate(roadmap.edges):
                for edge in edge_list:
                    a = fk_function(roadmap[i].to_list())[-1]
                    b = fk_function(roadmap[edge].to_list())[-1]
                    self.client.addUserDebugLine([a.x, a.y, a.z], [b.x, b.y, b.z])

    def draw_pointcloud(self, pc, lifetime: float = 0.):
        maxes = np.max(pc, axis = 0)
        colors = 0.8 * (pc / maxes)
        with DisableRendering(self.client), RedirectStream(sys.stdout), RedirectStream(sys.stderr):
            self.client.addUserDebugPoints(pc, colors, pointSize = 3, lifeTime = lifetime)

    def clear_pointcloud(self):
        with DisableRendering(self.client), RedirectStream(sys.stdout), RedirectStream(sys.stderr):
            self.client.removeAllUserDebugItems()

    def animate(self, plan, callback = None):
        """Animate a single robot (backward compatible)."""
        if not len(plan):
            print("""Path has no states!
            """)
            return

        plan_idx = 0
        playing = False
        moved = True
        left = self.client.B3G_LEFT_ARROW
        right = self.client.B3G_RIGHT_ARROW
        space_code = ord(' ')

        print(
            """Press `space` to start/stop animation.
Use left/right arrow keys to move through individual states."""
            )

        while True:
            c = plan[plan_idx]
            if isinstance(c, list):
                c_list = c
            elif isinstance(c, np.ndarray):
                c_list = c.tolist()
            else:
                c_list = c.to_list()

            self.set_joint_positions(c_list)

            if callable(callback):
                callback(c_list)

            moved = False
            keys = self.client.getKeyboardEvents()

            if space_code in keys and keys[space_code] & self.client.KEY_WAS_TRIGGERED:
                playing = not playing

            elif not playing and left in keys and keys[left] & self.client.KEY_WAS_TRIGGERED:
                plan_idx -= 1
                if plan_idx < 0:
                    plan_idx = len(plan) - 1

                moved = True

            elif not playing and right in keys and keys[right] & self.client.KEY_WAS_TRIGGERED:
                plan_idx += 1
                if plan_idx >= len(plan):
                    plan_idx = 0

                moved = True

            elif playing:
                plan_idx = min(len(plan), plan_idx + 1)
                moved = True

            if moved:
                if plan_idx >= len(plan):
                    plan_idx = 0

            time.sleep(0.016)

    def animate_multi(self, robot_plans: Union[Dict[int, Any], List[int], Any], 
                     callbacks: Optional[Dict[int, Callable]] = None,
                     plan: Optional[Any] = None):
        """
        Animate multiple robots with the same or different plans.
        
        Args:
            robot_plans: Either a dict mapping robot_id -> plan, or a list of robot_ids with a single plan
            callbacks: Optional dict mapping robot_id -> callback function
            plan: Single plan to use for all robots (when robot_plans is a list of robot_ids)
        """
        # Handle different input formats
        if isinstance(robot_plans, list):
            # List of robot IDs with single plan
            if plan is None:
                raise ValueError("plan must be provided when robot_plans is a list of robot IDs")
            robot_configs = {robot_id: plan for robot_id in robot_plans}
        else:
            # Dict of robot_id -> plan
            robot_configs = robot_plans
        
        # Convert to RobotAnimationConfig objects
        configs = []
        for robot_id, robot_plan in robot_configs.items():
            callback = callbacks.get(robot_id) if callbacks else None
            
            # Handle main robot (skel_id) which isn't in the robots registry
            if robot_id == self.skel_id:
                robot_name = "Main Robot"
            else:
                robot_name = self.robots.get(robot_id, {}).get('name', f"Robot_{robot_id}")
            
            configs.append(RobotAnimationConfig(
                robot_id=robot_id,
                plan=robot_plan,
                callback=callback,
                name=robot_name
            ))
        
        self.animate_multi_advanced(configs)

    def animate_multi_advanced(self, configs: List[RobotAnimationConfig]):
        """
        Advanced multi-robot animation with independent control per robot.
        
        Args:
            configs: List of RobotAnimationConfig objects defining each robot's animation
        """
        if not configs:
            print("No robot configurations provided!")
            return

        # Validate all plans have states
        for config in configs:
            if not len(config.plan):
                print(f"Robot {config.robot_id} ({config.name}) has no states in plan!")
                return

        # Initialize state for each robot
        robot_states = {}
        for config in configs:
            robot_states[config.robot_id] = {
                'plan_idx': 0,
                'playing': False,
                'config': config
            }

        # Global animation state
        global_playing = False
        selected_robot = None  # None = all robots, otherwise specific robot ID
        
        # Keyboard controls
        left = self.client.B3G_LEFT_ARROW
        right = self.client.B3G_RIGHT_ARROW
        space_code = ord(' ')
        tab_code = ord('\t')
        number_keys = [ord(str(i)) for i in range(10)]  # 0-9 keys

        print("Multi-robot animation controls:")
        print("  Space: Start/stop global animation")
        print("  Tab: Select next robot for individual control (or 'all robots')")
        print("  A: Select 'all robots' mode (arrow keys step all robots together)")
        print("  0-9: Select specific robot by number")
        print("  Left/Right arrows: Step through states (when animation paused)")
        print("  R: Reset all robots to start")
        print("  Q: Quit animation")

        while True:
            # Update all robots
            for robot_id, state in robot_states.items():
                config = state['config']
                plan_idx = state['plan_idx']
                
                # Get current state
                c = config.plan[plan_idx]
                if isinstance(c, list):
                    c_list = c
                elif isinstance(c, np.ndarray):
                    c_list = c.tolist()
                else:
                    c_list = c.to_list()

                # Set joint positions
                self.set_joint_positions(c_list, robot_id)

                # Call robot-specific callback
                if callable(config.callback):
                    config.callback(c_list, robot_id)

            # Handle keyboard input
            keys = self.client.getKeyboardEvents()
            
            # Global play/pause
            if space_code in keys and keys[space_code] & self.client.KEY_WAS_TRIGGERED:
                global_playing = not global_playing
                for state in robot_states.values():
                    state['playing'] = global_playing
                print(f"Global animation: {'Playing' if global_playing else 'Paused'}")

            # Select "all robots" mode explicitly
            elif ord('a') in keys and keys[ord('a')] & self.client.KEY_WAS_TRIGGERED:
                selected_robot = None
                print("Selected: All robots (arrow keys will step all robots together)")

            # Robot selection (including "all robots")
            elif tab_code in keys and keys[tab_code] & self.client.KEY_WAS_TRIGGERED:
                robot_ids = list(robot_states.keys())
                if selected_robot is None:
                    # First tab: select first robot
                    selected_robot = robot_ids[0]
                    print(f"Selected robot: {selected_robot} ({robot_states[selected_robot]['config'].name})")
                elif selected_robot in robot_ids:
                    # Subsequent tabs: cycle through robots
                    current_idx = robot_ids.index(selected_robot)
                    next_idx = (current_idx + 1) % len(robot_ids)
                    selected_robot = robot_ids[next_idx]
                    print(f"Selected robot: {selected_robot} ({robot_states[selected_robot]['config'].name})")
                else:
                    # After cycling through all robots, select "all robots"
                    selected_robot = None
                    print("Selected: All robots (arrow keys will step all robots together)")

            # Number key selection
            elif any(key in keys and keys[key] & self.client.KEY_WAS_TRIGGERED for key in number_keys):
                for key in number_keys:
                    if key in keys and keys[key] & self.client.KEY_WAS_TRIGGERED:
                        robot_num = int(chr(key))
                        robot_ids = list(robot_states.keys())
                        if robot_num < len(robot_ids):
                            selected_robot = robot_ids[robot_num]
                            print(f"Selected robot: {selected_robot} ({robot_states[selected_robot]['config'].name})")
                        else:
                            # Number key beyond robot count: select "all robots"
                            selected_robot = None
                            print("Selected: All robots (arrow keys will step all robots together)")
                        break

            # Individual robot control (when global animation is paused)
            elif not global_playing and selected_robot is not None:
                state = robot_states[selected_robot]
                
                if left in keys and keys[left] & self.client.KEY_WAS_TRIGGERED:
                    state['plan_idx'] -= 1
                    if state['plan_idx'] < 0:
                        state['plan_idx'] = len(state['config'].plan) - 1
                    print(f"Robot {selected_robot}: State {state['plan_idx']}")

                elif right in keys and keys[right] & self.client.KEY_WAS_TRIGGERED:
                    state['plan_idx'] += 1
                    if state['plan_idx'] >= len(state['config'].plan):
                        state['plan_idx'] = 0
                    print(f"Robot {selected_robot}: State {state['plan_idx']}")

            # All robots control (when global animation is paused and selected_robot is None)
            elif not global_playing and selected_robot is None:
                if left in keys and keys[left] & self.client.KEY_WAS_TRIGGERED:
                    # Step all robots backward
                    for state in robot_states.values():
                        state['plan_idx'] -= 1
                        if state['plan_idx'] < 0:
                            state['plan_idx'] = len(state['config'].plan) - 1
                    print("All robots: Stepped backward")

                elif right in keys and keys[right] & self.client.KEY_WAS_TRIGGERED:
                    # Step all robots forward
                    for state in robot_states.values():
                        state['plan_idx'] += 1
                        if state['plan_idx'] >= len(state['config'].plan):
                            state['plan_idx'] = 0
                    print("All robots: Stepped forward")

            # Reset all robots
            elif ord('r') in keys and keys[ord('r')] & self.client.KEY_WAS_TRIGGERED:
                for state in robot_states.values():
                    state['plan_idx'] = 0
                print("Reset all robots to start")

            # Quit
            elif ord('q') in keys and keys[ord('q')] & self.client.KEY_WAS_TRIGGERED:
                print("Quitting animation")
                break

            # Advance playing robots
            if global_playing:
                for state in robot_states.values():
                    if state['playing']:
                        state['plan_idx'] += 1
                        if state['plan_idx'] >= len(state['config'].plan):
                            state['plan_idx'] = 0

            time.sleep(0.016)
