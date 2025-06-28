DEFAULT_ITERATIONS = 1000000

ROBOT_JOINTS = {
    "ur5": [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
        ],
    "panda": [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
        ],
    "fetch": [
        "torso_lift_joint",
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "upperarm_roll_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint",
        ],
    "baxter": [
        "right_s0",
        "right_s1",
        "right_e0",
        "right_e1",
        "right_w0",
        "right_w1",
        "right_w2",
        "left_s0",
        "left_s1",
        "left_e0",
        "left_e1",
        "left_w0",
        "left_w1",
        "left_w2",
        ],
    }

ROBOT_RRT_RANGES = {
    "sphere": 1,
    "ur5": 1.5,
    "panda": 1.0,
    "fetch": 1.0,
    "baxter": 0.5,
    }

ROBOT_RADII_RANGES = {
    "baxter": (0.012, 0.08),
    "fetch": (0.012, 0.055),
    "panda": (0.012, 0.06),
    "sphere": (0.2, 0.2),
    "ur5": (0.015, 0.08),
    }

ROBOT_FIRST_JOINT_LOCATIONS = {
    "fetch": [0.0, 0.0, 0.424],
    "ur5": [0.0, 0.0, 0.089159],
    "panda": [0.0, 0.0, 0.05],
    "baxter": [0.0, 0.0, 0.0],
    }

ROBOT_MAX_RADII = {
    "ur5": 1.2,
    "fetch": 1.5,
    "panda": 1.19,
    }

# Multi-robot grid variants for Panda
# Note: These represent the actual base positions in meters, not the template parameters
# The C++ template parameters are in centimeters * 100 for compiler compatibility
PANDA_GRID_VARIANTS = {
    "panda_0_0": {"base_x": 0.0, "base_y": 0.0, "base_z": 0.05},
    "panda_0_1": {"base_x": 0.0, "base_y": 1.0, "base_z": 0.05},
    "panda_0_2": {"base_x": 0.0, "base_y": 2.0, "base_z": 0.05},
    "panda_1_0": {"base_x": 1.0, "base_y": 0.0, "base_z": 0.05},
    "panda_1_1": {"base_x": 1.0, "base_y": 1.0, "base_z": 0.05},
    "panda_1_2": {"base_x": 1.0, "base_y": 2.0, "base_z": 0.05},
    "panda_2_0": {"base_x": 2.0, "base_y": 0.0, "base_z": 0.05},
    "panda_2_1": {"base_x": 2.0, "base_y": 1.0, "base_z": 0.05},
    "panda_2_2": {"base_x": 2.0, "base_y": 2.0, "base_z": 0.05},
}

POINT_RADIUS = 0.0025
