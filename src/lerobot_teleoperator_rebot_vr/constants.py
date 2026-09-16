"""Shared B601 names and fixed safety bounds, independent of control modules."""

ARM_JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
)
GRIPPER_NAME = "gripper"
JOINT_NAMES = (*ARM_JOINT_NAMES, GRIPPER_NAME)

# LeRobot follower software limits; distinct from URDF and feedback margins.
FOLLOWER_LOWER_DEG = (-150.0, -200.0, -200.0, -80.0, -90.0, -90.0)
FOLLOWER_UPPER_DEG = (150.0, 1.0, 1.0, 90.0, 90.0, 90.0)

# Fixed B601 feedforward safety ceiling, matching the packaged URDF efforts.
# Custom dynamics URDFs and configurable torque limits do not raise this ceiling.
ARM_EFFORT_LIMIT_NM = (27.0, 27.0, 27.0, 7.0, 7.0, 7.0)
