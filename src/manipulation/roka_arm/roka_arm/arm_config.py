"""
Validated active constants from the 2026-08-28 ROKA arm program.

Only deployment-specific values are overridden from ROS parameters. Motion
limits, poses, and servo increments retain the values used by the split
``arm_control_node.py`` implementation, not the older monolithic copy.
"""

DEVICENAME_ARM = (
    "/dev/serial/by-id/"
    "usb-FTDI_USB__-__Serial_Converter_FT89FBMQ-if00-port0"
)
ARM_BAUDRATE = 1_000_000
DEVICENAME_WRIST = (
    "/dev/serial/by-id/"
    "usb-FTDI_USB__-__Serial_Converter_FT78LQH1-if00-port0"
)
WRIST_BAUDRATE = 57_600
PROTOCOL_VERSION = 2.0

CONNECT_HARDWARE = True
ENABLE_MOTION = True
TORQUE_OFF_ON_EXIT = True
VISUALIZE = True
INFERENCE_DEVICE = "auto"
INFERENCE_RATE_HZ = 10.0
MAXIMUM_DEPTH_AGE_SEC = 0.12
TARGET_CLASS_IDS = (0, 1)
ACTIVATION_MIN_Y_RATIO = 0.25

JOINT_1_ID = 11
JOINT_2_ID = 12
JOINT_3_ID = 13
WRIST_FORWARD_ID = 4
WRIST_REVERSE_ID = 6
GRIPPER_ID = 1

H54_ADDR_TORQUE_ENABLE = 562
H54_ADDR_GOAL_POSITION = 596
H54_ADDR_PRESENT_POSITION = 611
H54_ADDR_OPERATING_MODE = 11
H54_ADDR_GOAL_VELOCITY = 600
H54_ADDR_GOAL_ACCELERATION = 606
H54_EXTENDED_POSITION_MODE = 4
H54_GOAL_VELOCITY = 1000
H54_GOAL_ACCELERATION = 1

XM_ADDR_TORQUE_ENABLE = 64
XM_ADDR_OPERATING_MODE = 11
XM_ADDR_PROFILE_ACCELERATION = 108
XM_ADDR_PROFILE_VELOCITY = 112
XM_ADDR_GOAL_POSITION = 116
XM_ADDR_PRESENT_POSITION = 132
XM_ADDR_PRESENT_CURRENT = 126
CURRENT_UNIT_MA = 2.69
XM_POSITION_MODE = 3
TORQUE_OFF = 0
TORQUE_ON = 1

WRIST_PROFILE_ACCELERATION = 10
WRIST_PROFILE_VELOCITY = 20
WRIST_MIN = 800
WRIST_MAX = 3100
WRIST_START_POSITION = 1100

GRIPPER_MIN = 0
GRIPPER_MAX = 4095
GRIPPER_OPEN_POSITION = 1400
GRIPPER_CLOSE_POSITION = 2300
GRIPPER_PROFILE_ACCELERATION = 10
GRIPPER_PROFILE_VELOCITY = 20

ARM_POSITION_LIMITS = {
    11: (-10000, 155000),
    12: None,
    13: (-3000, 220000),
}
#   START_POSE = {11: -7500, 12: 0, 13: 162500}
START_POSE = {11: 0, 12: 0, 13: 125480}

TARGET_X = 355
TARGET_Y = 360
DEAD_ZONE_X = 20
DEAD_ZONE_Y = 20
CONTROL_PERIOD = 0.05
CAMERA_TOPIC = "/roka/arm_camera/color/image_raw"
DEPTH_TOPIC = "/roka/arm_camera/aligned_depth_to_color/image_raw"
YOLO_MODEL_PATH = ""
YOLO_MODEL_SHA256 = (
    "059e5c485f5162659ae76b6492d7a8291bbcbce6ab75d1dbbf555f6e9f7acc95"
)
YOLO_CONFIDENCE = 0.50

X_JOINT_ID = JOINT_2_ID
X_DELTA = 500
X_RIGHT_ERROR_DELTA_SIGN = -1
X_COMMAND_INTERVAL = 0.15
Y_DELTA = 15
Y_DOWN_ERROR_DELTA_SIGN = -1
Y_COMMAND_INTERVAL = 0.25
Z11_DELTA = -500
Z13_DELTA = -1000
Z_COMMAND_INTERVAL = 0.15
JOINT_11_ZERO_TOLERANCE = 10
TARGET_DEPTH_MM = 250
DEPTH_SAMPLE_RADIUS = 5
AUTO_CONTROL_START = False
WINDOW_NAME = "ROKA Arm XYZ Visual Servoing"

READY_STABLE_FRAMES = 5
GRASP_OFFSET_11 = 32100
GRASP_OFFSET_12 = 0
GRASP_OFFSET_13 = -60000
GRASP_OFFSET_WRIST = 150
GRIP_HOLD_TIME = 3.0
GRASP_CURRENT_THRESHOLD = 300.0
GRASP_VERIFY_DELAY = 0.5
GRASP_CURRENT_SAMPLES = 10
GRASP_CURRENT_SAMPLE_INTERVAL = 0.05
MAX_GRASP_RETRY = 3
RETRY_WRIST_POSITION = 1100
H54_POSITION_TOLERANCE = 1000
WRIST_POSITION_TOLERANCE = 20
GRIPPER_POSITION_TOLERANCE = 20
POSE_REACHED_TIMEOUT = 30.0
POSE_CHECK_PERIOD = 0.1

HOME2_POSE = {
    11: -7500,
    12: 0,
    13: 162500,
    "wrist": 1600,
    "gripper": 2300,
}

PLACE_POSE = {
    11: -5000,
    12: -125480,
    13: 120000,
    "wrist": 1200,
    "gripper": 2300,
}

DROP_POSE = {
#    11: -5000,
#    12: -125480,
#    13: 140000,
    11: -9500,
    12: -125480,
    13: 120000,
    "wrist": 1100,
    "gripper": 1400,
}

DEFAULT_SECTION = 2

SECTION1_POSE = { #봄
    11: -7500,
    12: 0,
    13: 210000,
    "wrist": 1500,
    "gripper": 1400,
}

SECTION2_START_POSE = {
    11: 0,
    12: 0,
    13: 125480,
    "wrist": 1100,
    "gripper": 1400,
}

SECTION2_DRIVE_POSE = {
    11: -2000,
    12: 0,
    13: 150000,
    "wrist": 2200,
    "gripper": 2300,
}

SECTION3_POSE = {
    11: 80000,
    12: 0,
    13: 170000,
    "wrist": 1200,
    "gripper": 1400,
}

SECTION4_POSE = {
    11: -7500,
    12: 0,
    13: 210000,
    "wrist": 1500,
    "gripper": 1400,
}

SECTION5_POSE = { #봄
    11: -7500,
    12: 0,
    13: 210000,
    "wrist": 1500,
    "gripper": 1400,
}


_NAMED_START_POSES = {
    "section1": SECTION1_POSE,
    "section2": SECTION2_START_POSE,
    "section2_start": SECTION2_START_POSE,
    "section3": SECTION3_POSE,
    "section4": SECTION4_POSE,
    "section5": SECTION5_POSE,
}


def get_named_start_pose(name):
    """Return a validated section start pose without mutating configuration."""
    normalized = str(name).strip().lower().replace("-", "_")
    if normalized in {"", "auto", "default", "transport", "start"}:
        normalized = f"section{DEFAULT_SECTION}"
    if normalized.startswith("section_"):
        normalized = "section" + normalized[len("section_"):]
    try:
        pose = dict(_NAMED_START_POSES[normalized])
    except KeyError as error:
        choices = ", ".join(sorted(_NAMED_START_POSES))
        raise ValueError(
            f"unknown arm start pose {name!r}; expected one of: {choices}"
        ) from error
    required = {JOINT_1_ID, JOINT_2_ID, JOINT_3_ID, "wrist", "gripper"}
    if set(pose) != required:
        raise ValueError(f"{normalized} must define exactly {sorted(map(str, required))}")
    for motor_id in (JOINT_1_ID, JOINT_2_ID, JOINT_3_ID):
        limit = ARM_POSITION_LIMITS[motor_id]
        if limit is not None and not limit[0] <= int(pose[motor_id]) <= limit[1]:
            raise ValueError(f"{normalized} ID {motor_id} is outside {limit}")
    if not WRIST_MIN <= int(pose["wrist"]) <= WRIST_MAX:
        raise ValueError(f"{normalized} wrist is outside {WRIST_MIN}..{WRIST_MAX}")
    if not GRIPPER_MIN <= int(pose["gripper"]) <= GRIPPER_MAX:
        raise ValueError(
            f"{normalized} gripper is outside {GRIPPER_MIN}..{GRIPPER_MAX}"
        )
    return pose


def activate_start_pose(name):
    """Select the section pose used by the existing driver startup path."""
    global START_POSE, WRIST_START_POSITION, GRIPPER_OPEN_POSITION
    pose = get_named_start_pose(name)
    START_POSE = {
        JOINT_1_ID: pose[JOINT_1_ID],
        JOINT_2_ID: pose[JOINT_2_ID],
        JOINT_3_ID: pose[JOINT_3_ID],
    }
    WRIST_START_POSITION = pose["wrist"]
    GRIPPER_OPEN_POSITION = pose["gripper"]
    return dict(pose)


def apply_runtime_overrides(values):
    """Apply the explicit deployment overrides read from ROS parameters."""
    global DEVICENAME_ARM, DEVICENAME_WRIST
    global CONNECT_HARDWARE, ENABLE_MOTION, TORQUE_OFF_ON_EXIT
    global CAMERA_TOPIC, DEPTH_TOPIC, YOLO_MODEL_PATH
    global VISUALIZE, INFERENCE_DEVICE, INFERENCE_RATE_HZ
    global MAXIMUM_DEPTH_AGE_SEC, TARGET_CLASS_IDS, AUTO_CONTROL_START
    global ACTIVATION_MIN_Y_RATIO
    global TARGET_X, TARGET_Y, TARGET_DEPTH_MM, Y_DELTA
    global X_COMMAND_INTERVAL, Y_COMMAND_INTERVAL, Z_COMMAND_INTERVAL

    connect_hardware = bool(values["connect_hardware"])
    enable_motion = bool(values["enable_motion"])
    inference_rate_hz = float(values["inference_rate_hz"])
    maximum_depth_age_sec = float(values["maximum_depth_age_sec"])
    target_class_ids = tuple(int(x) for x in values["target_class_ids"])
    activation_min_y_ratio = float(values["activation_min_y_ratio"])
    if enable_motion and not connect_hardware:
        raise ValueError("enable_motion=true requires connect_hardware=true")
    if inference_rate_hz <= 0.0:
        raise ValueError("inference_rate_hz must be positive")
    if maximum_depth_age_sec <= 0.0:
        raise ValueError("maximum_depth_age_sec must be positive")
    if not target_class_ids:
        raise ValueError("target_class_ids must not be empty")
    if not 0.0 <= activation_min_y_ratio < 1.0:
        raise ValueError("activation_min_y_ratio must be in [0.0, 1.0)")

    DEVICENAME_ARM = str(values["arm_serial_port"])
    DEVICENAME_WRIST = str(values["wrist_serial_port"])
    CONNECT_HARDWARE = connect_hardware
    ENABLE_MOTION = enable_motion
    TORQUE_OFF_ON_EXIT = bool(values["torque_off_on_exit"])
    CAMERA_TOPIC = str(values["color_topic"])
    DEPTH_TOPIC = str(values["depth_topic"])
    YOLO_MODEL_PATH = str(values["model_path"])
    VISUALIZE = bool(values["visualize"])
    INFERENCE_DEVICE = str(values["inference_device"])
    INFERENCE_RATE_HZ = inference_rate_hz
    MAXIMUM_DEPTH_AGE_SEC = maximum_depth_age_sec
    TARGET_CLASS_IDS = target_class_ids
    ACTIVATION_MIN_Y_RATIO = activation_min_y_ratio
    AUTO_CONTROL_START = bool(values["auto_control_start"])
    TARGET_X = int(values["target_x"])
    TARGET_Y = int(values["target_y"])
    TARGET_DEPTH_MM = int(values["target_depth_mm"])
    Y_DELTA = int(values["y_delta"])
    X_COMMAND_INTERVAL = float(values["x_command_interval"])
    Y_COMMAND_INTERVAL = float(values["y_command_interval"])
    Z_COMMAND_INTERVAL = float(values["z_command_interval"])
