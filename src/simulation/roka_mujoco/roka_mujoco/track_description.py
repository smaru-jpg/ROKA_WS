"""Build the track-only ros2_control description for the native C++ runtime."""

from pathlib import Path
import xml.etree.ElementTree as ET


def build_track_description(
    source_urdf: Path,
    mujoco_scene: Path,
    *,
    sim_speed_factor: float = 1.0,
    headless: bool = False,
    render_fps: float = 60.0,
    render_vsync: bool = False,
    window_width: int = 1600,
    window_height: int = 900,
    ui_font_scale: int = 100,
    camera_publish_rate: float = 5.0,
) -> str:
    """Attach MuJoCoSystemInterface and interfaces for movable URDF joints."""
    source = Path(source_urdf)
    scene = Path(mujoco_scene)
    if not source.is_file():
        raise FileNotFoundError(source)
    if not scene.is_file():
        raise FileNotFoundError(scene)
    if camera_publish_rate <= 0.0:
        raise ValueError("camera_publish_rate must be positive")
    if sim_speed_factor <= 0.0:
        raise ValueError("sim_speed_factor must be positive")

    root = ET.parse(source).getroot()
    if root.tag != "robot":
        raise ValueError(f"Expected a robot root element, got {root.tag!r}")
    if root.find("ros2_control") is not None:
        raise ValueError("Track URDF already contains a ros2_control element")

    control = ET.SubElement(root, "ros2_control", {
        "name": f'{root.attrib.get("name", "Mujoco")}System',
        "type": "system",
    })
    hardware = ET.SubElement(control, "hardware")
    ET.SubElement(hardware, "plugin").text = (
        "mujoco_ros2_control/MujocoSystemInterface"
    )
    parameters = {
        "mujoco_model": str(scene.resolve()),
        "sim_speed_factor": str(float(sim_speed_factor)),
        "headless": str(bool(headless)).lower(),
        "render_fps": str(float(render_fps)),
        "render_vsync": str(bool(render_vsync)).lower(),
        "window_width": str(int(window_width)),
        "window_height": str(int(window_height)),
        "ui_font_scale": str(int(ui_font_scale)),
        "camera_publish_rate": str(float(camera_publish_rate)),
    }
    for name, value in parameters.items():
        ET.SubElement(hardware, "param", {"name": name}).text = value

    for joint in root.findall("joint"):
        if joint.attrib.get("type") == "fixed":
            continue
        joint_name = joint.attrib.get("name")
        if not joint_name:
            raise ValueError("Every movable URDF joint must have a name")
        control_joint = ET.SubElement(control, "joint", {"name": joint_name})
        command = ET.SubElement(
            control_joint, "command_interface", {"name": "velocity"}
        )
        limit = joint.find("limit")
        if limit is not None and "velocity" in limit.attrib:
            maximum = abs(float(limit.attrib["velocity"]))
            ET.SubElement(command, "param", {"name": "min"}).text = str(-maximum)
            ET.SubElement(command, "param", {"name": "max"}).text = str(maximum)
            ET.SubElement(command, "param", {"name": "initial_value"}).text = "0.0"
        for interface_name in ("position", "velocity", "effort"):
            ET.SubElement(
                control_joint,
                "state_interface",
                {"name": interface_name},
            )

    # The robot MJCF already contains an IMU at the measured imu_link mount.
    # Export it through ros2_control only for the robot description; the
    # track-only shell intentionally has no sensors.
    if root.find("./link[@name='imu_link']") is not None:
        sensor = ET.SubElement(control, "sensor", {"name": "base_imu"})
        ET.SubElement(sensor, "param", {"name": "mujoco_type"}).text = "imu"
        ET.SubElement(sensor, "param", {"name": "mujoco_sensor_name"}).text = (
            "base_imu"
        )
        for interface_name in (
            "orientation.x",
            "orientation.y",
            "orientation.z",
            "orientation.w",
            "angular_velocity.x",
            "angular_velocity.y",
            "angular_velocity.z",
            "linear_acceleration.x",
            "linear_acceleration.y",
            "linear_acceleration.z",
        ):
            ET.SubElement(sensor, "state_interface", {"name": interface_name})
    return ET.tostring(root, encoding="unicode")
