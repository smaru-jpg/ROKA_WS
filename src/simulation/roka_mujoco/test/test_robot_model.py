import json
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

from roka_mujoco.track_description import build_track_description
from roka_mujoco.validate_robot import validate


ROOT = Path(__file__).resolve().parents[1]


def test_combined_robot_scene_is_finite_and_supported():
    result = validate(
        ROOT / "description" / "pk3a" / "models" / "pk3a_robot_scene.xml",
        ROOT
        / "description"
        / "pk3a"
        / "assets"
        / "robot"
        / "robot_metadata.json",
    )
    assert result["passed"], result["failures"]
    assert result["model"]["joints"] == 5
    assert result["model"]["actuators"] == 4
    assert result["contact"]["all_wheels_touched_track"]


def test_robot_description_exports_four_velocity_interfaces():
    description = build_track_description(
        ROOT / "description" / "roka_robot.urdf",
        ROOT / "description" / "pk3a" / "models" / "pk3a_robot_scene.xml",
        headless=True,
    )
    root = ET.fromstring(description)
    joints = root.findall("./ros2_control/joint")
    assert len(joints) == 4
    assert all(
        joint.find("./command_interface[@name='velocity']") is not None
        for joint in joints
    )
    sensors = root.findall("./ros2_control/sensor")
    assert [sensor.attrib["name"] for sensor in sensors] == ["base_imu"]
    interfaces = [
        item.attrib["name"] for item in sensors[0].findall("state_interface")
    ]
    assert interfaces == [
        "orientation.x", "orientation.y", "orientation.z", "orientation.w",
        "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
        "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z",
    ]


def test_robot_wheel_axes_follow_ros_forward_convention():
    urdf = ET.parse(ROOT / "description" / "roka_robot.urdf").getroot()
    wheel_axes = [
        joint.find("axis").attrib["xyz"]
        for joint in urdf.findall("joint")
        if joint.attrib.get("type") == "continuous"
    ]
    assert wheel_axes == ["0 1 0"] * 4

    scene = ET.parse(
        ROOT / "description" / "pk3a" / "models" / "pk3a_robot_scene.xml"
    ).getroot()
    scene_axes = [
        joint.attrib["axis"]
        for joint in scene.findall(".//joint")
        if joint.attrib.get("name", "").endswith("_wheel_joint")
    ]
    assert scene_axes == ["0 1 0"] * 4


def test_sensor_frames_match_planned_hardware_layout():
    config = json.loads((ROOT / "config" / "roka_robot.json").read_text())
    urdf = ET.parse(ROOT / "description" / "roka_robot.urdf").getroot()
    joints = {joint.attrib["name"]: joint for joint in urdf.findall("joint")}

    velodyne_xyz = [
        float(value)
        for value in joints["base_to_velodyne_joint"].find("origin").attrib["xyz"].split()
    ]
    imu_xyz = [
        float(value)
        for value in joints["base_to_imu_joint"].find("origin").attrib["xyz"].split()
    ]
    assert velodyne_xyz == config["sensors"]["velodyne"]["data_origin_position_m"]
    assert imu_xyz == config["sensors"]["imu"]["position_m"]

    scene = ET.parse(
        ROOT / "description" / "pk3a" / "models" / "pk3a_robot_scene.xml"
    ).getroot()
    assert imu_xyz[:2] == velodyne_xyz[:2]
    assert abs((imu_xyz[2] - velodyne_xyz[2]) - 0.0943) < 1.0e-9
    assert (
        scene.find(".//site[@name='base_imu_site']").attrib["pos"]
        == "0.1815 0 0.206625"
    )


def test_driver_scene_has_only_the_body_mounted_camera():
    scene = ET.parse(
        ROOT
        / "description"
        / "pk3a"
        / "models"
        / "pk3a_robot_drive_scene.xml"
    ).getroot()
    cameras = scene.findall(".//camera")
    assert [camera.attrib["name"] for camera in cameras] == ["driver_camera"]
    camera = cameras[0]
    assert camera.attrib["pos"] == "0.53568 -0.00011 0.43009"
    assert camera.attrib["xyaxes"] == "0 -1 0 0 0 1"
    assert camera.attrib["resolution"] == "640 360"
    assert camera.attrib["fovy"] == "58"
    global_visual = scene.find("./visual/global")
    assert global_visual.attrib["offwidth"] == "640"
    assert global_visual.attrib["offheight"] == "360"


def test_fast_xbox_profile_is_staged_and_separate():
    controller = yaml.safe_load(
        (ROOT / "config" / "mujoco_xbox_fast_controllers.yaml").read_text()
    )["sim_diff_drive_controller"]["ros__parameters"]
    teleop = yaml.safe_load(
        (ROOT / "config" / "mujoco_xbox_fast_teleop.yaml").read_text()
    )["/**"]["ros__parameters"]

    assert controller["linear.x.max_velocity"] == 0.55
    assert controller["angular.z.max_velocity"] == 1.0
    assert teleop["scale_linear"]["x"] == 0.40
    assert teleop["scale_linear_turbo"]["x"] == 0.55
    assert teleop["enable_button"] == 4
    assert teleop["enable_turbo_button"] == 5


def test_fast_xbox_demonstration_uses_fast_profiles():
    base = (
        ROOT / "launch" / "pk3a_xbox_demonstration.launch.py"
    ).read_text()
    fast = (
        ROOT / "launch" / "pk3a_xbox_fast_demonstration.launch.py"
    ).read_text()

    assert '"controller_config_file"' in base
    assert '"teleop_config_file"' in base
    assert "pk3a_xbox_demonstration.launch.py" in fast
    assert "mujoco_xbox_fast_controllers.yaml" in fast
    assert "mujoco_xbox_fast_teleop.yaml" in fast
    assert 'default_value="fast_lap"' in fast


def test_robot_metadata_preserves_step_inventory_and_face_partition():
    metadata = json.loads((
        ROOT
        / "description"
        / "pk3a"
        / "assets"
        / "robot"
        / "robot_metadata.json"
    ).read_text())
    assert metadata["step_inventory"]["millimetre_units_declared"]
    assert metadata["step_inventory"]["assembly_occurrence_count"] == 98
    assert metadata["exported_visual_face_count"] == 339572
    assert len(metadata["wheels"]) == 4
