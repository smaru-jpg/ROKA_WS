from pathlib import Path
import xml.etree.ElementTree as ET

from roka_mujoco.track_description import build_track_description


ROOT = Path(__file__).resolve().parents[1]


def test_track_only_description_uses_native_cpp_backend(tmp_path):
    scene = tmp_path / "scene.xml"
    scene.write_text("<mujoco model='test'><worldbody/></mujoco>")
    description = build_track_description(
        ROOT / "description" / "pk3a_track.urdf",
        scene,
        sim_speed_factor=3.0,
        headless=True,
    )
    root = ET.fromstring(description)
    plugin = root.find("./ros2_control/hardware/plugin")
    assert plugin is not None
    assert plugin.text == "mujoco_ros2_control/MujocoSystemInterface"
    parameters = {
        item.attrib["name"]: item.text
        for item in root.findall("./ros2_control/hardware/param")
    }
    assert parameters["mujoco_model"] == str(scene.resolve())
    assert parameters["sim_speed_factor"] == "3.0"
    assert parameters["headless"] == "true"
    assert parameters["render_fps"] == "60.0"
    assert parameters["render_vsync"] == "false"
    assert parameters["window_width"] == "1600"
    assert parameters["window_height"] == "900"
    assert parameters["ui_font_scale"] == "100"
    assert not root.findall("./ros2_control/joint")
