from pathlib import Path

from roka_mission.section_manual_joy_router import normalize_section, rising_edge


def test_section_names_are_normalized_and_bounded():
    assert normalize_section("section_1") == "section1"
    assert normalize_section("SECTION-5") == "section5"


def test_face_buttons_use_rising_edges_only():
    assert rising_edge([0, 0, 0, 0], [1, 0, 0, 0], 0)
    assert not rising_edge([1, 0, 0, 0], [1, 0, 0, 0], 0)
    assert not rising_edge([0, 0, 0, 0], [0, 0, 0, 0], 0)


def test_section_manual_launch_has_one_camera_path_and_full_arm_controller():
    package = Path(__file__).resolve().parents[1]
    text = (package / "launch" / "section_manual.launch.py").read_text(
        encoding="utf-8"
    )
    assert '"roka_arm.launch.py"' in text
    assert '"manual_joy_enabled": enable_arm_motion' in text
    assert '"spring.launch.py"' in text
    assert '"autumn.launch.py"' in text
    assert '"camera_view.launch.py"' in text
    assert '"traffic.launch.py"' in text
    assert '"operator_trigger_required": True' in text
    assert '"control_recorded_route": False' in text
    assert '"require_enable_release": True' in text


def test_operator_commands_are_exposed_by_the_roka_launcher():
    launcher = Path(__file__).resolve().parents[4] / "roka"
    text = launcher.read_text(encoding="utf-8")
    assert "roka drive pro-hold" in text
    for number in range(1, 6):
        assert f"roka manual section{number}" in text
