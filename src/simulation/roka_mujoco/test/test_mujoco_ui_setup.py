import sys

from roka_mujoco.mujoco_ui_setup import parse_arguments


def test_ui_setup_parses_capstone_panel_contract(monkeypatch):
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "mujoco_ui_setup",
            "--width",
            "1600",
            "--height",
            "900",
            "--left",
            "--right",
            "--joint",
            "--profiler",
            "--sensor",
            "--ros-args",
        ],
    )
    arguments = parse_arguments()
    assert (arguments.width, arguments.height) == (1600, 900)
    assert arguments.left
    assert arguments.right
    assert arguments.joint
    assert arguments.profiler
    assert arguments.sensor


def test_ui_setup_defaults_to_no_panels(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["mujoco_ui_setup"])
    arguments = parse_arguments()
    assert arguments.title == "MuJoCo :"
    assert not arguments.left
    assert not arguments.right
    assert not arguments.profiler
    assert not arguments.sensor
