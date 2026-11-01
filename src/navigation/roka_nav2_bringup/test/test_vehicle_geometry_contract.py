import ast
import json
from pathlib import Path

import pytest
import yaml

from roka_robot_description.hardware_config import load_hardware_config


PACKAGE = Path(__file__).parents[1]
SRC = Path(__file__).parents[3]


def test_nav_terrain_and_optional_mujoco_use_the_same_physical_geometry():
    hardware = load_hardware_config(
        SRC / "common/roka_robot_description/config/pk3a_hardware.yaml"
    )
    nav = yaml.safe_load((PACKAGE / "config/lidar_md_nav2.yaml").read_text())
    footprint = ast.literal_eval(
        nav["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
    )
    assert tuple(value for point in footprint for value in point) == pytest.approx(
        tuple(value for point in hardware.navigation_footprint for value in point)
    )

    terrain = yaml.safe_load((
        SRC / "navigation/roka_terrain_supervisor/config/terrain_supervisor.yaml"
    ).read_text())
    supervisor = terrain["terrain_supervisor"]["ros__parameters"]
    contact = terrain["wheel_contact_estimator"]["ros__parameters"]
    assert supervisor["wheel_track"] == pytest.approx(hardware.physical_wheel_track_m)
    assert supervisor["footprint_width"] == pytest.approx(hardware.navigation_width_m)
    assert supervisor["footprint_length"] == pytest.approx(hardware.navigation_length_m)
    assert contact["wheel_track"] == pytest.approx(hardware.physical_wheel_track_m)
    assert contact["wheel_radius"] == pytest.approx(hardware.wheel_radius_m)

    md = yaml.safe_load((
        SRC / "drivers/roka_md_driver/config/mdh250_md200t_four_wheel.yaml"
    ).read_text())["roka_md_driver"]["ros__parameters"]
    assert md["wheel_separation"] == pytest.approx(
        hardware.effective_drive_separation_m
    )

    # The real-robot handoff intentionally omits src/simulation.  Validate the
    # personal MuJoCo copy when present without making the field-only bundle
    # depend on simulator assets.
    simulation_root = SRC / "simulation/roka_mujoco"
    if not simulation_root.is_dir():
        return

    simulation = json.loads((simulation_root / "config/roka_robot.json").read_text())
    metadata = json.loads((
        simulation_root / "description/pk3a/assets/robot/robot_metadata.json"
    ).read_text())
    wheels = metadata["wheels"]
    wheelbase = wheels["front_left"]["position_m"][0] - wheels["rear_left"]["position_m"][0]
    wheel_track = wheels["front_left"]["position_m"][1] - wheels["front_right"]["position_m"][1]
    assert simulation["wheel_physics"]["radius_m"] == pytest.approx(hardware.wheel_radius_m)
    assert simulation["wheel_physics"]["half_width_m"] == pytest.approx(hardware.tire_half_width_m)
    assert wheelbase == pytest.approx(hardware.wheelbase_m, abs=1.0e-8)
    assert wheel_track == pytest.approx(hardware.physical_wheel_track_m, abs=1.0e-8)

    sim_controller = yaml.safe_load((
        simulation_root / "config/mujoco_safe_controllers.yaml"
    ).read_text())["sim_diff_drive_controller"]["ros__parameters"]
    assert sim_controller["wheel_separation"] == pytest.approx(
        hardware.physical_wheel_track_m
    )


def test_effective_drive_separation_is_deliberately_not_physical_track():
    hardware = load_hardware_config(
        SRC / "common/roka_robot_description/config/pk3a_hardware.yaml"
    )
    assert hardware.effective_drive_separation_m > hardware.physical_wheel_track_m
    assert hardware.effective_drive_separation_status == "section5_multibag_flat_calibrated"
