from pathlib import Path

import mujoco
import numpy as np

from roka_mujoco.closed_loop_pipeline import generate


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "pk3a_closed_loop.json"


def test_closed_loop_counts_and_signed_curve_groups(tmp_path):
    result = generate(CONFIG, tmp_path)
    assert result["track_role"] == "base_track"
    assert result["start_zone"]["line_instance_id"] == "S00_DB1"
    assert result["start_zone"]["instance_ids"] == ["S00_DB1"]
    assert result["instance_count"] == 64
    assert result["seam_count"] == 64
    assert result["module_counts"] == {
        "3DB1": 2,
        "3BA1": 12,
        "3TA1L": 8,
        "3TB1L": 8,
        "3CV1": 32,
        "F_HALF": 1,
        "F_CLOSE": 1,
    }
    assert np.allclose(
        result["cv_group_turns_deg"],
        [-202.42475862, 61.77264452, -149.07312171, -70.27476425],
        atol=1.0e-5,
    )
    assert np.isclose(result["cv_boundary_turn_deg"], -360.0, atol=1.0e-5)

    variants = result["module_variant_counts"]
    assert variants["F_HALF"] == 1
    assert variants["3CV1_G1"] == 14
    assert variants["3CV1_G2"] == 4
    assert variants["3CV1_G3"] == 9
    assert variants["3CV1_G4"] == 5
    assert result["surface_material"]["profile"] == (
        "highonesys_pk3a_dry_composite_antislip"
    )
    assert result["surface_contact"]["friction"] == [1.1, 0.01, 0.001]
    assert result["surface_contact"]["condim"] == 6
    assert result["contact_solver"]["cone"] == "elliptic"

    model = mujoco.MjModel.from_xml_path(result["model"])
    track_ids = [
        model.geom(index).id
        for index in range(model.ngeom)
        if model.geom(index).name.startswith("track_")
    ]
    assert track_ids
    assert np.all(model.geom_condim[track_ids] == 6)
    assert np.all(model.geom_priority[track_ids] == 1)
    assert np.allclose(model.geom_friction[track_ids], [1.1, 0.01, 0.001])
    assert model.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC
    assert model.opt.solver == mujoco.mjtSolver.mjSOL_NEWTON


def test_closed_loop_has_no_steps_perpendicular_joints_or_overlap(tmp_path):
    result = generate(CONFIG, tmp_path)
    assert result["maximum_adjacent_axis_angle_deg"] <= 17.0
    assert result["maximum_analytic_seam_height_error_m"] < 0.01
    assert result["maximum_analytic_seam_plan_error_m"] < 0.005
    assert result["maximum_course_tangent_error_deg"] < 0.05
    assert result["module_slope_metrics"]["3TA1L_C"][
        "maximum_longitudinal_slope_deg"
    ] < 21.0
    assert result["module_slope_metrics"]["3TB1L_C"][
        "maximum_longitudinal_slope_deg"
    ] < 14.5
    assert result["nonadjacent_overlap_count"] == 0


def test_closed_loop_pose_residual_is_sub_millimetre(tmp_path):
    result = generate(CONFIG, tmp_path)
    assert result["closed_loop"]
    assert result["closure"]["position_error_m"] < 0.001
    assert result["closure"]["yaw_error_deg"] < 0.01
    assert result["closure"]["maximum_plan_error_m"] < 0.001
