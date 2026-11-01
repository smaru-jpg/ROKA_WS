from pathlib import Path
import math

from roka_mujoco.dynamic_safety_envelope import derive_envelope, ros_parameters
from roka_robot_description.workspace import workspace_root


WORKSPACE = workspace_root()


def test_mujoco_runs_produce_separate_roll_and_pitch_guards():
    envelope = derive_envelope(
        WORKSPACE / "datasets/xbox_demonstrations",
        WORKSPACE
        / "src/simulation/roka_mujoco/description/pk3a/models"
        / "pk3a_mapped_flat_scene.xml",
    )
    limits = envelope["limits"]
    geometry = envelope["model_geometry"]
    assert limits["roll_warning_deg"] < limits["roll_critical_deg"]
    assert limits["roll_critical_deg"] < limits["roll_hard_deg"]
    assert limits["pitch_warning_deg"] < limits["pitch_critical_deg"]
    assert limits["pitch_critical_deg"] < limits["pitch_hard_deg"]
    assert envelope["model_geometry"]["static_rollover_deg"] > limits["roll_hard_deg"]
    assert (
        envelope["model_geometry"]["static_forward_pitchover_deg"]
        > limits["pitch_hard_deg"]
    )
    assert limits["roll_hard_deg"] == (
        math.floor(geometry["static_rollover_deg"] * 0.90 * 10.0) / 10.0
    )
    static_pitch = min(
        geometry["static_forward_pitchover_deg"],
        geometry["static_reverse_pitchover_deg"],
    )
    assert limits["pitch_hard_deg"] == (
        math.floor(static_pitch * 0.90 * 10.0) / 10.0
    )
    assert any(item["completed"] for item in envelope["inputs"])
    assert any(not item["completed"] for item in envelope["inputs"])
    validation = envelope["replay_validation"]
    assert validation["successful_roll_hard_samples"] == 0
    assert validation["successful_pitch_hard_samples"] == 0
    assert validation["successful_false_airborne_samples"] == 0
    assert validation["roll_guard_lead_time_sec"] >= limits["attitude_hard_hold_sec"]
    assert validation["pass"] is True


def test_generated_overlay_targets_both_runtime_supervisors():
    envelope = derive_envelope(
        WORKSPACE / "datasets/xbox_demonstrations",
        WORKSPACE
        / "src/simulation/roka_mujoco/description/pk3a/models"
        / "pk3a_mapped_flat_scene.xml",
    )
    parameters = ros_parameters(envelope)
    assert parameters["terrain_supervisor"]["ros__parameters"]["roll_hard_deg"] > 0
    assert parameters["mapping_terrain_monitor"]["ros__parameters"]["pitch_hard_deg"] > 0
    assert parameters["wheel_contact_estimator"]["ros__parameters"][
        "airborne_accel_norm_max_mps2"
    ] == 3.0
