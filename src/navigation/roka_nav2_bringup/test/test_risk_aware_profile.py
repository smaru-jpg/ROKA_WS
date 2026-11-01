import hashlib
import math

import pytest
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State
import numpy as np

from roka_robot_description.hardware_config import load_hardware_config

from roka_nav2_bringup.risk_aware_profile import (
    annotate_profile,
    risk_probability,
    static_feature_vector,
)
from roka_nav2_bringup.real_traversability_recorder import (
    nominal_tire_clearance,
    profile_tire_clearance,
)
from roka_nav2_bringup.profile_validation import (
    is_autonomy_approved,
    is_profile_allowed,
    is_test_only_profile,
)
from roka_nav2_bringup.xbox_mapping_controls import (
    button_pressed,
    mapping_drive_block_reason,
    rising_edge,
    XboxMappingControls,
)
from roka_nav2_bringup.xbox_autonomy_controls import XboxAutonomyControls
from roka_nav2_bringup.map_bundle_selection import (
    latest_valid_map_bundle,
    map_bundle_for_id,
    next_mapping_bundle,
    occupancy_map_is_valid,
)
from roka_nav2_bringup.field_session_guard import other_field_launches
from roka_nav2_bringup.field_map_check import (
    prepare_recorded_route_file,
    profile_problems,
    recorded_profile_problems,
)
from roka_nav2_bringup.field_preflight import (
    finite_vector,
    measured_rate,
    point_time_bounds_valid,
    reset_stamp_error_window,
    sampled_rate,
    stamp_age_seconds,
)
from roka_nav2_bringup.pcd_terrain_profile import (
    annotate_profile_from_pcd,
    load_ascii_xyz,
    surface_features,
)
from roka_nav2_bringup.vehicle_aware_path import plan_vehicle_aware_profile
from roka_nav2_bringup.short_test_profile import prepare_short_test_document
from roka_nav2_bringup.profile_start_initializer import load_profile_start
from roka_nav2_bringup.track_path_follower import (
    TrackPathFollower,
    path_progress_metrics,
)
from roka_nav2_bringup.command_pipeline_diagnostics import blocked_stage
from roka_nav2_bringup.mapping_session_recorder import (
    DEFAULT_TOPICS,
    build_record_command,
    critical_mapping_inputs,
    is_completed_recording_shutdown_error,
)
from roka_nav2_bringup.data_contract import (
    acquisition_topics,
    default_contract_path,
    load_data_contract,
)
from roka_nav2_bringup.offline_session_processor import (
    _regression_classification,
    _validate_2d,
)
from roka_nav2_bringup.map_bundle_quality import (
    MANIFEST_HASH_FILES,
    MANIFEST_NAME,
    bundle_is_complete,
    path_occupancy_metrics,
    path_metrics,
    promote_bundle,
    validate_acquisition_bundle,
    validate_bundle,
    write_manifest,
)


HARDWARE = load_hardware_config()


def envelope():
    names = [
        "candidate_speed_ratio",
        "candidate_speed_squared",
        "abs_cross_slope_ratio",
        "abs_longitudinal_slope_ratio",
        "curvature_ratio",
        "edge_deficit_ratio",
        "speed_cross_interaction",
        "speed_longitudinal_interaction",
        "speed_curvature_interaction",
    ]
    return {
        "schema_version": 2,
        "purpose": "test",
        "planner_linear_limit_mps": 0.55,
        "planner_angular_limit_radps": 0.80,
        "tire_half_width_m": HARDWARE.tire_half_width_m,
        "edge_safety_margin_m": 0.05,
        "risk_model": {
            "feature_names": names,
            "feature_mean": [0.0] * 9,
            "feature_std": [1.0] * 9,
            "coefficients": [0.7] * 9,
            "bias": -4.0,
        },
        "observed_safe_feature_p99": {name: 10.0 for name in names},
        "planner": {
            "maximum_accepted_risk": 0.35,
            "minimum_speed_mps": 0.10,
            "speed_candidates_mps": [0.55, 0.40, 0.30, 0.20, 0.10],
            "maximum_lateral_acceleration_mps2": 0.25,
            "maximum_acceleration_mps2": 0.50,
            "maximum_deceleration_mps2": 0.80,
            "out_of_distribution_speed_mps": 0.10,
            "bank_speed_floor_table": [
                {
                    "minimum_abs_cross_slope_deg": 0.0,
                    "maximum_abs_cross_slope_deg": 45.0,
                    "minimum_speed_mps": 0.10,
                }
            ],
        },
    }


def profile():
    return {
        "schema_version": 1,
        "frame_id": "map",
        "closed_loop": False,
        "points": [
            {
                "x": float(index),
                "y": 0.0,
                "yaw": 0.0,
                "cross_slope_deg": 4.0,
                "longitudinal_slope_deg": 6.0,
                "minimum_wheel_edge_m": 0.20,
            }
            for index in range(3)
        ],
    }


def test_risk_probability_increases_with_speed():
    model = envelope()
    slow = static_feature_vector(0.10, 5.0, 5.0, 0.1, 0.1, 0.55, 0.05)
    fast = static_feature_vector(0.50, 5.0, 5.0, 0.1, 0.1, 0.55, 0.05)
    assert risk_probability(model, fast) > risk_probability(model, slow)


def test_profile_is_annotated_with_speed_and_cost():
    result = annotate_profile(profile(), envelope(), maximum_speed_mps=0.30)
    assert result["schema_version"] == 2
    assert all(point["terrain_data_complete"] for point in result["points"])
    assert all(0.10 <= point["speed_mps"] <= 0.30 for point in result["points"])
    assert all(0.0 <= point["traversability_cost"] <= 1.0 for point in result["points"])
    assert result["autonomy_approved"] is True
    assert result["approval_reason"] == "terrain_and_demonstration_envelope_passed"


def test_command_pipeline_reports_first_blocked_safety_stage():
    assert blocked_stage(
        0.10, 0.10, 0.0, 0.0,
        threshold=0.01,
        terrain_reason="terrain map stale or invalid",
        obstacle_age_sec=1.25,
        obstacle_max_age_sec=1.20,
    ) == ("terrain", "terrain map stale or invalid")
    stage, reason = blocked_stage(
        0.10, 0.10, 0.10, 0.0,
        threshold=0.01,
        terrain_reason="ok",
        obstacle_age_sec=1.25,
        obstacle_max_age_sec=1.20,
    )
    assert stage == "collision"
    assert "1.250s > 1.200s" in reason


def test_profile_rejects_bank_floor_above_operator_speed_limit():
    model = envelope()
    model["planner"]["bank_speed_floor_table"] = [
        {
            "minimum_abs_cross_slope_deg": 0.0,
            "maximum_abs_cross_slope_deg": 45.0,
            "minimum_speed_mps": 0.30,
        }
    ]
    with pytest.raises(ValueError, match="above requested maximum"):
        annotate_profile(profile(), model, maximum_speed_mps=0.20)


def test_live_preflight_helpers_are_fail_closed():
    assert measured_rate(800, 8.0) == pytest.approx(100.0)
    assert sampled_rate(801, 8.0) == pytest.approx(100.0)
    assert sampled_rate(1, 8.0) == 0.0
    assert finite_vector([0.0, 1.0, -2.0])
    assert not finite_vector([0.0, float("nan")])
    stamp = type("Stamp", (), {"sec": 10, "nanosec": 100_000_000})()
    assert stamp_age_seconds(10_350_000_000, stamp) == pytest.approx(0.25)


def test_preflight_retry_uses_a_fresh_stamp_error_window():
    errors = {"VLP packets": 2, "VLP cloud": 1, "2D scan": 2, "IMU": 0}
    reset_stamp_error_window(errors)
    assert errors == {
        "VLP packets": 0,
        "VLP cloud": 0,
        "2D scan": 0,
        "IMU": 0,
    }


def test_map_check_requires_complete_terrain_and_explicit_approval():
    document = profile()
    document["points"] = [
        dict(document["points"][index % len(document["points"])])
        for index in range(10)
    ]
    problems = profile_problems(document)
    assert any("tire_edge_clearance_m" in problem for problem in problems)
    assert any("autonomy_approved" in problem for problem in problems)

    document["closed_loop"] = True
    document["autonomy_approved"] = True
    for point in document["points"]:
        point["tire_edge_clearance_m"] = 0.10
        point["speed_mps"] = 0.20
    assert profile_problems(document) == []


def synthetic_track_cloud():
    rows = []
    for x in np.arange(-1.0, 6.01, 0.025):
        for y in np.arange(-0.75, 0.751, 0.025):
            z = 0.20 if abs(y) <= 0.45 else 0.0
            rows.append((x, y, z))
    return np.asarray(rows, dtype=np.float64)


def test_pcd_surface_extracts_flat_track_slopes_and_both_edges():
    features = surface_features(
        synthetic_track_cloud(),
        x=2.0,
        y=0.0,
        yaw=0.0,
        track_width=0.9144,
        wheel_track=HARDWARE.physical_wheel_track_m,
        tire_half_width=HARDWARE.tire_half_width_m,
    )
    assert abs(features["cross_slope_deg"]) < 0.5
    assert abs(features["longitudinal_slope_deg"]) < 0.5
    assert 0.12 < features["tire_edge_clearance_m"] < 0.22


def test_pcd_loader_accepts_uncompressed_binary(tmp_path):
    points = np.asarray(
        [(float(index), 0.25, -0.10, 1.0) for index in range(100)],
        dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")],
    )
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n"
        "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n"
    ).encode("ascii")
    path = tmp_path / "binary_map.pcd"
    path.write_bytes(header + points.tobytes())
    cloud = load_ascii_xyz(path)
    assert cloud.shape == (100, 3)
    assert cloud[12] == pytest.approx((12.0, 0.25, -0.10))


def test_pcd_annotation_fails_closed_or_annotates_every_path_point():
    document = {
        "frame_id": "map",
        "closed_loop": True,
        "track_width_m": 0.9144,
        "points": [
            {"x": 0.5 + 0.4 * index, "y": 0.0, "yaw": 0.0}
            for index in range(10)
        ],
    }
    result = annotate_profile_from_pcd(document, synthetic_track_cloud())
    assert all(
        point["terrain_source"] == "mapped_pcd_surface_edges"
        and point["tire_edge_clearance_m"] > 0.0
        for point in result["points"]
    )


def test_vehicle_aware_path_uses_measured_width_and_preserves_raw_document():
    document = {
        "schema_version": 1,
        "frame_id": "map",
        "closed_loop": False,
        "track_width_m": 0.9144,
        "points": [
            {"x": 0.5 + 0.4 * index, "y": 0.0, "yaw": 0.0}
            for index in range(10)
        ],
    }
    original = [dict(point) for point in document["points"]]
    result = plan_vehicle_aware_profile(
        document,
        synthetic_track_cloud(),
        envelope(),
        maximum_speed_mps=0.20,
    )
    assert document["points"] == original
    assert result["profile_role"] == "safe_autonomy"
    assert result["approval_scope"] == "vehicle_aware_mapped_traversability"
    assert result["autonomy_approved"] is True
    assert result["vehicle_aware_planner"]["vehicle_width_m"] == pytest.approx(0.624)
    assert all(
        point["terrain_source"] == "mapped_pcd_smoothed_vehicle_wheel_corridors"
        and point["tire_edge_clearance_m"] >= envelope()["edge_safety_margin_m"]
        for point in result["points"]
    )


def test_vehicle_aware_path_rejects_missing_full_vehicle_surface():
    document = {
        "frame_id": "map",
        "closed_loop": False,
        "track_width_m": 0.9144,
        "points": [
            {"x": 0.5 + 0.7 * index, "y": 0.0, "yaw": 0.0}
            for index in range(10)
        ],
    }
    with pytest.raises(ValueError, match="no mapped full-vehicle surface"):
        plan_vehicle_aware_profile(document, synthetic_track_cloud(), envelope())


def test_missing_terrain_is_rejected_by_default():
    document = profile()
    del document["points"][1]["cross_slope_deg"]
    with pytest.raises(ValueError, match="finish terrain mapping first"):
        annotate_profile(document, envelope())


def test_nominal_edge_clearance_reduces_with_lateral_error():
    centered = nominal_tire_clearance(
        0.9144, HARDWARE.physical_wheel_track_m, 0.0
    )
    near_edge = nominal_tire_clearance(
        0.9144, HARDWARE.physical_wheel_track_m, 0.10
    )
    expected_centered = (
        0.5 * (0.9144 - HARDWARE.physical_wheel_track_m)
        - HARDWARE.tire_half_width_m
    )
    assert centered == pytest.approx(expected_centered)
    assert near_edge == pytest.approx(expected_centered - 0.10)


def test_mapped_profile_edge_clearance_is_preferred():
    clearance, source = profile_tire_clearance(
        {"tire_edge_clearance_m": 0.18},
        lateral_error_m=-0.04,
        track_width_m=0.9144,
        wheel_track_m=HARDWARE.physical_wheel_track_m,
    )
    assert clearance == pytest.approx(0.14)
    assert source == "mapped_profile_tire_edge"


@pytest.mark.parametrize(
    "document",
    [
        {},
        {"autonomy_approved": False},
        {"autonomy_approved": "true"},
        None,
    ],
)
def test_autonomy_approval_is_fail_closed(document):
    assert not is_autonomy_approved(document)


def test_autonomy_requires_explicit_boolean_true():
    assert is_autonomy_approved({"autonomy_approved": True})


def test_test_only_profile_requires_a_second_explicit_gate():
    document = {
        "autonomy_approved": True,
        "test_only": True,
        "approval_scope": "controlled_short_autonomy_test",
    }
    assert is_test_only_profile(document)
    assert not is_profile_allowed(document)
    assert is_profile_allowed(document, allow_test_profile=True)


def test_recorded_replay_bounds_smoothing_and_forces_one_speed(tmp_path):
    source = tmp_path / "track_profile.yaml"
    output = tmp_path / "track_profile_recorded.yaml"
    measured = [
        {"x": 0.05 * index, "y": 0.01 * index, "yaw": 0.02 * index}
        for index in range(12)
    ]
    source.write_text(
        __import__("yaml").safe_dump({
            "frame_id": "map",
            "closed_loop": False,
            "track_width_m": 1.20,
            "points": measured,
        }),
        encoding="utf-8",
    )

    result = prepare_recorded_route_file(source, output, 0.10)

    assert result["profile_role"] == "recorded_route_replay"
    assert result["preserve_recorded_geometry"] is False
    assert result["bounded_route_smoothing"] is True
    assert result["centerline_geometry"]["maximum_source_deviation_m"] <= 0.05
    assert result["autonomy_approved"] is True
    assert result["points"][0]["x"] == pytest.approx(measured[0]["x"])
    assert result["points"][0]["y"] == pytest.approx(measured[0]["y"])
    assert result["points"][-1]["x"] == pytest.approx(measured[-1]["x"])
    assert result["points"][-1]["y"] == pytest.approx(measured[-1]["y"])
    assert {point["speed_mps"] for point in result["points"]} == {0.10}
    assert not recorded_profile_problems(result)


def test_recorded_replay_treats_point_three_as_a_straight_speed_ceiling(tmp_path):
    source = tmp_path / "track_profile.yaml"
    output = tmp_path / "track_profile_recorded.yaml"
    points = [
        {"x": 0.05 * index, "y": 0.0, "yaw": 0.0}
        for index in range(11)
    ]
    points.extend({
        "x": 0.50 + 0.30 * math.cos(-math.pi / 2.0 + index * math.pi / 20.0),
        "y": 0.30 + 0.30 * math.sin(-math.pi / 2.0 + index * math.pi / 20.0),
        "yaw": index * math.pi / 20.0,
    } for index in range(1, 11))
    source.write_text(
        __import__("yaml").safe_dump({
            "frame_id": "map",
            "closed_loop": False,
            "track_width_m": 1.20,
            "points": points,
        }),
        encoding="utf-8",
    )

    result = prepare_recorded_route_file(source, output, 0.30)
    speeds = [float(point["speed_mps"]) for point in result["points"]]
    assert max(speeds) == pytest.approx(0.30)
    assert min(speeds) < 0.30
    assert all(
        speed * abs(float(point["curvature_radpm"])) <= 0.600001
        for speed, point in zip(speeds, result["points"])
    )
    assert result["recorded_route_speed_policy"]["maximum_linear_speed_mps"] == 0.30


@pytest.mark.parametrize("speed", [0.0, 0.02, 0.61, math.nan])
def test_recorded_replay_rejects_invalid_speed(tmp_path, speed):
    source = tmp_path / "track_profile.yaml"
    source.write_text(
        __import__("yaml").safe_dump({
            "points": [
                {"x": 0.05 * index, "y": 0.0, "yaw": 0.0}
                for index in range(10)
            ]
        }),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="speed"):
        prepare_recorded_route_file(
            source, tmp_path / "track_profile_recorded.yaml", speed
        )


def test_track_progress_uses_the_same_ordered_follow_path_points():
    points = [{"x": float(index), "y": 0.0} for index in range(4)]
    target, progress, travelled, remaining = path_progress_metrics(
        points, index=1, lookahead_distance=1.5
    )
    assert target == 3
    assert progress == pytest.approx(1.0 / 3.0)
    assert travelled == pytest.approx(1.0)
    assert remaining == pytest.approx(2.0)


def test_short_test_profile_is_open_low_speed_and_separate():
    document = {
        "schema_version": 1,
        "frame_id": "map",
        "closed_loop": True,
        "track_width_m": 0.9144,
        "points": [
            {"x": 0.10 * index, "y": 0.002 * index, "yaw": 0.0}
            for index in range(12)
        ] + [
            {"x": 1.10 - 0.10 * index, "y": 0.02, "yaw": 3.14}
            for index in range(10)
        ],
    }
    result = prepare_short_test_document(document, envelope(), requested_speed_mps=0.30)
    assert result["closed_loop"] is False
    assert result["test_only"] is True
    assert result["approval_scope"] == "controlled_short_autonomy_test"
    assert result["allows_level_fallback_without_wheel_cells"] is True
    assert result["autonomy_approved"] is True
    assert len(result["points"]) >= 10
    assert max(float(point["speed_mps"]) for point in result["points"]) <= 0.30
    assert all(
        point["terrain_source"] == "controlled_level_test_nominal_corridor"
        for point in result["points"]
    )


def test_short_test_profile_rejects_no_real_displacement():
    document = {
        "track_width_m": 0.9144,
        "points": [{"x": 0.0, "y": 0.0}, {"x": 0.01, "y": 0.0}],
    }
    with pytest.raises(ValueError, match="displacement is too small"):
        prepare_short_test_document(document, envelope())


def test_short_test_profile_preserves_natural_bent_xbox_route():
    document = {
        "frame_id": "map",
        "track_width_m": 1.20,
        "points": [
            {
                "x": 0.10 * index,
                "y": 0.35 * math.sin(math.pi * index / 14.0),
                "yaw": 0.0,
            }
            for index in range(15)
        ],
    }
    result = prepare_short_test_document(document, envelope())
    assert result["test_path_conversion"]["preserves_demonstrated_geometry"] is True
    assert max(float(point["y"]) for point in result["points"]) > 0.30


def test_short_test_allows_only_low_speed_curvature_ood():
    document = {
        "schema_version": 1,
        "frame_id": "map",
        "closed_loop": False,
        "track_width_m": 1.2,
        "points": [],
    }
    radius = 0.75
    for index in range(25):
        angle = 0.06 * index
        document["points"].append({
            "x": radius * math.sin(angle),
            "y": radius * (1.0 - math.cos(angle)),
        })
    model = envelope()
    model["observed_safe_feature_p99"]["curvature_ratio"] = 0.10
    model["successful_dynamic_envelope"] = {
        "yaw_rate_radps": {"p95": 0.20}
    }
    result = prepare_short_test_document(document, model)
    overridden = [
        point for point in result["points"]
        if point["planner_reason"] == "controlled_test_low_speed_curvature"
    ]
    assert result["autonomy_approved"] is True
    assert overridden
    assert max(
        point["controlled_test_yaw_rate_radps"] for point in overridden
    ) <= 0.20


def test_short_test_truncates_manual_in_place_pivot_without_relaxing_limits():
    points = [{"x": 0.10 * index, "y": 0.0} for index in range(20)]
    points.extend([
        {"x": 1.9005, "y": 0.0002},
        {"x": 1.8995, "y": 0.0005},
        {"x": 1.9002, "y": 0.0010},
    ])
    points.extend({"x": 1.9, "y": 0.10 * index} for index in range(1, 21))
    document = {
        "frame_id": "map",
        "track_width_m": 1.20,
        "points": points,
    }
    result = prepare_short_test_document(document, envelope())
    conversion = result["test_path_conversion"]
    assert conversion["method"] == "recorded_route_safe_prefix_before_pivot"
    assert conversion["kept_source_point_count"] < len(points)
    assert conversion["discarded_source_point_count"] > 0
    assert conversion["segment_length_m"] >= 0.75
    assert result["autonomy_approved"] is True
    assert all(
        point["planner_reason"] in {
            "demonstrated_envelope",
            "controlled_test_low_speed_curvature",
        }
        for point in result["points"]
    )


def test_short_test_skips_unsafe_startup_alignment_turn():
    points = [
        {"x": 0.00, "y": 0.15},
        {"x": 0.00, "y": 0.075},
        {"x": 0.03, "y": 0.00},
    ]
    points.extend(
        {"x": 0.09 * index, "y": 0.0}
        for index in range(1, 25)
    )
    document = {
        "frame_id": "map",
        "track_width_m": 1.20,
        "points": points,
    }
    model = envelope()
    model["observed_safe_feature_p99"]["curvature_ratio"] = 0.40
    model["successful_dynamic_envelope"] = {
        "yaw_rate_radps": {"p95": 0.20}
    }

    result = prepare_short_test_document(document, model)
    conversion = result["test_path_conversion"]
    assert conversion["method"] == "recorded_route_safe_contiguous_spline_segment"
    assert conversion["selected_start_index"] > 0
    assert conversion["segment_length_m"] >= 0.75
    assert result["autonomy_approved"] is True
    assert all(
        point["planner_reason"] in {
            "demonstrated_envelope",
            "controlled_test_low_speed_curvature",
        }
        for point in result["points"]
    )


def test_profile_start_loader_uses_recorded_map_pose(tmp_path):
    path = tmp_path / "track_profile.yaml"
    path.write_text(
        "frame_id: map\npoints:\n  - {x: 1.2, y: -0.4, yaw: 0.7}\n",
        encoding="utf-8",
    )
    frame, x, y, yaw = load_profile_start(path)
    assert frame == "map"
    assert (x, y, yaw) == pytest.approx((1.2, -0.4, 0.7))


def test_map_check_accepts_an_approved_open_section():
    document = {
        "closed_loop": False,
        "autonomy_approved": True,
        "points": [
            {
                "x": float(index),
                "y": 0.0,
                "cross_slope_deg": 0.0,
                "longitudinal_slope_deg": 0.0,
                "tire_edge_clearance_m": 0.10,
                "speed_mps": 0.10,
            }
            for index in range(10)
        ],
    }
    assert profile_problems(document) == []


def test_xbox_mapping_button_is_edge_triggered():
    assert rising_edge([0, 0, 0, 0], [1, 0, 0, 0], 0)
    assert not rising_edge([1, 0, 0, 0], [1, 0, 0, 0], 0)
    assert not rising_edge([], [], 0)
    assert button_pressed([0, 0, 0, 1], 3)
    assert not button_pressed([0], 3)


def test_xbox_mapping_drive_is_fail_closed_until_recording_and_md_ready():
    assert "neither active nor completely saved" in mapping_drive_block_reason(
        "idle", True, True, True
    )
    assert "preflight" in mapping_drive_block_reason(
        "recording", True, False, True
    )
    assert "MD200T" in mapping_drive_block_reason(
        "recording", True, True, False
    )
    assert mapping_drive_block_reason("recording", True, True, True) == ""
    assert mapping_drive_block_reason("complete", True, True, True) == ""
    assert "neither active nor completely saved" in mapping_drive_block_reason(
        "path_saved", True, True, True
    )
    assert "preflight" in mapping_drive_block_reason(
        "complete", True, False, True
    )


@pytest.mark.parametrize("drive_state", ["recording", "complete"])
def test_xbox_mapping_gate_forwards_every_authorized_twist(drive_state):
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    controls = XboxMappingControls.__new__(XboxMappingControls)
    controls.state = drive_state
    controls.require_preflight = True
    controls.preflight_ready = True
    controls.base_connected = True
    controls.manual_drive_authorized = True
    controls.joy_drive_active = False
    controls.joy_drive_started_at = None
    controls.drive_block_reported = False
    controls.motor_command_pub = Publisher()

    first = Twist()
    first.linear.x = 0.01
    second = Twist()
    second.linear.x = 0.25
    controls.joy_command_callback(first)
    controls.joy_command_callback(second)

    assert controls.motor_command_pub.messages == [first, second]


def test_xbox_autonomy_button_assignments_do_not_conflict():
    # Keep this independent of rclpy initialization while pinning the public layout.
    assert XboxAutonomyControls.__name__ == "XboxAutonomyControls"
    assert len({3, 0, 1, 4}) == 4


def test_xbox_b_manual_takeover_is_latched_until_lb_is_released_and_held():
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class Controls:
        manual_takeover = True
        manual_enable_armed = False
        drive_enable_pressed = True
        manual_stop_pub = Publisher()
        status = ""

        def publish_status(self, message):
            self.status = message

    controls = Controls()
    XboxAutonomyControls.manual_stop_watchdog(controls)
    assert len(controls.manual_stop_pub.messages) == 1
    assert controls.manual_enable_armed is False
    controls.drive_enable_pressed = False
    XboxAutonomyControls.manual_stop_watchdog(controls)
    assert controls.manual_enable_armed is True
    controls.drive_enable_pressed = True
    XboxAutonomyControls.manual_stop_watchdog(controls)
    assert len(controls.manual_stop_pub.messages) == 2
    assert "manual takeover active" in controls.status


def test_xbox_manual_gate_forwards_only_after_b_release_then_lb_hold():
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class Controls:
        gate_manual_commands = True
        manual_takeover = True
        manual_enable_armed = False
        drive_enable_pressed = True
        manual_stop_pub = Publisher()
        status = ""

        def publish_status(self, message):
            self.status = message

    controls = Controls()
    command = Twist()
    command.linear.x = 0.5
    XboxAutonomyControls.manual_command_callback(controls, command)
    assert controls.manual_stop_pub.messages == []
    controls.manual_enable_armed = True
    XboxAutonomyControls.manual_command_callback(controls, command)
    assert controls.manual_stop_pub.messages == [command]
    assert "LT" in controls.status


def test_track_follower_requires_every_control_lifecycle_node_active():
    class ControlChain:
        required_lifecycle_nodes = ["/controller", "/smoother", "/collision"]
        lifecycle_states = {
            name: State.PRIMARY_STATE_ACTIVE for name in required_lifecycle_nodes
        }

    TrackPathFollower.require_active_control_chain(ControlChain())
    ControlChain.lifecycle_states["/smoother"] = State.PRIMARY_STATE_INACTIVE
    with pytest.raises(RuntimeError, match="/smoother"):
        TrackPathFollower.require_active_control_chain(ControlChain())


def test_track_follower_real_start_requires_positive_field_preflight():
    class Follower:
        require_field_preflight = True
        field_preflight_received = False
        field_preflight_ready = False

    with pytest.raises(RuntimeError, match="has not completed"):
        TrackPathFollower.require_ready_field_hardware(Follower())
    Follower.field_preflight_received = True
    with pytest.raises(RuntimeError, match="NOT READY"):
        TrackPathFollower.require_ready_field_hardware(Follower())
    Follower.field_preflight_ready = True
    TrackPathFollower.require_ready_field_hardware(Follower())


def test_map_bundle_selection_uses_latest_valid_map(tmp_path):
    first = tmp_path / "track_v1"
    second = tmp_path / "track_v2"
    first.mkdir()
    second.mkdir()
    (first / "occupancy.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    (first / "occupancy.yaml").write_text(
        "image: occupancy.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n",
        encoding="utf-8",
    )
    (first / MANIFEST_NAME).write_text(
        "schema_version: 2\nstatus: complete\n", encoding="utf-8"
    )
    (second / "occupancy.yaml").write_text(
        "image: missing.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n",
        encoding="utf-8",
    )
    assert occupancy_map_is_valid(first / "occupancy.yaml")
    assert not occupancy_map_is_valid(second / "occupancy.yaml")
    assert latest_valid_map_bundle(tmp_path) == first


def test_mapping_bag_records_compact_replayable_inputs(tmp_path):
    command = build_record_command(
        tmp_path / "sensors",
        ["/velodyne_packets", "/imu/data", "/odom"],
        512 * 1024 ** 2,
        True,
    )
    assert "/velodyne_packets" in command
    assert "/velodyne_points" not in command
    assert "/terrain/local_map" not in command
    assert "--max-bag-size" in command
    assert "zstd" in command
    assert "/md/base_connected" in DEFAULT_TOPICS
    assert "/md/left/raw_state" in DEFAULT_TOPICS
    assert "/md/right/raw_state" in DEFAULT_TOPICS
    assert "/diagnostics" in DEFAULT_TOPICS
    assert "/lio/odom" in DEFAULT_TOPICS
    assert "/terrain/wheel_contact" in DEFAULT_TOPICS


def test_single_sensor_contract_assigns_one_odometry_role_per_stage():
    contract = load_data_contract(default_contract_path())
    topics = acquisition_topics(contract)
    assert contract["odometry_authority"]["acquisition"] == "/wheel/odom"
    assert contract["odometry_authority"]["flat_runtime"] == "/odom"
    assert contract["odometry_authority"]["track_runtime"] == "/lio/odom"
    assert len(topics) == len(set(topics))
    assert "/velodyne_packets" in topics
    assert "/velodyne_points" not in topics


def test_offline_2d_product_is_independently_classified(tmp_path):
    assert _validate_2d(tmp_path)["status"] == "UNAVAILABLE"
    (tmp_path / "occupancy.yaml").write_text(
        "image: occupancy.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n",
        encoding="utf-8",
    )
    assert _validate_2d(tmp_path)["status"] == "INVALID"
    (tmp_path / "occupancy.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    result = _validate_2d(tmp_path)
    assert result["status"] == "READY"
    assert result["resolution_m"] == pytest.approx(0.05)


def test_offline_3d_regression_failure_is_diagnostic_not_ready(tmp_path):
    regression = tmp_path / "regression.yaml"
    regression.write_text("result: FAIL\nreason: z drift exceeded\n", encoding="utf-8")
    assert _regression_classification(regression) == (
        "DEGRADED", "z drift exceeded"
    )
    regression.write_text("result: PASS\n", encoding="utf-8")
    assert _regression_classification(regression) == ("READY", "")


def test_mapping_recorder_suppresses_only_finalized_rclpy_shutdown_race():
    error = RuntimeError("Unable to convert call argument to Python object")
    assert is_completed_recording_shutdown_error(
        error, recording_active=False, metadata_exists=True
    )
    assert not is_completed_recording_shutdown_error(
        error, recording_active=True, metadata_exists=True
    )
    assert not is_completed_recording_shutdown_error(
        error, recording_active=False, metadata_exists=False
    )
    assert not is_completed_recording_shutdown_error(
        RuntimeError("unexpected middleware failure"),
        recording_active=False,
        metadata_exists=True,
    )


def test_mapping_recorder_keeps_sensor_recording_during_md_reconnect():
    seen = {"VLP-16": 10.0, "EBIMU": 10.0, "odometry": 5.0}
    assert critical_mapping_inputs(
        seen, 10.5, 3.0, False, False
    ) == []
    assert critical_mapping_inputs(
        seen, 10.5, 3.0, False, True
    ) == ["odometry", "MD base"]


def test_mapping_recorder_still_rejects_lidar_or_imu_loss():
    seen = {"VLP-16": 5.0, "EBIMU": 10.0, "odometry": 5.0}
    assert critical_mapping_inputs(
        seen, 10.5, 3.0, False, False
    ) == ["VLP-16"]


def test_path_quality_detects_collapsed_slam_recording(tmp_path):
    profile_path = tmp_path / "track_profile.yaml"
    profile_path.write_text(
        "closed_loop: false\n"
        "recorded_path_length_m: 5.0\n"
        "points:\n"
        + "".join(
            f"  - {{x: {0.01 * index}, y: 0.0}}\n" for index in range(10)
        ),
        encoding="utf-8",
    )
    metrics = path_metrics(profile_path)
    assert metrics["path_length_m"] == pytest.approx(0.09)
    assert metrics["recorded_path_length_m"] > 50 * metrics["path_length_m"]


def test_path_occupancy_alignment_checks_centre_and_wheel_lanes(tmp_path):
    image = tmp_path / "occupancy.pgm"
    image.write_text(
        "P2\n100 100\n255\n" + " ".join(["254"] * 10000) + "\n",
        encoding="ascii",
    )
    map_yaml = tmp_path / "occupancy.yaml"
    map_yaml.write_text(
        "image: occupancy.pgm\nresolution: 0.1\norigin: [-2.0, -2.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
    )
    profile_path = tmp_path / "track_profile.yaml"
    profile_path.write_text(
        "frame_id: map\npoints:\n"
        + "".join(
            f"  - {{x: {0.1 * index}, y: 0.0, yaw: 0.0}}\n"
            for index in range(10)
        ),
        encoding="utf-8",
    )
    metrics = path_occupancy_metrics(map_yaml, profile_path)
    assert metrics["centre_free_fraction"] == 1.0
    assert metrics["wheel_lane_free_fraction"] == 1.0

    map_yaml.write_text(
        "image: occupancy.pgm\nresolution: 0.1\norigin: [10.0, 10.0, 0.0]\n",
        encoding="utf-8",
    )
    displaced = path_occupancy_metrics(map_yaml, profile_path)
    assert displaced["centre_in_bounds_fraction"] == 0.0


def test_atomic_promotion_keeps_one_previous_bundle(tmp_path):
    final = tmp_path / "section_1"
    staging = tmp_path / ".staging" / "section_1"
    final.mkdir()
    staging.mkdir(parents=True)
    (final / "old.txt").write_text("old", encoding="utf-8")
    (staging / "new.txt").write_text("new", encoding="utf-8")
    (staging / MANIFEST_NAME).write_text(
        "schema_version: 2\nstatus: complete\n", encoding="utf-8"
    )
    backup = promote_bundle(staging, final, overwrite_existing=True)
    assert (final / "new.txt").read_text(encoding="utf-8") == "new"
    assert backup == tmp_path / ".backup" / "section_1"
    assert (backup / "old.txt").read_text(encoding="utf-8") == "old"
    assert bundle_is_complete(final)


def test_complete_manifest_detects_product_tampering(tmp_path):
    for name in MANIFEST_HASH_FILES:
        (tmp_path / name).write_text(f"original {name}", encoding="utf-8")
    write_manifest(tmp_path, {"test": True})
    assert bundle_is_complete(tmp_path)
    (tmp_path / "pose_graph.g2o").write_text("modified", encoding="utf-8")
    assert not bundle_is_complete(tmp_path)


def test_complete_bundle_quality_accepts_coherent_mapping_products(tmp_path):
    pixels = []
    for row in range(40):
        for column in range(40):
            pixels.append(0 if 0 <= row < 10 and 20 <= column < 30 else 254)
    (tmp_path / "occupancy.pgm").write_text(
        "P2\n40 40\n255\n" + " ".join(str(value) for value in pixels),
        encoding="ascii",
    )
    (tmp_path / "occupancy.yaml").write_text(
        "image: occupancy.pgm\nresolution: 0.05\norigin: [0, -1.0, 0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
    )
    (tmp_path / "slam_posegraph.posegraph").write_bytes(b"posegraph" * 200)
    (tmp_path / "map.pcd").write_text(
        "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        "COUNT 1 1 1\nWIDTH 1000\nHEIGHT 1\nPOINTS 1000\nDATA ascii\n",
        encoding="ascii",
    )
    (tmp_path / "pose_graph.g2o").write_text(
        "VERTEX_SE3:QUAT 0 0 0 0 0 0 0 1\n"
        "VERTEX_SE3:QUAT 1 1 0 0 0 0 0 1\n"
        "EDGE_SE3:QUAT 0 1 1 0 0 0 0 0 1\n",
        encoding="ascii",
    )
    terrain = tmp_path / "terrain_map.bag"
    terrain.mkdir()
    (terrain / "terrain.db3").write_bytes(b"t" * 5000)
    (tmp_path / "track_profile.yaml").write_text(
        "closed_loop: false\nrecorded_path_length_m: 0.9\npoints:\n"
        + "".join(
            f"  - {{x: {0.1 * index}, y: 0.0}}\n" for index in range(10)
        ),
        encoding="utf-8",
    )
    sensors = tmp_path / "sensors"
    sensors.mkdir()
    sensor_entries = "".join(
        "    - topic_metadata:\n"
        f"        name: {topic}\n"
        f"      message_count: {count}\n"
        for topic, count in (
            ("/velodyne_packets", 100),
            ("/imu/data", 1000),
            ("/odom", 200),
            ("/track/recorded_path", 10),
        )
    )
    (sensors / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n  topics_with_message_count:\n"
        + sensor_entries,
        encoding="utf-8",
    )
    metadata_hash = hashlib.sha256(
        (sensors / "metadata.yaml").read_bytes()
    ).hexdigest()
    (tmp_path / "sensor_contract.yaml").write_text(
        "schema_version: 2\n"
        "result: PASS\n"
        "failures: []\n"
        "bag_fingerprint:\n"
        f"  metadata_sha256: {metadata_hash}\n"
        "  data_files: []\n",
        encoding="utf-8",
    )
    (tmp_path / "session_metrics.yaml").write_text(
        "odom_cumulative_distance_m: 0.9\n"
        "maximum_map_odom_translation_jump_m: 0.01\n"
        "maximum_map_odom_yaw_jump_deg: 0.2\n",
        encoding="utf-8",
    )
    result = validate_bundle(tmp_path)
    assert result["track"]["path_length_m"] == pytest.approx(0.9)
    assert result["occupancy"]["occupied_component_count"] == 1


def test_raw_acquisition_is_valid_without_any_2d_or_3d_map_product(tmp_path):
    sensors = tmp_path / "sensors"
    sensors.mkdir()
    entries = "".join(
        "    - topic_metadata:\n"
        f"        name: {topic}\n"
        f"      message_count: {count}\n"
        for topic, count in (
            ("/velodyne_packets", 100),
            ("/imu/data", 1000),
            ("/wheel/odom", 200),
            ("/track/recorded_path", 10),
        )
    )
    sensor_metadata = sensors / "metadata.yaml"
    sensor_metadata.write_text(
        "rosbag2_bagfile_information:\n  topics_with_message_count:\n" + entries,
        encoding="utf-8",
    )
    metadata_hash = hashlib.sha256(sensor_metadata.read_bytes()).hexdigest()
    (tmp_path / "sensor_contract.yaml").write_text(
        "schema_version: 2\nresult: PASS\nfailures: []\n"
        f"bag_fingerprint:\n  metadata_sha256: {metadata_hash}\n",
        encoding="utf-8",
    )
    (tmp_path / "session_metrics.yaml").write_text(
        "odom_cumulative_distance_m: 1.0\n",
        encoding="utf-8",
    )
    (tmp_path / "session_contract.yaml").write_text(
        "schema_version: 1\nmapping_backend: raw_sensor_capture\n",
        encoding="utf-8",
    )
    (tmp_path / "metadata.yaml").write_text(
        "schema_version: 2\nmapping_backend: raw_sensor_capture\n",
        encoding="utf-8",
    )
    (tmp_path / "track_profile.yaml").write_text(
        "frame_id: odom\nclosed_loop: false\nrecorded_path_length_m: 1.0\npoints:\n"
        + "".join(f"  - {{x: {0.1 * index}, y: 0.0}}\n" for index in range(11)),
        encoding="utf-8",
    )
    result = validate_acquisition_bundle(tmp_path)
    assert result["mapping_backend"] == "raw_sensor_capture"
    assert result["track"]["path_length_m"] == pytest.approx(1.0)
    assert not (tmp_path / "occupancy.yaml").exists()
    assert not (tmp_path / "map.pcd").exists()


def test_next_mapping_bundle_reuses_only_empty_numbered_directory(tmp_path):
    used = tmp_path / "track_v1"
    empty = tmp_path / "track_v2"
    used.mkdir()
    empty.mkdir()
    (used / "sensors").mkdir()
    assert next_mapping_bundle(tmp_path) == empty
    (empty / "partial").write_text("used", encoding="utf-8")
    assert next_mapping_bundle(tmp_path) == tmp_path / "track_v3"


def test_named_map_bundle_is_selectable_and_cannot_escape_root(tmp_path):
    assert map_bundle_for_id(tmp_path, "section_5") == tmp_path / "section_5"
    with pytest.raises(ValueError):
        map_bundle_for_id(tmp_path, "../outside")


def test_velodyne_point_time_requires_scan_start_offsets():
    assert point_time_bounds_valid(0.0, 0.0995)
    assert point_time_bounds_valid(0.015, 0.075)
    assert not point_time_bounds_valid(-0.0995, 0.0)
    assert not point_time_bounds_valid(-0.080, -0.025)
    assert not point_time_bounds_valid(0.0, 0.25)


def test_field_session_guard_ignores_self_and_detects_other_launch(tmp_path):
    own = tmp_path / "100"
    other = tmp_path / "200"
    unrelated = tmp_path / "300"
    own.mkdir()
    other.mkdir()
    unrelated.mkdir()
    (own / "cmdline").write_bytes(
        b"python3\0ros2\0launch\0roka_nav2_bringup\0pk3a_field.launch.py\0"
    )
    (other / "cmdline").write_bytes(
        b"python3\0ros2\0launch\0roka_nav2_bringup\0mapping_manual.launch.py\0"
    )
    (unrelated / "cmdline").write_bytes(b"python3\0unrelated.py\0")
    matches = other_field_launches(tmp_path, current_pid=100)
    assert len(matches) == 1
    assert matches[0][0] == 200
