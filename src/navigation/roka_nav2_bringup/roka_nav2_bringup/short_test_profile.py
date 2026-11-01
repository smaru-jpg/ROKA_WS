"""Build a conservative, explicit test-only path from a mapping run."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import yaml

from roka_nav2_bringup.profile_validation import is_autonomy_approved
from roka_nav2_bringup.route_geometry import prepare_centerline
from roka_nav2_bringup.risk_aware_profile import (
    annotate_profile,
    static_feature_vector,
)
from roka_robot_description.hardware_config import load_hardware_config


TEST_SPEED_LIMIT_MPS = 0.30
MINIMUM_TEST_DISPLACEMENT_M = 0.75
CONTROLLED_CURVATURE_REASON = "controlled_test_low_speed_curvature"


def _finite_xy(point: dict[str, Any]) -> tuple[float, float]:
    x = float(point["x"])
    y = float(point["y"])
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError("short test path contains a non-finite position")
    return x, y


def _apply_controlled_curvature_policy(
    prepared: dict[str, Any], envelope: dict[str, Any]
) -> int:
    """Allow curvature-only OOD points when low-speed dynamics are demonstrated.

    This is deliberately scoped to the separately gated controlled-area test
    profile. It never relaxes missing terrain, edge clearance, slope, risk,
    lateral acceleration, or the normal-track approval path.
    """
    planner = envelope["planner"]
    risk_limit = float(planner["maximum_accepted_risk"])
    lateral_limit = float(planner["maximum_lateral_acceleration_mps2"])
    planner_yaw_limit = float(envelope["planner_angular_limit_radps"])
    demonstrated_yaw = (
        envelope.get("successful_dynamic_envelope", {})
        .get("yaw_rate_radps", {})
        .get("p95", planner_yaw_limit)
    )
    yaw_limit = min(planner_yaw_limit, float(demonstrated_yaw))
    feature_names = list(envelope["risk_model"]["feature_names"])
    observed = envelope["observed_safe_feature_p99"]
    checked = {
        "abs_cross_slope_ratio",
        "abs_longitudinal_slope_ratio",
        "curvature_ratio",
        "edge_deficit_ratio",
    }
    linear_limit = float(envelope["planner_linear_limit_mps"])
    edge_margin = float(envelope["edge_safety_margin_m"])
    overridden = 0

    for point in prepared["points"]:
        if point.get("planner_reason") != "outside_demonstrated_envelope":
            continue
        speed = abs(float(point["speed_mps"]))
        curvature = abs(float(point["curvature_radpm"]))
        features = static_feature_vector(
            speed,
            float(point["cross_slope_deg"]),
            float(point["longitudinal_slope_deg"]),
            curvature,
            float(point["tire_edge_clearance_m"]),
            linear_limit,
            edge_margin,
        )
        exceeded = {
            name
            for index, name in enumerate(feature_names)
            if name in checked
            and features[index] > 1.20 * max(float(observed[name]), 1.0e-6)
        }
        yaw_rate = speed * curvature
        lateral_acceleration = speed * speed * curvature
        risk = float(point["demonstration_risk"])
        if (
            exceeded == {"curvature_ratio"}
            and risk <= risk_limit
            and yaw_rate <= yaw_limit
            and lateral_acceleration <= lateral_limit
        ):
            point["planner_reason"] = CONTROLLED_CURVATURE_REASON
            point["controlled_test_yaw_rate_radps"] = yaw_rate
            point["controlled_test_lateral_acceleration_mps2"] = lateral_acceleration
            overridden += 1

    accepted_reasons = {"demonstrated_envelope", CONTROLLED_CURVATURE_REASON}
    prepared["autonomy_approved"] = all(
        point.get("terrain_data_complete") is True
        and point.get("planner_reason") in accepted_reasons
        for point in prepared["points"]
    )
    prepared["controlled_test_curvature_policy"] = {
        "curvature_only_override": True,
        "maximum_yaw_rate_radps": yaw_limit,
        "maximum_lateral_acceleration_mps2": lateral_limit,
        "maximum_risk": risk_limit,
        "overridden_point_count": overridden,
    }
    return overridden


def prepare_short_test_document(
    document: dict[str, Any],
    envelope: dict[str, Any],
    requested_speed_mps: float = TEST_SPEED_LIMIT_MPS,
    _allow_segment_fallback: bool = True,
) -> dict[str, Any]:
    """Create an open test segment that preserves the demonstrated route.

    The former start-to-end chord could cut straight through an obstacle when
    the operator naturally drove around it.  Keep the recorded prefix through
    the farthest observed position instead. Exact straight lines and 90-degree
    turns are deliberately not required from the Xbox operator. Normal track
    autonomy never consumes this file without a second opt-in.
    """
    points = list(document.get("points", []))
    if len(points) < 2:
        raise ValueError("short test preparation needs at least two recorded points")
    start_x, start_y = _finite_xy(points[0])
    indexed = []
    for index, point in enumerate(points):
        x, y = _finite_xy(point)
        indexed.append((math.hypot(x - start_x, y - start_y), index, x, y))
    distance, endpoint_index, _, _ = max(indexed)
    if distance < MINIMUM_TEST_DISPLACEMENT_M:
        raise ValueError(
            "recorded displacement is too small for a controlled autonomy test: "
            f"{distance:.3f} m < {MINIMUM_TEST_DISPLACEMENT_M:.3f} m"
        )

    hardware = load_hardware_config()
    track_width = float(document.get("track_width_m", 0.9144))
    tire_half_width = float(envelope["tire_half_width_m"])
    tire_clearance = (
        0.5 * track_width
        - 0.5 * hardware.physical_wheel_track_m
        - tire_half_width
    )
    edge_margin = float(envelope["edge_safety_margin_m"])
    if tire_clearance <= edge_margin:
        raise ValueError(
            "nominal test corridor does not leave the configured tire edge margin"
        )

    speed_limit = min(float(requested_speed_mps), TEST_SPEED_LIMIT_MPS)
    if speed_limit <= 0.0:
        raise ValueError("test speed must be positive")
    demonstrated = [dict(point) for point in points[:endpoint_index + 1]]
    if len(demonstrated) < 2:
        raise ValueError("short test route has no forward demonstrated segment")
    nominal_points = []
    for index, demonstrated_point in enumerate(demonstrated):
        x, y = _finite_xy(demonstrated_point)
        nominal_points.append({
            "x": x,
            "y": y,
            "source_index": index,
            # These are explicit controlled-area assumptions, not mapped
            # terrain claims. Live terrain and collision monitoring remain on.
            "cross_slope_deg": 0.0,
            "longitudinal_slope_deg": 0.0,
            "tire_edge_clearance_m": tire_clearance,
            "minimum_wheel_edge_m": tire_clearance + tire_half_width,
            "terrain_source": "controlled_level_test_nominal_corridor",
        })
    maximum_smoothing_deviation = min(
        0.08,
        max(0.02, 0.5 * (track_width - hardware.navigation_width_m) / 2.0),
    )
    test_points, centerline_metadata = prepare_centerline(
        nominal_points,
        closed=False,
        spacing_m=0.05,
        maximum_deviation_m=maximum_smoothing_deviation,
    )

    candidate = dict(document)
    candidate.update({
        "schema_version": max(2, int(document.get("schema_version", 1))),
        "closed_loop": False,
        "test_only": True,
        "approval_scope": "controlled_short_autonomy_test",
        "requires_live_terrain": True,
        "allows_level_fallback_without_wheel_cells": True,
        "requires_collision_monitor": True,
        "source_profile": "track_profile.yaml",
        "test_path_conversion": {
            "method": "recorded_route_prefix_bounded_spline",
            "original_point_count": len(points),
            "selected_endpoint_index": endpoint_index,
            "segment_length_m": distance,
            "preserves_demonstrated_geometry": True,
            "assumed_level_surface": True,
            "nominal_track_width_only": True,
            "centerline": centerline_metadata,
        },
        "points": test_points,
    })
    prepared = annotate_profile(
        candidate,
        envelope,
        allow_missing_terrain=False,
        maximum_speed_mps=speed_limit,
    )
    _apply_controlled_curvature_policy(prepared, envelope)
    if not is_autonomy_approved(prepared):
        # A manual mapping lap can contain an in-place pivot/reversal before
        # the operator continues.  The farthest-position prefix then includes
        # a mathematically roka curvature spike even though the earlier
        # demonstrated approach is usable.  Keep fail-closed limits, but retry
        # only the longest prefix ending at the first rejected point.  Making
        # that point the endpoint recomputes its tangent without cutting a
        # chord across unobserved space.
        rejected = [
            index
            for index, point in enumerate(prepared["points"])
            if not (
                point.get("terrain_data_complete") is True
                and point.get("planner_reason") in {
                    "demonstrated_envelope",
                    CONTROLLED_CURVATURE_REASON,
                }
            )
        ]
        if rejected and _allow_segment_fallback:
            prefix_count = int(
                prepared["points"][rejected[0]].get("source_index", rejected[0])
            ) + 1
            if 2 <= prefix_count < len(points):
                prefix_document = dict(document)
                prefix_document["points"] = points[:prefix_count]
                try:
                    fallback = prepare_short_test_document(
                        prefix_document,
                        envelope,
                        requested_speed_mps=speed_limit,
                        _allow_segment_fallback=False,
                    )
                except ValueError:
                    fallback = None
                if fallback is not None:
                    conversion = fallback["test_path_conversion"]
                    conversion.update({
                        "method": "recorded_route_safe_prefix_before_pivot",
                        "original_point_count": len(points),
                        "kept_source_point_count": prefix_count,
                        "discarded_source_point_count": len(points) - prefix_count,
                        "first_rejected_full_prefix_index": rejected[0],
                    })
                    return fallback

            # The spline has more samples than the raw Xbox trace. Convert a
            # continuously accepted spline run back to its retained source
            # index range instead of assuming one-to-one point numbering.
            accepted_reasons = {
                "demonstrated_envelope",
                CONTROLLED_CURVATURE_REASON,
            }
            source_runs: list[tuple[float, int, int]] = []
            run_start: int | None = None
            run_length = 0.0
            for sample_index, point in enumerate(prepared["points"]):
                accepted = (
                    point.get("terrain_data_complete") is True
                    and point.get("planner_reason") in accepted_reasons
                )
                if accepted and run_start is None:
                    run_start = sample_index
                    run_length = 0.0
                if accepted and run_start is not None and sample_index > run_start:
                    before = prepared["points"][sample_index - 1]
                    run_length += math.hypot(
                        float(point["x"]) - float(before["x"]),
                        float(point["y"]) - float(before["y"]),
                    )
                if run_start is not None and (
                    not accepted or sample_index == len(prepared["points"]) - 1
                ):
                    run_end = sample_index if accepted else sample_index - 1
                    if run_end > run_start:
                        source_start = int(
                            prepared["points"][run_start].get("source_index", run_start)
                        )
                        source_end = int(
                            prepared["points"][run_end].get("source_index", run_end)
                        )
                        if source_end > source_start:
                            source_runs.append((run_length, source_start, source_end))
                    run_start = None
                    run_length = 0.0
            source_runs.sort(key=lambda item: (-item[0], item[1]))
            for _, source_start, source_end in source_runs:
                start_x, start_y = _finite_xy(points[source_start])
                end_x, end_y = _finite_xy(points[source_end])
                if math.hypot(end_x - start_x, end_y - start_y) < MINIMUM_TEST_DISPLACEMENT_M:
                    continue
                segment_document = dict(document)
                segment_document["points"] = points[source_start:source_end + 1]
                try:
                    fallback = prepare_short_test_document(
                        segment_document,
                        envelope,
                        requested_speed_mps=speed_limit,
                        _allow_segment_fallback=False,
                    )
                except ValueError:
                    continue
                conversion = fallback["test_path_conversion"]
                conversion.update({
                    "method": "recorded_route_safe_contiguous_spline_segment",
                    "original_point_count": len(points),
                    "selected_start_index": source_start,
                    "selected_endpoint_index": source_end,
                    "kept_source_point_count": source_end - source_start + 1,
                    "discarded_source_point_count": len(points) - (source_end - source_start + 1),
                    "first_rejected_full_prefix_index": rejected[0],
                })
                return fallback

            # A mapping session can start while the operator is still
            # aligning the robot with the test lane.  In that case point zero
            # itself is rejected and the prefix fallback above cannot retain
            # any useful motion, even though a long, continuously
            # demonstrated segment follows it.  Search only within the
            # already evaluated prefix and retain a contiguous accepted run;
            # never bridge a rejected point or draw a new chord through
            # unobserved space.
            if len(prepared["points"]) == endpoint_index + 1:
                accepted_reasons = {
                    "demonstrated_envelope",
                    CONTROLLED_CURVATURE_REASON,
                }
                runs: list[tuple[float, int, int]] = []
                run_start: int | None = None
                for index, point in enumerate(prepared["points"]):
                    accepted = (
                        point.get("terrain_data_complete") is True
                        and point.get("planner_reason") in accepted_reasons
                    )
                    if accepted and run_start is None:
                        run_start = index
                    if run_start is not None and (
                        not accepted or index == len(prepared["points"]) - 1
                    ):
                        run_end = index if accepted else index - 1
                        if run_end > run_start:
                            route_length = 0.0
                            for point_index in range(run_start + 1, run_end + 1):
                                before_x, before_y = _finite_xy(points[point_index - 1])
                                point_x, point_y = _finite_xy(points[point_index])
                                route_length += math.hypot(
                                    point_x - before_x, point_y - before_y
                                )
                            runs.append((route_length, run_start, run_end))
                        run_start = None

                # Prefer the longest safe demonstrated run. Earlier route
                # order breaks ties so preparation remains deterministic.
                runs.sort(key=lambda item: (-item[0], item[1]))
                for _, run_start, run_end in runs:
                    start_x, start_y = _finite_xy(points[run_start])
                    end_x, end_y = _finite_xy(points[run_end])
                    if math.hypot(end_x - start_x, end_y - start_y) < (
                        MINIMUM_TEST_DISPLACEMENT_M
                    ):
                        continue
                    segment_document = dict(document)
                    segment_document["points"] = points[run_start:run_end + 1]
                    try:
                        fallback = prepare_short_test_document(
                            segment_document,
                            envelope,
                            requested_speed_mps=speed_limit,
                            _allow_segment_fallback=False,
                        )
                    except ValueError:
                        continue
                    conversion = fallback["test_path_conversion"]
                    local_endpoint = int(conversion["selected_endpoint_index"])
                    kept_count = local_endpoint + 1
                    conversion.update({
                        "method": "recorded_route_safe_contiguous_segment",
                        "original_point_count": len(points),
                        "selected_start_index": run_start,
                        "selected_endpoint_index": run_start + local_endpoint,
                        "kept_source_point_count": kept_count,
                        "discarded_source_point_count": len(points) - kept_count,
                        "first_rejected_full_prefix_index": rejected[0],
                    })
                    return fallback

        reason_counts: dict[str, int] = {}
        for point in prepared["points"]:
            reason = str(point.get("planner_reason", "unknown"))
            if reason not in {
                "demonstrated_envelope",
                CONTROLLED_CURVATURE_REASON,
            }:
                reason_counts[reason] = reason_counts.get(reason, 0) + 1
        detail = ", ".join(
            f"{reason}={count}" for reason, count in sorted(reason_counts.items())
        )
        raise ValueError(
            "short test segment is outside the demonstrated curvature/risk envelope"
            + (f" ({detail})" if detail else "")
        )
    prepared["test_only"] = True
    prepared["approval_scope"] = "controlled_short_autonomy_test"
    prepared["approval_reason"] = (
        "controlled_level_short_test_with_fresh_lidar_and_collision_monitor"
    )
    prepared["test_speed_limit_mps"] = speed_limit
    return prepared


def prepare_short_test_file(
    source_profile: Path,
    output_profile: Path,
    envelope_path: Path,
    requested_speed_mps: float = TEST_SPEED_LIMIT_MPS,
) -> dict[str, Any]:
    document = yaml.safe_load(source_profile.read_text(encoding="utf-8")) or {}
    envelope = json.loads(envelope_path.read_text(encoding="utf-8"))
    prepared = prepare_short_test_document(document, envelope, requested_speed_mps)
    output_profile.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_profile.with_suffix(output_profile.suffix + ".preparing")
    temporary.write_text(
        yaml.safe_dump(prepared, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    temporary.replace(output_profile)
    return prepared
