"""Write one concise, persistent operator report for an ROKA map section."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import math
from pathlib import Path
from typing import Any

import yaml

from roka_nav2_bringup.map_bundle_quality import (
    bundle_is_complete,
    requires_embedded_sensor_bag,
    validate_bundle,
)
from roka_nav2_bringup.map_review import active_reviewed_occupancy
from roka_nav2_bringup.profile_validation import (
    is_profile_allowed,
    is_test_only_profile,
)
from roka_robot_description.workspace import workspace_root


DEFAULT_WORKSPACE = workspace_root()


def _load_yaml(path: Path) -> dict[str, Any]:
    document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(document, dict):
        raise ValueError(f"YAML root is not a mapping: {path}")
    return document


def _split_problems(error: Exception) -> list[str]:
    return [item.strip() for item in str(error).split(";") if item.strip()]


def _finite_max(points: list[dict[str, Any]], name: str) -> float | None:
    values = []
    for point in points:
        try:
            value = float(point[name])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            values.append(value)
    return max(values) if values else None


def analyze_section(
    workspace: Path,
    map_id: str,
    *,
    scope: str = "test",
    map_root: Path | None = None,
) -> dict[str, Any]:
    workspace = workspace.expanduser().resolve()
    workspace_maps = (workspace / "maps").resolve()
    selected_root = (
        Path(map_root).expanduser().resolve()
        if map_root is not None
        else workspace_maps
    )
    if selected_root != workspace_maps and workspace_maps not in selected_root.parents:
        raise ValueError(f"map root is outside workspace maps: {selected_root}")
    bundle = (selected_root / map_id).resolve()
    if bundle.parent != selected_root:
        raise ValueError(f"invalid map id: {map_id}")
    if scope not in {"map", "test", "recorded"}:
        raise ValueError(f"invalid status scope: {scope}")

    report: dict[str, Any] = {
        "schema_version": 2,
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "map_id": map_id,
        "map_profile": (
            "legacy"
            if selected_root == workspace_maps
            else selected_root.relative_to(workspace_maps).as_posix()
        ),
        "bundle": bundle.relative_to(workspace).as_posix(),
        "scope": scope,
        "problems": [],
        "warnings": [],
    }
    problems: list[str] = report["problems"]
    warnings: list[str] = report["warnings"]

    if not bundle.is_dir():
        problems.append("저장된 지도 섹션이 없습니다")
        report["result"] = "NOT_READY"
        return report
    if not bundle_is_complete(bundle):
        problems.append("지도 완료 manifest 또는 파일 해시가 유효하지 않습니다")

    metrics: dict[str, Any] = {}
    try:
        metrics = validate_bundle(
            bundle,
            require_sensor_bag=requires_embedded_sensor_bag(bundle),
        )
    except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
        problems.extend(_split_problems(error))
        manifest = bundle / "bundle_manifest.yaml"
        if manifest.is_file():
            try:
                metrics = _load_yaml(manifest).get("quality", {}) or {}
            except (OSError, TypeError, ValueError, yaml.YAMLError):
                pass

    backend = str(metrics.get("mapping_backend", "unknown"))
    occupancy = metrics.get("occupancy", {}) or {}
    track = metrics.get("track", {}) or {}
    alignment = metrics.get("path_occupancy_alignment", {}) or {}
    session = metrics.get("session", {}) or {}
    report["map"] = {
        "backend": backend,
        "size_mib": round(float(metrics.get("bundle_size_bytes", 0)) / 1024 ** 2, 2),
        "occupancy_width": occupancy.get("width"),
        "occupancy_height": occupancy.get("height"),
        "occupied_component_count": occupancy.get("occupied_component_count"),
        "occupied_fragment_component_count": occupancy.get(
            "occupied_fragment_component_count"
        ),
        "occupied_ignored_small_component_count": occupancy.get(
            "occupied_ignored_small_component_count"
        ),
        "path_points": track.get("point_count"),
        "path_length_m": track.get("path_length_m"),
        "closed_loop": track.get("closed_loop"),
        "closure_distance_m": track.get("closure_distance_m"),
        "enclosed_area_m2": track.get("enclosed_area_m2"),
        "centre_free_fraction": alignment.get("centre_free_fraction"),
        "wheel_lane_free_fraction": alignment.get("wheel_lane_free_fraction"),
        "map_odom_translation_jump_m": session.get(
            "maximum_map_odom_translation_jump_m"
        ),
        "map_odom_yaw_jump_deg": session.get("maximum_map_odom_yaw_jump_deg"),
    }

    contract = metrics.get("sensor_contract", {}) or {}
    if not contract:
        contract_path = bundle / "sensor_contract.yaml"
        if contract_path.is_file():
            try:
                contract = _load_yaml(contract_path)
            except (OSError, TypeError, ValueError, yaml.YAMLError):
                contract = {}
    topics = contract.get("topics", {}) or {}
    diagnostics = contract.get("diagnostics", {}) or {}
    contract_key = f"{report['map_profile']}_{map_id}".replace("/", "_")
    detailed_contract_path = workspace / "benchmarks" / f"{contract_key}_sensor_contract.yaml"
    if detailed_contract_path.is_file():
        try:
            detailed_contract = _load_yaml(detailed_contract_path)
            if (
                int(detailed_contract.get("schema_version", 0)) >= 3
                and detailed_contract.get("bag_fingerprint")
                == contract.get("bag_fingerprint")
            ):
                diagnostics = detailed_contract.get("diagnostics", {}) or diagnostics
        except (OSError, TypeError, ValueError, yaml.YAMLError):
            pass
    report["sensors"] = {
        "result": contract.get("result", "UNKNOWN"),
        "vlp16_rate_hz": (topics.get("/velodyne_packets", {}) or {}).get("rate_hz"),
        "imu_rate_hz": (topics.get("/imu/data", {}) or {}).get("rate_hz"),
        "wheel_odom_rate_hz": (topics.get("/wheel/odom", {}) or {}).get("rate_hz"),
        "lidar_imu_p95_ms": (
            contract.get("lidar_imu_timing", {}) or {}
        ).get("nearest_absolute_difference_p95_ms"),
        "diagnostic_error_count": diagnostics.get("error_status_count", 0),
        "diagnostic_issues": diagnostics.get("issues", []),
        "terrain_diagnostic_extrema": diagnostics.get("terrain_extrema", {}),
        "terrain_extrema_locations": diagnostics.get(
            "terrain_extrema_locations", {}
        ),
    }
    if contract.get("result") not in {None, "PASS"}:
        problems.extend(str(item) for item in contract.get("failures", []))
    diagnostic_errors = int(diagnostics.get("error_status_count", 0) or 0)
    if diagnostic_errors:
        if diagnostics.get("issues"):
            issues = diagnostics["issues"]
            terrain_errors = sum(
                int(item.get("count", 0) or 0)
                for item in issues
                if item.get("name") == "terrain_supervisor/safety"
                and int(item.get("level", 0) or 0) >= 2
            )
            ekf_errors = sum(
                int(item.get("count", 0) or 0)
                for item in issues
                if str(item.get("name", "")).startswith("ekf_filter_node")
                and int(item.get("level", 0) or 0) >= 2
            )
            extrema = diagnostics.get("terrain_extrema", {}) or {}
            warnings.append(
                f"구형 기록 진단: 관찰용 terrain ERROR {terrain_errors}건, "
                f"EKF Z설정 ERROR {ekf_errors}건; 다음 매핑 설정부터 반복 원인을 수정했습니다"
            )
            if extrema:
                locations = diagnostics.get("terrain_extrema_locations", {}) or {}
                attitude_location = locations.get(
                    "maximum_abs_relative_pitch_deg",
                    locations.get("maximum_abs_relative_roll_deg", {}),
                )
                location_text = ""
                if attitude_location:
                    location_text = (
                        f", 경로 누적점={attitude_location.get('recorded_path_points')}, "
                        f"매핑 {attitude_location.get('mapping_elapsed_sec')}초"
                    )
                warnings.append(
                    "기록 자세 최대값 확인 필요: "
                    f"|roll|={_fmt(extrema.get('maximum_abs_relative_roll_deg'), 1, ' deg')}, "
                    f"|pitch|={_fmt(extrema.get('maximum_abs_relative_pitch_deg'), 1, ' deg')}, "
                    f"risk={_fmt(extrema.get('maximum_risk'), 2)}{location_text}"
                )
        else:
            warnings.append(
                f"구형 audit에 진단 ERROR/STALE {diagnostic_errors}건이 있으나 "
                "원인별 분류가 없습니다; roka audit로 상세 확인"
            )

    if backend == "slam_toolbox_2d":
        warnings.append(
            "같은 원본 수집에서 만든 2D 주행 지도입니다; 경사 판정은 함께 처리한 3D 지도를 확인하십시오"
        )

    surface_path = bundle / "surface_extraction.yaml"
    if surface_path.is_file():
        try:
            surface = _load_yaml(surface_path)
            report["drivable_surface"] = {
                "result": surface.get("result", "UNKNOWN"),
                "cells": surface.get("surface_cells"),
                "route_coverage": surface.get("route_station_coverage"),
                "slope_p95_deg": surface.get("neighbour_slope_p95_deg"),
                "z_min_m": surface.get("surface_z_min_m"),
                "z_max_m": surface.get("surface_z_max_m"),
            }
            if surface.get("result") != "PASS":
                problems.append("주행경로 연결 지면 추출이 통과하지 못했습니다")
        except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
            problems.append(f"주행경로 연결 지면 결과를 읽을 수 없습니다: {error}")
    elif backend in {"fastlio_sc_pgo", "fastlio_sc_pgo_3d"}:
        problems.append("3D 지도에 주행경로 연결 지면(surface.pcd)이 없습니다")

    localization_path = bundle / "localization_regression.yaml"
    if localization_path.is_file():
        try:
            localization = _load_yaml(localization_path)
            report["localization_regression"] = {
                "result": localization.get("result", "UNKNOWN"),
                "ready": localization.get("ready", False),
                "fitness": str(localization.get("status", "")).partition(
                    "fitness="
                )[2]
                or None,
                "pose_count": localization.get("pose_count"),
                "observation_sec": localization.get("observation_sec"),
                "maximum_linear_rate_mps": localization.get(
                    "maximum_linear_rate_mps"
                ),
                "maximum_yaw_rate_radps": localization.get(
                    "maximum_yaw_rate_radps"
                ),
            }
            if localization.get("result") != "PASS" or not localization.get(
                "ready", False
            ):
                warnings.append("저장 3D 지도 위치추정 회귀시험이 통과하지 못했습니다")
        except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
            warnings.append(f"3D 위치추정 회귀시험 결과를 읽을 수 없습니다: {error}")

    if scope in {"test", "recorded"}:
        try:
            reviewed = active_reviewed_occupancy(bundle, require_approved=True)
            report["reviewed_occupancy"] = reviewed.name
        except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
            problems.append(f"지도 승인이 없거나 만료되었습니다: {error}")

    if scope == "test":
        profile_path = bundle / "track_profile_test.yaml"
        if not profile_path.is_file():
            problems.append("시험 경로가 없습니다; roka auto가 자동 생성할 수 있습니다")
        else:
            try:
                profile = _load_yaml(profile_path)
                points = profile.get("points", [])
                conversion = profile.get("test_path_conversion", {}) or {}
                approved = (
                    is_test_only_profile(profile)
                    and is_profile_allowed(profile, allow_test_profile=True)
                    and isinstance(points, list)
                    and len(points) >= 10
                )
                report["test_path"] = {
                    "approved": approved,
                    "points": len(points) if isinstance(points, list) else 0,
                    "length_m": conversion.get("segment_length_m"),
                    "speed_limit_mps": profile.get("test_speed_limit_mps"),
                    "selection_method": conversion.get("method"),
                    "source_start_index": conversion.get("selected_start_index", 0),
                    "source_end_index": conversion.get("selected_endpoint_index"),
                    "source_point_count": conversion.get("original_point_count"),
                    "maximum_curvature_radpm": _finite_max(points, "curvature_radpm"),
                    "maximum_risk": _finite_max(points, "demonstration_risk"),
                }
                if not approved:
                    problems.append("시험 경로가 명시적으로 승인된 test-only 경로가 아닙니다")
                start = int(conversion.get("selected_start_index", 0) or 0)
                original_count = int(conversion.get("original_point_count", 0) or 0)
                kept = int(conversion.get("kept_source_point_count", len(points)) or 0)
                if start > 0 or (original_count and kept < original_count):
                    warnings.append(
                        f"수동 기록 전체가 아니라 안전 연속 구간만 사용합니다: "
                        f"원본 점 {conversion.get('selected_start_index', 0)}.."
                        f"{conversion.get('selected_endpoint_index')}"
                    )
                locations = diagnostics.get("terrain_extrema_locations", {}) or {}
                attitude_location = locations.get(
                    "maximum_abs_relative_pitch_deg",
                    locations.get("maximum_abs_relative_roll_deg", {}),
                )
                attitude_point = int(
                    attitude_location.get("recorded_path_points", 0) or 0
                )
                selected_end = int(
                    conversion.get("selected_endpoint_index", -1) or -1
                )
                if attitude_point > selected_end >= 0:
                    warnings.append(
                        f"최대 자세 이상 구간(누적점 {attitude_point})은 "
                        f"현재 시험구간 끝점 {selected_end} 밖이라 시험 경로에서 제외됩니다"
                    )
            except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
                problems.append(f"시험 경로를 읽을 수 없습니다: {error}")

    if scope == "recorded":
        profile_path = bundle / "track_profile_recorded.yaml"
        if not profile_path.is_file():
            problems.append(
                "직접 기록경로가 없습니다; roka auto가 자동 생성할 수 있습니다"
            )
        else:
            try:
                profile = _load_yaml(profile_path)
                points = profile.get("points", [])
                approved = (
                    profile.get("profile_role") == "recorded_route_replay"
                    and profile.get("recorded_route_replay") is True
                    and (
                        profile.get("preserve_recorded_geometry") is True
                        or profile.get("bounded_route_smoothing") is True
                    )
                    and is_profile_allowed(profile, allow_test_profile=False)
                    and isinstance(points, list)
                    and len(points) >= 10
                )
                length = 0.0
                if isinstance(points, list):
                    for before, after in zip(points, points[1:]):
                        length += math.hypot(
                            float(after["x"]) - float(before["x"]),
                            float(after["y"]) - float(before["y"]),
                        )
                speeds = [
                    float(point["speed_mps"])
                    for point in points
                    if isinstance(point, dict) and "speed_mps" in point
                ]
                report["recorded_path"] = {
                    "approved": approved,
                    "points": len(points) if isinstance(points, list) else 0,
                    "length_m": length,
                    "minimum_speed_mps": min(speeds) if speeds else None,
                    "maximum_speed_mps": max(speeds) if speeds else None,
                    "closed_loop": profile.get("closed_loop", False),
                    "obstacle_terrain_decisions": False,
                }
                if not approved:
                    problems.append(
                        "직접 기록경로가 전체 경로 재생으로 승인되지 않았습니다"
                    )
            except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as error:
                problems.append(f"직접 기록경로를 읽을 수 없습니다: {error}")

    report["result"] = "READY" if not problems else "NOT_READY"
    return report


def _fmt(value: Any, digits: int = 2, suffix: str = "") -> str:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return "n/a"
    return f"{numeric:.{digits}f}{suffix}"


def print_report(report: dict[str, Any], report_path: Path) -> None:
    map_data = report.get("map", {}) or {}
    sensors = report.get("sensors", {}) or {}
    path_state = "PASS" if map_data.get("path_points") else "FAIL"
    print(
        f"[CHECK][MAP] {path_state} {report['map_id']} | "
        f"{map_data.get('backend', 'unknown')} | "
        f"{map_data.get('occupancy_width', 'n/a')}x"
        f"{map_data.get('occupancy_height', 'n/a')} | "
        f"{_fmt(map_data.get('size_mib'), 1, ' MiB')}"
    )
    print(
        f"[CHECK][SENSORS] {sensors.get('result', 'UNKNOWN')} | "
        f"VLP={_fmt(sensors.get('vlp16_rate_hz'), 1, ' Hz')} | "
        f"IMU={_fmt(sensors.get('imu_rate_hz'), 1, ' Hz')} | "
        f"ODOM={_fmt(sensors.get('wheel_odom_rate_hz'), 1, ' Hz')} | "
        f"LiDAR-IMU p95={_fmt(sensors.get('lidar_imu_p95_ms'), 1, ' ms')}"
    )
    print(
        f"[CHECK][PATH] {path_state} | points={map_data.get('path_points', 'n/a')} | "
        f"length={_fmt(map_data.get('path_length_m'), 2, ' m')} | "
        f"closed={map_data.get('closed_loop', 'n/a')} | "
        f"closure={_fmt(map_data.get('closure_distance_m'), 3, ' m')} | "
        f"map-jump={_fmt(map_data.get('map_odom_translation_jump_m'), 3, ' m')}/"
        f"{_fmt(map_data.get('map_odom_yaw_jump_deg'), 2, ' deg')}"
    )
    test_path = report.get("test_path")
    if test_path:
        state = "READY" if test_path.get("approved") else "FAIL"
        print(
            f"[CHECK][TEST] {state} | "
            f"length={_fmt(test_path.get('length_m'), 3, ' m')} | "
            f"speed<={_fmt(test_path.get('speed_limit_mps'), 2, ' m/s')} | "
            f"source={test_path.get('source_start_index')}.."
            f"{test_path.get('source_end_index')}"
        )
    recorded_path = report.get("recorded_path")
    if recorded_path:
        state = "READY" if recorded_path.get("approved") else "FAIL"
        print(
            f"[CHECK][RECORDED] {state} | "
            f"points={recorded_path.get('points')} | "
            f"length={_fmt(recorded_path.get('length_m'), 2, ' m')} | "
            f"speed={_fmt(recorded_path.get('minimum_speed_mps'), 2)}.."
            f"{_fmt(recorded_path.get('maximum_speed_mps'), 2, ' m/s')} | "
            "obstacle/terrain=OFF"
        )
    surface = report.get("drivable_surface")
    if surface:
        state = "PASS" if surface.get("result") == "PASS" else "FAIL"
        coverage = surface.get("route_coverage")
        coverage_text = (
            f"{100.0 * float(coverage):.1f}%" if coverage is not None else "n/a"
        )
        print(
            f"[CHECK][3D-SURFACE] {state} | "
            f"cells={surface.get('cells', 'n/a')} | coverage={coverage_text} | "
            f"slope-p95={_fmt(surface.get('slope_p95_deg'), 1, ' deg')} | "
            f"z={_fmt(surface.get('z_min_m'), 2, ' m')}.."
            f"{_fmt(surface.get('z_max_m'), 2, ' m')}"
        )
    localization = report.get("localization_regression")
    if localization:
        state = (
            "PASS"
            if localization.get("result") == "PASS"
            and localization.get("ready") is True
            else "FAIL"
        )
        print(
            f"[CHECK][3D-LOCALIZATION] {state} | "
            f"poses={localization.get('pose_count', 'n/a')} | "
            f"duration={_fmt(localization.get('observation_sec'), 1, ' s')} | "
            f"fitness={_fmt(localization.get('fitness'), 4)} | "
            f"rate={_fmt(localization.get('maximum_linear_rate_mps'), 2, ' m/s')}/"
            f"{_fmt(localization.get('maximum_yaw_rate_radps'), 2, ' rad/s')}"
        )
    for warning in report.get("warnings", []):
        print(f"[CHECK][WARN] {warning}")
    for problem in report.get("problems", []):
        print(f"[CHECK][FAIL] {problem}")
    print(f"[CHECK][RESULT] {report['result']} | report={report_path}")


def write_report(report: dict[str, Any], workspace: Path) -> Path:
    report_dir = workspace / "reports"
    report_dir.mkdir(parents=True, exist_ok=True)
    path = report_dir / "latest_status.yaml"
    temporary = report_dir / ".latest_status.yaml.preparing"
    temporary.write_text(
        yaml.safe_dump(report, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    temporary.replace(path)
    history = report_dir / "roka_history.log"
    with history.open("a", encoding="utf-8") as stream:
        stream.write(
            f"{report['generated_utc']} check {report['map_id']} "
            f"scope={report['scope']} result={report['result']} "
            f"problems={len(report['problems'])} warnings={len(report['warnings'])}\n"
        )
    return path


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=DEFAULT_WORKSPACE)
    parser.add_argument(
        "--map-root",
        type=Path,
        default=None,
        help="profile-specific root such as maps/flat or maps/track",
    )
    parser.add_argument("--map-id", default="section_1")
    parser.add_argument(
        "--scope", choices=("map", "test", "recorded"), default="recorded"
    )
    args = parser.parse_args(argv)
    workspace = args.workspace.expanduser().resolve()
    try:
        report = analyze_section(
            workspace,
            args.map_id,
            scope=args.scope,
            map_root=args.map_root,
        )
    except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
        report = {
            "schema_version": 1,
            "generated_utc": datetime.now(timezone.utc).isoformat(),
            "map_id": args.map_id,
            "scope": args.scope,
            "problems": [str(error)],
            "warnings": [],
            "result": "NOT_READY",
        }
    path = write_report(report, workspace)
    print_report(report, path)
    raise SystemExit(0 if report["result"] == "READY" else 1)


if __name__ == "__main__":
    main()
