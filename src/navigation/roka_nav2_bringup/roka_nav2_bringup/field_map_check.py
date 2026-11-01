"""Validate or strictly prepare the latest real PK-3A map for autonomy."""

from __future__ import annotations

import argparse
import copy
import math
from pathlib import Path
import sys

import yaml

from roka_nav2_bringup.map_bundle_selection import (
    latest_valid_map_bundle,
    map_bundle_for_id,
    occupancy_map_is_valid,
)
from roka_nav2_bringup.map_bundle_quality import (
    bundle_is_complete,
    requires_embedded_sensor_bag,
    validate_bundle,
)
from roka_nav2_bringup.profile_validation import is_autonomy_approved
from roka_nav2_bringup.map_review import active_reviewed_occupancy
from roka_nav2_bringup.short_test_profile import prepare_short_test_file
from roka_nav2_bringup.vehicle_aware_path import write_vehicle_aware_profile
from roka_nav2_bringup.route_geometry import prepare_centerline
from roka_robot_description.workspace import workspace_path


RECORDED_ROUTE_MAX_YAW_RATE_RADPS = 0.60
RECORDED_ROUTE_MAX_LATERAL_ACCELERATION_MPS2 = 0.30
RECORDED_ROUTE_MINIMUM_CURVE_SPEED_MPS = 0.05


def select_bundle(map_root: Path, bundle: str, map_id: str = "auto") -> Path | None:
    if map_id.strip().lower() != "auto":
        return map_bundle_for_id(map_root, map_id)
    if bundle.strip().lower() == "auto":
        return latest_valid_map_bundle(map_root)
    return Path(bundle).expanduser().resolve()


def profile_problems(document: dict) -> list[str]:
    problems = []
    points = document.get("points", [])
    if not isinstance(document.get("closed_loop"), bool):
        problems.append("track_profile closed_loop must be boolean")
    if not isinstance(points, list) or len(points) < 10:
        problems.append("track_profile needs at least 10 points")
        return problems
    required = (
        "cross_slope_deg",
        "longitudinal_slope_deg",
        "tire_edge_clearance_m",
        "speed_mps",
    )

    def finite_field(point, name):
        try:
            return name in point and math.isfinite(float(point[name]))
        except (TypeError, ValueError):
            return False

    for name in required:
        missing = sum(
            not finite_field(point, name)
            for point in points
        )
        if missing:
            problems.append(f"{missing}/{len(points)} points lack finite {name}")
    if not is_autonomy_approved(document):
        problems.append(
            "autonomy_approved is not boolean true: "
            + str(document.get("approval_reason", "no approval reason"))
        )
    return problems


def load_yaml(path: Path) -> dict:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"cannot read {path}: {error}") from error
    if not isinstance(document, dict):
        raise ValueError(f"{path} is not a YAML mapping")
    return document


def prepare_recorded_route_file(
    source_profile_path: Path,
    output_profile_path: Path,
    maximum_speed: float,
) -> dict:
    """Create a measured-route replay with a common straight-line speed cap."""
    if not math.isfinite(maximum_speed) or not 0.03 <= maximum_speed <= 0.60:
        raise ValueError("recorded-route speed must be between 0.03 and 0.60 m/s")
    source = load_yaml(source_profile_path)
    source_points = source.get("points", [])
    if not isinstance(source_points, list) or len(source_points) < 10:
        raise ValueError("recorded route needs at least 10 points")

    measured_points = []
    for index, source_point in enumerate(source_points):
        if not isinstance(source_point, dict):
            raise ValueError(f"recorded route point {index} is not a mapping")
        try:
            x = float(source_point["x"])
            y = float(source_point["y"])
            yaw = float(source_point.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"recorded route point {index} is invalid") from error
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError(f"recorded route point {index} is non-finite")
        point = copy.deepcopy(source_point)
        point.update({"x": x, "y": y, "yaw": yaw, "speed_mps": maximum_speed})
        measured_points.append(point)

    points, centerline_geometry = prepare_centerline(
        measured_points,
        closed=bool(source.get("closed_loop", False)),
        spacing_m=0.05,
        maximum_deviation_m=0.05,
        smoothing_iterations=3,
    )
    for point in points:
        curvature = abs(float(point.get("curvature_radpm", 0.0)))
        if curvature <= 1.0e-6:
            curve_speed = maximum_speed
        else:
            curve_speed = min(
                maximum_speed,
                RECORDED_ROUTE_MAX_YAW_RATE_RADPS / curvature,
                math.sqrt(
                    RECORDED_ROUTE_MAX_LATERAL_ACCELERATION_MPS2 / curvature
                ),
            )
        if curve_speed + 1.0e-9 < RECORDED_ROUTE_MINIMUM_CURVE_SPEED_MPS:
            raise ValueError(
                "recorded route contains curvature that cannot be followed "
                "within the configured yaw-rate limit"
            )
        point["speed_mps"] = max(
            RECORDED_ROUTE_MINIMUM_CURVE_SPEED_MPS, curve_speed
        )

    prepared = copy.deepcopy(source)
    prepared.update({
        "points": points,
        "default_speed_mps": maximum_speed,
        "minimum_speed_mps": min(float(point["speed_mps"]) for point in points),
        "maximum_speed_mps": maximum_speed,
        "profile_role": "recorded_route_replay",
        "approval_scope": "operator_requested_recorded_route_replay",
        "autonomy_approved": True,
        "approval_reason": "recorded_geometry_curvature_limited_replay",
        "recorded_route_replay": True,
        "source_profile": source_profile_path.name,
        # Millimetre-scale joystick/odometry jitter made the pure-pursuit
        # controller steer left-right around a visually straight route. Keep
        # the route inside a strict 5 cm tube but follow its bounded centerline.
        "preserve_recorded_geometry": False,
        "bounded_route_smoothing": True,
        "centerline_geometry": centerline_geometry,
        "recorded_route_speed_policy": {
            "maximum_linear_speed_mps": maximum_speed,
            "maximum_yaw_rate_radps": RECORDED_ROUTE_MAX_YAW_RATE_RADPS,
            "maximum_lateral_acceleration_mps2": (
                RECORDED_ROUTE_MAX_LATERAL_ACCELERATION_MPS2
            ),
            "minimum_curve_speed_mps": RECORDED_ROUTE_MINIMUM_CURVE_SPEED_MPS,
        },
        "test_only": False,
    })
    temporary = output_profile_path.with_suffix(output_profile_path.suffix + ".tmp")
    temporary.write_text(
        yaml.safe_dump(prepared, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    temporary.replace(output_profile_path)
    return prepared


def recorded_profile_problems(document: dict) -> list[str]:
    problems = []
    if document.get("profile_role") != "recorded_route_replay":
        problems.append("recorded replay requires profile_role=recorded_route_replay")
    if document.get("recorded_route_replay") is not True:
        problems.append("recorded_route_replay is not boolean true")
    if document.get("bounded_route_smoothing") is not True:
        problems.append("recorded replay has no bounded centerline smoothing evidence")
    if not is_autonomy_approved(document):
        problems.append("autonomy_approved is not boolean true")
    points = document.get("points", [])
    if not isinstance(points, list) or len(points) < 10:
        problems.append("recorded route needs at least 10 points")
        return problems
    for index, point in enumerate(points):
        try:
            values = (
                float(point["x"]),
                float(point["y"]),
                float(point.get("yaw", 0.0)),
                float(point["speed_mps"]),
            )
        except (KeyError, TypeError, ValueError):
            problems.append(f"recorded route point {index} is invalid")
            continue
        if not all(math.isfinite(value) for value in values) or values[-1] <= 0.0:
            problems.append(f"recorded route point {index} has invalid values")
    return problems


def prepare_profile(
    source_profile_path: Path,
    output_profile_path: Path,
    envelope_path: Path,
    maximum_speed: float,
) -> None:
    try:
        prepared = write_vehicle_aware_profile(
            source_profile_path,
            output_profile_path,
            source_profile_path.parent / "map.pcd",
            envelope_path,
            maximum_speed,
        )
    except (OSError, ValueError, TypeError, KeyError) as error:
        hint = ""
        if "no mapped full-vehicle surface candidate" in str(error):
            hint = (
                "; this means the PCD has no local floor returns, not that the "
                "physical floor is absent. For a declared controlled flat test, "
                "use --prepare-test; keep --prepare for a measured slope track"
            )
        raise ValueError(
            f"strict traversability preparation failed: {error}{hint}"
        ) from error
    if not is_autonomy_approved(prepared):
        raise ValueError(
            "strict traversability preparation did not approve every path point"
        )
    if prepared.get("profile_role") != "safe_autonomy":
        raise ValueError("strict preparation did not create a safe-autonomy profile")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--map-root", type=Path, default=workspace_path("maps")
    )
    parser.add_argument("--bundle", default="auto")
    parser.add_argument(
        "--map-id",
        default="auto",
        help="named bundle under --map-root, for example section_1",
    )
    preparation = parser.add_mutually_exclusive_group()
    preparation.add_argument("--prepare", action="store_true")
    preparation.add_argument(
        "--prepare-test",
        action="store_true",
        help=(
            "create track_profile_test.yaml for an explicitly controlled, "
            "short, low-speed autonomy test"
        ),
    )
    preparation.add_argument(
        "--prepare-recorded",
        action="store_true",
        help="preserve the complete measured route and set one replay speed",
    )
    parser.add_argument(
        "--test-profile",
        action="store_true",
        help="validate track_profile_test.yaml instead of the normal track profile",
    )
    parser.add_argument(
        "--recorded-profile",
        action="store_true",
        help="validate track_profile_recorded.yaml",
    )
    parser.add_argument(
        "--envelope-file",
        type=Path,
        default=workspace_path(
            "models", "traversability", "traversability_envelope_v2.json"
        ),
    )
    parser.add_argument("--maximum-speed-mps", type=float, default=0.30)
    args = parser.parse_args(argv)

    try:
        bundle = select_bundle(
            args.map_root.expanduser().resolve(), args.bundle, args.map_id
        )
    except ValueError as error:
        print(f"[MAP-CHECK][FAIL] {error}", file=sys.stderr)
        raise SystemExit(1)
    if bundle is None:
        print("[MAP-CHECK][FAIL] no valid occupancy map bundle exists", file=sys.stderr)
        raise SystemExit(1)
    print(f"[MAP-CHECK] bundle={bundle}")
    if not bundle_is_complete(bundle):
        print(
            "[MAP-CHECK][FAIL] bundle has no validated completion manifest",
            file=sys.stderr,
        )
        raise SystemExit(1)
    if not occupancy_map_is_valid(bundle / "occupancy.yaml"):
        print("[MAP-CHECK][FAIL] occupancy map/image is invalid", file=sys.stderr)
        raise SystemExit(1)
    try:
        reviewed_occupancy = active_reviewed_occupancy(bundle, require_approved=True)
        print(f"[MAP-CHECK][PASS] reviewed occupancy: {reviewed_occupancy.name}")
    except (OSError, TypeError, ValueError, yaml.YAMLError) as error:
        print(f"[MAP-CHECK][FAIL] {error}", file=sys.stderr)
        raise SystemExit(1)
    try:
        quality = validate_bundle(
            bundle,
            require_sensor_bag=requires_embedded_sensor_bag(bundle),
        )
        backend_detail = (
            f", pcd={quality['pointcloud_points']} points"
            if "pointcloud_points" in quality else ""
        )
        print(
            "[MAP-CHECK][PASS] bundle quality: "
            f"backend={quality['mapping_backend']}, "
            f"path={quality['track']['path_length_m']:.2f}m"
            f"{backend_detail}"
        )
    except ValueError as error:
        print(f"[MAP-CHECK][FAIL] {error}", file=sys.stderr)
        raise SystemExit(1)

    source_profile_path = bundle / "track_profile.yaml"
    safe_profile_path = bundle / "track_profile_safe.yaml"
    if args.recorded_profile or args.prepare_recorded:
        profile_path = bundle / "track_profile_recorded.yaml"
    elif args.test_profile or args.prepare_test:
        profile_path = bundle / "track_profile_test.yaml"
    else:
        profile_path = safe_profile_path
    if args.prepare:
        try:
            prepare_profile(
                source_profile_path,
                safe_profile_path,
                args.envelope_file.expanduser().resolve(),
                args.maximum_speed_mps,
            )
            print(
                "[MAP-CHECK][PASS] vehicle-aware safe profile prepared: "
                f"{safe_profile_path}"
            )
        except ValueError as error:
            print(f"[MAP-CHECK][FAIL] {error}", file=sys.stderr)
            raise SystemExit(1)
    elif args.prepare_test:
        try:
            prepared = prepare_short_test_file(
                source_profile_path,
                profile_path,
                args.envelope_file.expanduser().resolve(),
                args.maximum_speed_mps,
            )
            conversion = prepared["test_path_conversion"]
            source_range = ""
            if "selected_start_index" in conversion:
                source_range = (
                    "; source-points="
                    f"{conversion['selected_start_index']}.."
                    f"{conversion['selected_endpoint_index']}"
                )
            print(
                "[MAP-CHECK][PASS] controlled short test profile prepared: "
                f"{profile_path}; length={conversion['segment_length_m']:.3f} m; "
                f"speed<={prepared['test_speed_limit_mps']:.2f} m/s"
                f"{source_range}"
            )
        except (OSError, ValueError, TypeError, KeyError) as error:
            print(f"[MAP-CHECK][FAIL] test preparation failed: {error}", file=sys.stderr)
            raise SystemExit(1)
    elif args.prepare_recorded:
        try:
            prepared = prepare_recorded_route_file(
                source_profile_path,
                profile_path,
                args.maximum_speed_mps,
            )
            print(
                "[MAP-CHECK][PASS] recorded route replay prepared: "
                f"{profile_path}; points={len(prepared['points'])}; "
                f"speed<={args.maximum_speed_mps:.2f} m/s"
            )
        except (OSError, ValueError, TypeError, KeyError) as error:
            print(
                f"[MAP-CHECK][FAIL] recorded replay preparation failed: {error}",
                file=sys.stderr,
            )
            raise SystemExit(1)

    try:
        profile_document = load_yaml(profile_path)
        problems = (
            recorded_profile_problems(profile_document)
            if args.recorded_profile or args.prepare_recorded
            else profile_problems(profile_document)
        )
        if (
            not (
                args.test_profile or args.prepare_test
                or args.recorded_profile or args.prepare_recorded
            )
            and profile_document.get("profile_role") != "safe_autonomy"
        ):
            problems.append("normal autonomy requires profile_role=safe_autonomy")
    except ValueError as error:
        problems = [str(error)]
    if problems:
        for problem in problems:
            print(f"[MAP-CHECK][FAIL] {problem}", file=sys.stderr)
        print(
            "[MAP-CHECK][RESULT] NOT READY; autonomy launch remains locked",
            file=sys.stderr,
        )
        raise SystemExit(1)
    document = load_yaml(profile_path)
    if document.get("recorded_route_replay") is True:
        print(
            "[MAP-CHECK][RESULT] READY for recorded-route replay; "
            "live obstacle and terrain decisions are not part of this mode"
        )
    elif document.get("test_only") is True:
        print(
            "[MAP-CHECK][RESULT] READY for controlled TEST autonomy only; "
            "normal track autonomy remains separate"
        )
    else:
        print("[MAP-CHECK][RESULT] READY for guarded autonomy")


if __name__ == "__main__":
    main()
