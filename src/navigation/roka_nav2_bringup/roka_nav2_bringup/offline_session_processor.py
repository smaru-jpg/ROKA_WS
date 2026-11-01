"""Process one captured field session without modifying its approved map."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Sequence

import yaml

from ament_index_python.packages import get_package_share_directory

from .map_bundle_quality import promote_bundle, validate_bundle, write_manifest
from .drivable_surface import extract_drivable_surface


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _command(command: Sequence[str]) -> None:
    print("[PROCESS] " + " ".join(command), flush=True)
    subprocess.run(list(command), check=True)


def _bag_fingerprint(bag: Path) -> dict:
    metadata = bag / "metadata.yaml"
    if not metadata.is_file():
        raise ValueError(f"sensor bag metadata is missing: {metadata}")
    storage = sorted(
        item for item in bag.iterdir()
        if item.is_file() and (".db3" in item.name or item.suffix == ".mcap")
    )
    return {
        "metadata_sha256": _sha256(metadata),
        "storage_files": [
            {"name": item.name, "size_bytes": item.stat().st_size}
            for item in storage
        ],
    }


def _bag_storage_names(bag: Path) -> set[str]:
    return {
        item.name for item in bag.iterdir()
        if item.is_file() and (".db3" in item.name or item.suffix == ".mcap")
    }


def _remove_temporary_decompressed_bags(bag: Path, original: set[str]) -> None:
    """Remove only DB files rosbag2 created beside an existing zstd archive."""
    for item in bag.iterdir():
        if (
            item.is_file()
            and item.suffix == ".db3"
            and item.name not in original
            and (bag / f"{item.name}.zstd").is_file()
        ):
            item.unlink()


def _validate_2d(bundle: Path) -> dict:
    yaml_path = bundle / "occupancy.yaml"
    if not yaml_path.is_file():
        return {"status": "UNAVAILABLE", "reason": "occupancy.yaml is missing"}
    document = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    image = bundle / str(document.get("image", ""))
    if not image.is_file() or image.stat().st_size == 0:
        return {"status": "INVALID", "reason": f"occupancy image is missing: {image}"}
    return {
        "status": "READY",
        "yaml": str(yaml_path),
        "image": str(image),
        "resolution_m": float(document.get("resolution", 0.0)),
    }


def _regression_classification(path: Path) -> tuple[str, str]:
    document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    result = str(document.get("result", document.get("status", ""))).upper()
    if result in {"PASS", "READY"}:
        return "READY", ""
    reason = str(
        document.get("reason", document.get("failure_reason", "3D regression did not pass"))
    )
    return "DEGRADED", reason


def _archive_existing(target: Path) -> None:
    if not target.exists():
        return
    archive = target.parent / "archive"
    archive.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    destination = archive / f"{stamp}_{target.name}"
    suffix = 1
    while destination.exists():
        destination = archive / f"{stamp}_{target.name}_{suffix}"
        suffix += 1
    target.rename(destination)


def _attach_acquisition(source: Path, target: Path) -> None:
    """Reference the immutable source bag and copy its small audit evidence."""
    sensor_link = target / "sensors"
    if sensor_link.exists() or sensor_link.is_symlink():
        if sensor_link.is_dir() and not sensor_link.is_symlink():
            shutil.rmtree(sensor_link)
        else:
            sensor_link.unlink()
    sensor_link.symlink_to(source / "sensors", target_is_directory=True)
    for name in ("sensor_contract.yaml", "session_contract.yaml", "session_metrics.yaml"):
        source_file = source / name
        if source_file.is_file():
            shutil.copy2(source_file, target / name)


def _commit_processed(source: Path, staging: Path, target: Path) -> Path:
    _attach_acquisition(source, staging)
    metrics = validate_bundle(staging, require_sensor_bag=True)
    # The raw bag remains once in datasets/mapping.  Processed maps retain its
    # audit and session metrics but do not duplicate or permanently depend on
    # the large bag directory.
    (staging / "sensors").unlink()
    write_manifest(staging, metrics)
    promote_bundle(staging, target, overwrite_existing=True)
    validate_bundle(target, require_sensor_bag=False)
    return target


def _mark_processed_lineage(directory: Path, source: Path, dimension: str) -> None:
    metadata_path = directory / "metadata.yaml"
    document = (
        yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
        if metadata_path.is_file() else {}
    )
    document["environment"] = f"processed_{dimension}"
    document["source_acquisition"] = str(source)
    products = document.setdefault("products", {})
    products.pop("sensor_recording", None)
    products["source_sensor_contract"] = "sensor_contract.yaml"
    metadata_path.write_text(
        yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def _process_2d(
    bundle: Path, playback_rate: float, output_root: Path | None
) -> dict:
    if output_root is None:
        # Legacy sessions already contain a live slam_toolbox product.
        return _validate_2d(bundle)
    output_root = output_root.expanduser().resolve()
    staging = output_root / ".staging" / bundle.name
    target = output_root / bundle.name
    if staging.exists():
        shutil.rmtree(staging)
    staging.mkdir(parents=True, exist_ok=False)
    original_bag_files = _bag_storage_names(bundle / "sensors")
    try:
        _command([
            "ros2", "launch", "roka_nav2_bringup",
            "offline_2d_processing.launch.py",
            f"source_directory:={bundle}",
            f"output_directory:={staging}",
            f"playback_rate:={playback_rate}",
        ])
        _remove_temporary_decompressed_bags(
            bundle / "sensors", original_bag_files
        )
        required = (
            "occupancy.yaml", "occupancy.pgm", "slam_posegraph.posegraph",
            "track_profile.yaml", "terrain_map.bag/metadata.yaml",
        )
        missing = [
            str(staging / name) for name in required
            if not (staging / name).is_file()
        ]
        if missing:
            raise RuntimeError("2D processor did not produce: " + ", ".join(missing))
        _mark_processed_lineage(staging, bundle, "2d")
        _commit_processed(bundle, staging, target)
        result = _validate_2d(target)
        result["directory"] = str(target)
        result["source_acquisition"] = str(bundle)
        return result
    except Exception:
        _remove_temporary_decompressed_bags(
            bundle / "sensors", original_bag_files
        )
        failed = output_root / ".failed" / bundle.name
        _archive_existing(failed)
        failed.parent.mkdir(parents=True, exist_ok=True)
        if staging.exists():
            staging.rename(failed)
        raise


def _process_global_terrain(directory: Path) -> Path:
    """Build full-map terrain layers from the optimized PCD.

    The live terrain_map.bag is a moving local window.  This offline product
    covers the complete optimized point cloud and remains a display/review
    product until the surrounding 3D regression and map approval pass.
    """
    # The complete SLAM cloud contains walls, furniture and ceilings. Feeding
    # it directly into a 2.5D grid turns vertical objects into false ramps.
    # Keep map.pcd for localization and review; use only the route-connected
    # surface for elevation/slope/roughness processing.
    extraction = extract_drivable_surface(directory)
    source = directory / str(extraction["surface_pcd"])
    output_bag = directory / "terrain_global.bag"
    for path in (output_bag,):
        if path.exists():
            shutil.rmtree(path)
    terrain_share = Path(get_package_share_directory("roka_terrain_mapping"))
    _command([
        "ros2", "run", "roka_terrain_mapping",
        "terrain_standard_postprocessor_node", "--ros-args",
        "--params-file", str(terrain_share / "config" / "terrain_filter_chain.yaml"),
        "-p", f"input_pcd:={source}",
        "-p", f"pcl_config:={terrain_share / 'config' / 'global_pcd.yaml'}",
        "-p", f"output_bag:={output_bag}",
        "-p", "output_topic:=/terrain/static_map",
        "-p", "map_frame:=map",
    ])
    if not (output_bag / "metadata.yaml").is_file():
        raise RuntimeError("global terrain postprocessor did not create terrain_global.bag")
    (directory / "terrain_global_metadata.yaml").write_text(
        yaml.safe_dump({
            "schema_version": 1,
            "source_pcd": source.name,
            "frame_id": "map",
            "resolution_m": 0.10,
            "topic": "/terrain/static_map",
            "layers": [
                "elevation", "elevation_inpainted", "elevation_smooth",
                "normal_vectors_x", "normal_vectors_y", "normal_vectors_z",
                "slope", "slope_deg", "roughness", "step_height", "risk",
                "traversability",
            ],
            "pipeline": "ANYbotics grid_map_pcl + grid_map_filters",
            "role": "path_connected_drivable_surface_review",
            "surface_extraction": "surface_extraction.yaml",
            "autonomy_approval_inherited": False,
        }, sort_keys=False),
        encoding="utf-8",
    )
    return output_bag


def _write_3d_metadata(directory: Path, source: Path) -> None:
    document = {
        "schema_version": 2,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "environment": "processed_3d",
        "mapping_backend": "fastlio_sc_pgo",
        "pose_authority": "sc_pgo",
        "frames": {
            "global": "map", "odometry": "odom",
            "body_3d": "base_link", "body_2d": "base_footprint",
        },
        "sensors": {"lidar": "Velodyne VLP-16", "imu": "EBIMU-9DOFV5"},
        "products": {
            "occupancy": "occupancy.yaml",
            "terrain": "terrain_map.bag",
            "global_terrain": "terrain_global.bag",
            "track_profile": "track_profile.yaml",
            "pointcloud": "map.pcd",
            "drivable_surface": "surface.pcd",
            "optimized_path": "optimized_path.csv",
            "pose_graph_3d": "pose_graph.g2o",
        },
    }
    (directory / "metadata.yaml").write_text(
        yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def _process_3d(
    bundle: Path, playback_rate: float, output_root: Path | None = None
) -> dict:
    bag = bundle / "sensors"
    if output_root is None:
        derived = bundle / "derived"
        staging = derived / ".staging_fastlio_sc_pgo"
        target = derived / "fastlio_sc_pgo"
    else:
        derived = output_root.expanduser().resolve()
        staging = derived / ".staging" / bundle.name
        target = derived / bundle.name
    if staging.exists():
        shutil.rmtree(staging)
    staging.mkdir(parents=True, exist_ok=False)
    profile = bundle / "track_profile.yaml"
    if profile.is_file():
        shutil.copy2(profile, staging / "track_profile.yaml")
    original_bag_files = _bag_storage_names(bag)
    try:
        _command([
            "ros2", "launch", "roka_fast_lio_localization",
            "fastlio_bag_regression.launch.py",
            f"bag:={bag}", f"output_dir:={staging}",
            f"playback_rate:={playback_rate}",
        ])
        _remove_temporary_decompressed_bags(bag, original_bag_files)
        required = [staging / "map.pcd", staging / "regression.yaml"]
        missing = [
            str(path)
            for path in required
            if not path.is_file() or path.stat().st_size == 0
        ]
        if missing:
            raise RuntimeError("3D processor did not produce: " + ", ".join(missing))
        # Generate occupancy/path products when the SC-PGO archive is complete.
        if (staging / "pose_graph.g2o").is_file():
            _command([
                "ros2", "run", "roka_nav2_bringup",
                "derive_sc_pgo_navigation", str(staging),
            ])
        global_terrain = _process_global_terrain(staging)
        _write_3d_metadata(staging, bundle)
        _mark_processed_lineage(staging, bundle, "3d")
        status, reason = _regression_classification(staging / "regression.yaml")
        if output_root is None:
            _archive_existing(target)
            staging.rename(target)
        else:
            # The full-map GridMap is the saved terrain product for this
            # offline map.  Keep its descriptive name for RViz and expose the
            # conventional terrain_map.bag path used by autonomy validation.
            terrain_link = staging / "terrain_map.bag"
            terrain_link.symlink_to(global_terrain.name, target_is_directory=True)
            _commit_processed(bundle, staging, target)
        result = {
            "status": status,
            "directory": str(target),
            "regression": str(target / "regression.yaml"),
            "playback_rate": playback_rate,
            "autonomy_eligible": status == "READY",
            "global_terrain": str(target / global_terrain.name),
        }
        if reason:
            result["reason"] = reason
        return result
    except Exception:
        _remove_temporary_decompressed_bags(bag, original_bag_files)
        # Keep failed products for diagnosis; never touch the approved bundle.
        failed = (
            derived / "failed_fastlio_sc_pgo"
            if output_root is None
            else derived / ".failed" / bundle.name
        )
        _archive_existing(failed)
        failed.parent.mkdir(parents=True, exist_ok=True)
        if staging.exists():
            staging.rename(failed)
        raise


def process_session(
    bundle: Path,
    mode: str,
    playback_rate: float,
    *,
    output_2d: Path | None = None,
    output_3d: Path | None = None,
) -> dict:
    bundle = bundle.expanduser().resolve()
    if not bundle.is_dir():
        raise ValueError(f"map bundle does not exist: {bundle}")
    bag = bundle / "sensors"
    fingerprint = _bag_fingerprint(bag)
    audit_path = bundle / "derived" / "sensor_contract.yaml"
    audit_path.parent.mkdir(parents=True, exist_ok=True)
    _command([
        "ros2", "run", "roka_fast_lio_localization", "roka_bag_sensor_audit",
        str(bag), "--require-diagnostics", "--summary-only",
        "--output", str(audit_path),
    ])
    result = {
        "schema_version": 1,
        "processed_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_bundle": str(bundle),
        "source_bag": fingerprint,
        "mode": mode,
        "sensor_audit": str(audit_path),
        "two_dimensional": {"status": "NOT_REQUESTED"},
        "three_dimensional": {"status": "NOT_REQUESTED"},
    }
    if mode in {"2d", "all"}:
        result["two_dimensional"] = _process_2d(
            bundle, playback_rate, output_2d
        )
    elif mode == "audit":
        result["two_dimensional"] = _validate_2d(bundle)
    if mode in {"3d", "all"}:
        result["three_dimensional"] = _process_3d(
            bundle, playback_rate, output_3d
        )
    manifest = bundle / "derived" / "processing_manifest.yaml"
    manifest.write_text(yaml.safe_dump(result, sort_keys=False), encoding="utf-8")
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Audit and deterministically reprocess one captured ROKA session."
    )
    parser.add_argument("bundle")
    parser.add_argument("--mode", choices=("audit", "2d", "3d", "all"), default="all")
    parser.add_argument("--playback-rate", type=float, default=1.0)
    parser.add_argument("--output-2d", type=Path)
    parser.add_argument("--output-3d", type=Path)
    args = parser.parse_args(argv)
    if not 0.0 < args.playback_rate <= 4.0:
        parser.error("--playback-rate must be in (0, 4]")
    try:
        result = process_session(
            Path(args.bundle), args.mode, args.playback_rate,
            output_2d=args.output_2d, output_3d=args.output_3d,
        )
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"[PROCESS][FAIL] {error}", file=sys.stderr)
        return 1
    print(yaml.safe_dump(result, sort_keys=False).rstrip())
    two_d = str(result["two_dimensional"].get("status", "UNKNOWN"))
    three_d = str(result["three_dimensional"].get("status", "UNKNOWN"))
    eligible = bool(result["three_dimensional"].get("autonomy_eligible", False))
    print(f"[PROCESS][2D] {two_d}")
    print(f"[PROCESS][3D] {three_d}; autonomy_eligible={str(eligible).lower()}")
    print("[PROCESS][RESULT] COMPLETE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
