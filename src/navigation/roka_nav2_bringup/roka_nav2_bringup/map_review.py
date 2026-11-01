"""Approve a reviewed PCD without ever overwriting the immutable source map."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from roka_nav2_bringup.map_bundle_quality import (
    file_sha256,
    occupancy_metrics,
    path_occupancy_metrics,
    pointcloud_count,
)


REVIEW_MANIFEST = "map_review.yaml"
OCCUPANCY_REVIEW_MANIFEST = "occupancy_review.yaml"


def _inside_bundle(bundle: Path, value: str | Path) -> Path:
    candidate = Path(value).expanduser()
    if not candidate.is_absolute():
        candidate = bundle / candidate
    candidate = candidate.resolve()
    try:
        candidate.relative_to(bundle)
    except ValueError as error:
        raise ValueError(f"reviewed product escapes the map bundle: {candidate}") from error
    return candidate


def approve_map(bundle: Path, *, use_cleaned: bool) -> Path:
    bundle = Path(bundle).expanduser().resolve()
    source = bundle / "map.pcd"
    if not source.is_file():
        raise ValueError(f"source map is missing: {source}")
    active = bundle / ("map_cleaned.pcd" if use_cleaned else "map.pcd")
    if not active.is_file():
        raise ValueError(f"reviewed map is missing: {active}")
    source_points = pointcloud_count(source)
    active_points = pointcloud_count(active)
    if source_points < 1000 or active_points < 1000:
        raise ValueError("source and reviewed maps must each contain at least 1000 points")
    if active_points < int(0.50 * source_points):
        raise ValueError("reviewed map removed more than 50 percent of source points")

    cleanup: dict[str, Any] = {}
    cleanup_path = bundle / "map_cleanup.yaml"
    if use_cleaned:
        if not cleanup_path.is_file():
            raise ValueError("map_cleanup.yaml is required for a cleaned-map approval")
        cleanup = yaml.safe_load(cleanup_path.read_text(encoding="utf-8")) or {}
        cleanup_source = _inside_bundle(bundle, str(cleanup.get("source_map", "")))
        cleanup_active = _inside_bundle(bundle, str(cleanup.get("cleaned_map", "")))
        if cleanup_source != source or cleanup_active != active:
            raise ValueError("cleanup audit does not describe this source/cleaned map pair")
        if cleanup.get("source_preserved") is not True:
            raise ValueError("cleanup audit does not prove source preservation")

    document = {
        "schema_version": 1,
        "status": "approved",
        "approved_utc": datetime.now(timezone.utc).isoformat(),
        "source_pcd": source.name,
        "active_pcd": active.name,
        "source_sha256": file_sha256(source),
        "active_sha256": file_sha256(active),
        "source_points": source_points,
        "active_points": active_points,
        "manual_cleanup_applied": use_cleaned,
        "cleanup_audit_sha256": file_sha256(cleanup_path) if use_cleaned else "",
    }
    output = bundle / REVIEW_MANIFEST
    temporary = bundle / f".{REVIEW_MANIFEST}.preparing"
    temporary.write_text(
        yaml.safe_dump(document, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    temporary.replace(output)
    return active


def active_reviewed_map(bundle: Path, *, require_approved: bool = True) -> Path:
    bundle = Path(bundle).expanduser().resolve()
    review_path = bundle / REVIEW_MANIFEST
    if not review_path.is_file():
        if require_approved:
            raise ValueError(f"3D map has not been reviewed: {review_path}")
        return bundle / "map.pcd"
    document = yaml.safe_load(review_path.read_text(encoding="utf-8")) or {}
    if document.get("status") != "approved" or document.get("schema_version") != 1:
        raise ValueError("map review manifest is not approved")
    source = _inside_bundle(bundle, str(document.get("source_pcd", "")))
    active = _inside_bundle(bundle, str(document.get("active_pcd", "")))
    if not source.is_file() or file_sha256(source) != str(document.get("source_sha256", "")):
        raise ValueError("source map changed after review")
    if not active.is_file() or file_sha256(active) != str(document.get("active_sha256", "")):
        raise ValueError("active reviewed map changed after approval")
    if pointcloud_count(active) != int(document.get("active_points", -1)):
        raise ValueError("active reviewed map point count changed after approval")
    if document.get("manual_cleanup_applied") is True:
        cleanup_path = bundle / "map_cleanup.yaml"
        if not cleanup_path.is_file() or file_sha256(cleanup_path) != str(
            document.get("cleanup_audit_sha256", "")
        ):
            raise ValueError("cleanup audit changed after approval")
    return active


def _occupancy_image(map_yaml: Path) -> Path:
    document = yaml.safe_load(map_yaml.read_text(encoding="utf-8")) or {}
    image_value = document.get("image")
    if not isinstance(image_value, str) or not image_value.strip():
        raise ValueError(f"occupancy YAML has no image: {map_yaml}")
    image = Path(image_value).expanduser()
    if not image.is_absolute():
        image = map_yaml.parent / image
    return image.resolve()


def approve_occupancy(bundle: Path, *, use_cleaned: bool) -> Path:
    """Approve an immutable source or non-destructively cleaned 2D map."""
    bundle = Path(bundle).expanduser().resolve()
    source = bundle / "occupancy.yaml"
    active = bundle / ("occupancy_cleaned.yaml" if use_cleaned else "occupancy.yaml")
    if not source.is_file() or not active.is_file():
        raise ValueError("source or reviewed occupancy YAML is missing")
    source_image = _occupancy_image(source)
    active_image = _occupancy_image(active)
    if not source_image.is_file() or not active_image.is_file():
        raise ValueError("source or reviewed occupancy image is missing")
    try:
        source_image_relative = source_image.relative_to(bundle).as_posix()
        active_image_relative = active_image.relative_to(bundle).as_posix()
    except ValueError as error:
        raise ValueError("occupancy images must remain inside the map bundle") from error

    source_metrics = occupancy_metrics(source)
    active_metrics = occupancy_metrics(active)
    for key in ("width", "height"):
        if source_metrics[key] != active_metrics[key]:
            raise ValueError("reviewed occupancy dimensions differ from the source")
    if active_metrics["known_count"] < 500 or active_metrics["occupied_count"] < 30:
        raise ValueError("reviewed occupancy map has insufficient observed structure")
    if active_metrics["known_largest_fraction"] < 0.70:
        raise ValueError("reviewed occupancy known space is fragmented")

    source_document = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    active_document = yaml.safe_load(active.read_text(encoding="utf-8")) or {}
    for key in ("resolution", "origin", "negate", "occupied_thresh", "free_thresh"):
        if source_document.get(key) != active_document.get(key):
            raise ValueError(f"reviewed occupancy changed map geometry/semantics: {key}")

    cleanup_path = bundle / "occupancy_cleanup.yaml"
    if use_cleaned:
        if not cleanup_path.is_file():
            raise ValueError("occupancy_cleanup.yaml is required for cleaned-map approval")
        cleanup = yaml.safe_load(cleanup_path.read_text(encoding="utf-8")) or {}
        cleanup_source = _inside_bundle(bundle, str(cleanup.get("source_yaml", "")))
        cleanup_active = _inside_bundle(bundle, str(cleanup.get("cleaned_yaml", "")))
        if cleanup_source != source or cleanup_active != active:
            raise ValueError("2D cleanup audit does not describe this map pair")
        if cleanup.get("source_preserved") is not True:
            raise ValueError("2D cleanup audit does not prove source preservation")
        changed_cells = int(cleanup.get("changed_cells", -1))
        total_cells = int(source_metrics["width"]) * int(source_metrics["height"])
        if changed_cells < 0 or changed_cells > int(0.25 * total_cells):
            raise ValueError("2D cleanup changed more than 25 percent of the map")

    profile = bundle / "track_profile.yaml"
    if profile.is_file():
        alignment = path_occupancy_metrics(active, profile)
        if alignment["centre_in_bounds_fraction"] < 0.95:
            raise ValueError("reviewed map no longer contains the recorded route")
        if alignment["centre_free_fraction"] < 0.90:
            raise ValueError("reviewed map does not keep the recorded route free")
        if alignment["wheel_lane_free_fraction"] < 0.80:
            raise ValueError("reviewed map does not keep enough wheel-lane clearance")

    document = {
        "schema_version": 1,
        "status": "approved",
        "approved_utc": datetime.now(timezone.utc).isoformat(),
        "source_yaml": source.name,
        "active_yaml": active.name,
        "source_yaml_sha256": file_sha256(source),
        "active_yaml_sha256": file_sha256(active),
        "source_image": source_image_relative,
        "active_image": active_image_relative,
        "source_image_sha256": file_sha256(source_image),
        "active_image_sha256": file_sha256(active_image),
        "manual_cleanup_applied": use_cleaned,
        "cleanup_audit_sha256": file_sha256(cleanup_path) if use_cleaned else "",
    }
    output = bundle / OCCUPANCY_REVIEW_MANIFEST
    temporary = bundle / f".{OCCUPANCY_REVIEW_MANIFEST}.preparing"
    temporary.write_text(
        yaml.safe_dump(document, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    temporary.replace(output)
    return active


def active_reviewed_occupancy(bundle: Path, *, require_approved: bool = True) -> Path:
    bundle = Path(bundle).expanduser().resolve()
    review_path = bundle / OCCUPANCY_REVIEW_MANIFEST
    if not review_path.is_file():
        if require_approved:
            raise ValueError(f"2D occupancy map has not been reviewed: {review_path}")
        return bundle / "occupancy.yaml"
    document = yaml.safe_load(review_path.read_text(encoding="utf-8")) or {}
    if document.get("status") != "approved" or document.get("schema_version") != 1:
        raise ValueError("occupancy review manifest is not approved")
    source = _inside_bundle(bundle, str(document.get("source_yaml", "")))
    active = _inside_bundle(bundle, str(document.get("active_yaml", "")))
    source_image = _inside_bundle(bundle, str(document.get("source_image", "")))
    active_image = _inside_bundle(bundle, str(document.get("active_image", "")))
    checks = (
        (source, "source_yaml_sha256", "source occupancy YAML"),
        (active, "active_yaml_sha256", "active occupancy YAML"),
        (source_image, "source_image_sha256", "source occupancy image"),
        (active_image, "active_image_sha256", "active occupancy image"),
    )
    for path, hash_key, label in checks:
        if not path.is_file() or file_sha256(path) != str(document.get(hash_key, "")):
            raise ValueError(f"{label} changed after approval")
    if _occupancy_image(source) != source_image or _occupancy_image(active) != active_image:
        raise ValueError("approved occupancy YAML now references another image")
    if document.get("manual_cleanup_applied") is True:
        cleanup_path = bundle / "occupancy_cleanup.yaml"
        if not cleanup_path.is_file() or file_sha256(cleanup_path) != str(
            document.get("cleanup_audit_sha256", "")
        ):
            raise ValueError("2D cleanup audit changed after approval")
    return active


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("bundle")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--approve-cleaned", action="store_true")
    group.add_argument("--approve-original", action="store_true")
    args = parser.parse_args(argv)
    active = approve_map(Path(args.bundle), use_cleaned=args.approve_cleaned)
    print(f"approved navigation PCD: {active}")


def occupancy_main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("bundle")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--approve-cleaned", action="store_true")
    group.add_argument("--approve-original", action="store_true")
    args = parser.parse_args(argv)
    active = approve_occupancy(Path(args.bundle), use_cleaned=args.approve_cleaned)
    print(f"approved navigation occupancy map: {active}")
