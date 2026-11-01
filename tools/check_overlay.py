#!/usr/bin/env python3
"""Verify that this ROKA install wins over every other ROS 2 overlay.

The handoff may be used on a PC that already has packages with the same names.
Having duplicates is not inherently wrong; using a package from the wrong
prefix is.  This checker fails when any package shipped in this workspace is
resolved outside this workspace's install tree.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix


def workspace_packages(
    workspace: Path, *, include_simulation: bool = False
) -> dict[str, Path]:
    packages: dict[str, Path] = {}
    source_root = (workspace / "src").resolve()
    for manifest in sorted(source_root.rglob("package.xml")):
        relative_manifest = manifest.relative_to(source_root)
        if (
            not include_simulation
            and relative_manifest.parts
            and relative_manifest.parts[0] == "simulation"
        ):
            continue
        # Match colcon discovery: a marker on any containing source directory
        # excludes that legacy/reference subtree from the active workspace.
        if any(
            (parent / "COLCON_IGNORE").is_file()
            for parent in (manifest.parent, *manifest.parents)
            if is_below(parent, source_root)
        ):
            continue
        try:
            name = (ET.parse(manifest).getroot().findtext("name") or "").strip()
        except ET.ParseError as error:
            raise RuntimeError(f"invalid package manifest {manifest}: {error}") from error
        if not name:
            raise RuntimeError(f"package name is missing in {manifest}")
        previous = packages.get(name)
        if previous is not None:
            raise RuntimeError(
                f"duplicate package '{name}' inside ROKA source: {previous} and {manifest}"
            )
        packages[name] = manifest.parent
    return packages


def resource_prefixes(package: str) -> list[Path]:
    matches = []
    for raw_prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if not raw_prefix:
            continue
        prefix = Path(raw_prefix).expanduser().resolve()
        resource = prefix / "share/ament_index/resource_index/packages" / package
        if resource.is_file():
            matches.append(prefix)
    return matches


def is_below(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def validate_domain() -> tuple[bool, str]:
    raw = os.environ.get("ROS_DOMAIN_ID", "")
    try:
        value = int(raw)
    except ValueError:
        return False, f"ROS_DOMAIN_ID is not an integer: {raw!r}"
    if not 0 <= value <= 232:
        return False, f"ROS_DOMAIN_ID must be in 0..232, got {value}"
    return True, str(value)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path(__file__).resolve().parents[1],
    )
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument(
        "--include-simulation",
        action="store_true",
        help="also require optional packages below src/simulation",
    )
    args = parser.parse_args(argv)

    workspace = args.workspace.expanduser().resolve()
    install = (workspace / "install").resolve()
    failures: list[str] = []
    shadowed: list[str] = []

    configured_root = os.environ.get("ROKA_WS_ROOT", "").strip()
    if not configured_root:
        failures.append("ROKA_WS_ROOT is unset")
    elif Path(configured_root).expanduser().resolve() != workspace:
        failures.append(
            f"ROKA_WS_ROOT points to {configured_root}, expected {workspace}"
        )

    domain_ok, domain_detail = validate_domain()
    if not domain_ok:
        failures.append(domain_detail)

    try:
        packages = workspace_packages(
            workspace, include_simulation=args.include_simulation
        )
    except RuntimeError as error:
        failures.append(str(error))
        packages = {}

    for package in sorted(packages):
        try:
            active = Path(get_package_prefix(package)).resolve()
        except LookupError:
            failures.append(f"{package}: not present in the active ament index")
            continue
        if not is_below(active, install):
            failures.append(
                f"{package}: active prefix is {active}, outside {install}"
            )
            continue
        candidates = resource_prefixes(package)
        foreign = [prefix for prefix in candidates if not is_below(prefix, install)]
        if foreign:
            shadowed.append(
                f"{package}: using {active}; inactive duplicates="
                + ",".join(str(prefix) for prefix in foreign)
            )

    if failures:
        for detail in failures:
            print(f"[ROKA-OVERLAY][FAIL] {detail}", file=sys.stderr)
        print(
            "[ROKA-OVERLAY][RESULT] NOT READY; open a fresh terminal or run only "
            "through this workspace's launcher",
            file=sys.stderr,
        )
        return 1

    if not args.quiet:
        print(
            f"[ROKA-OVERLAY][PASS] workspace={workspace}; "
            f"packages={len(packages)}; ROS_DOMAIN_ID={domain_detail}"
        )
        for detail in shadowed:
            print(f"[ROKA-OVERLAY][SHADOWED-SAFELY] {detail}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
