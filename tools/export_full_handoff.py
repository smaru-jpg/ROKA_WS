#!/usr/bin/env python3
"""Export the complete portable ROKA workspace, including acquired evidence."""

import argparse
from datetime import datetime, timezone
from hashlib import sha256
import os
from pathlib import Path
import shutil
import stat
import tempfile
import xml.etree.ElementTree as ET
import zipfile

import yaml


WORKSPACE = Path(__file__).resolve().parents[1]
TOP_LEVEL_FILES = {".gitignore", "DEPENDENCIES_HUMBLE.lock", "roka"}
TOP_LEVEL_DIRECTORIES = {
    "src", "maps", "models", "datasets", "reports", "third_party",
    "docs", "env", "tools", "benchmarks",
}
GENERATED_EXCLUSIONS = {
    "build", "install", "log", ".roka_python", "__pycache__",
    ".pytest_cache", ".git",
}


def excluded(relative):
    return any(
        part in GENERATED_EXCLUSIONS or part.endswith(".egg-info")
        for part in relative.parts
    ) or relative.suffix in {".pyc", ".pyo"}


def digest(path):
    hasher = sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def copy_workspace(destination):
    target = destination / "roka_ws"
    target.mkdir(parents=True)
    for source in WORKSPACE.rglob("*"):
        relative = source.relative_to(WORKSPACE)
        if not relative.parts or relative.parts[0] not in TOP_LEVEL_DIRECTORIES:
            continue
        if excluded(relative) or source.is_dir():
            continue
        output = target / relative
        output.parent.mkdir(parents=True, exist_ok=True)
        if source.is_symlink():
            output.symlink_to(os.readlink(source))
        else:
            shutil.copy2(source, output)
    for name in TOP_LEVEL_FILES:
        source = WORKSPACE / name
        if source.is_file():
            shutil.copy2(source, target / name)
    return target


def package_names(target):
    names = []
    for manifest in sorted((target / "src").rglob("package.xml")):
        name = (ET.parse(manifest).getroot().findtext("name") or "").strip()
        if name:
            names.append(name)
    return sorted(names)


def validate_models(target):
    mission = target / "src" / "mission_vision"
    metadata = yaml.safe_load(
        (mission / "models" / "model_metadata.yaml").read_text(encoding="utf-8")
    )
    contracts = list(metadata.get("models", {}).values()) + list(
        metadata.get("rollback_models", {}).values()
    )
    for contract in contracts:
        model = mission / "models" / contract["file"]
        if not model.is_file() or digest(model) != contract["sha256"]:
            raise RuntimeError(f"mission model contract mismatch: {model}")
    arm = target / "src" / "manipulation" / "roka_arm"
    arm_metadata = yaml.safe_load(
        (arm / "models" / "model_metadata.yaml").read_text(encoding="utf-8")
    )
    arm_model = arm / "models" / arm_metadata["model_file"]
    if not arm_model.is_file() or digest(arm_model) != arm_metadata["sha256"]:
        raise RuntimeError(f"arm model contract mismatch: {arm_model}")


def write_metadata(target):
    packages = package_names(target)
    file_count = sum(1 for path in target.rglob("*") if path.is_file())
    byte_count = sum(
        path.stat().st_size for path in target.rglob("*")
        if path.is_file() and not path.is_symlink()
    )
    (target / "HANDOFF_INFO.txt").write_text(
        "\n".join([
            "ROKA full portable handoff format: 3",
            f"created_utc: {datetime.now(timezone.utc).isoformat(timespec='seconds')}",
            "target: Ubuntu 22.04 / ROS 2 Humble / Python 3.10",
            "scope: source + maps + datasets + reports + models + simulation/reference",
            "excluded: build install log .roka_python and language/tool caches",
            f"portable_file_count: {file_count}",
            f"portable_uncompressed_bytes: {byte_count}",
            f"package_count: {len(packages)}",
            "packages: " + " ".join(packages),
            "system dependencies: ./tools/install_system_dependencies_low_memory.sh",
            "staged build/install: ./tools/install_handoff.sh",
            "verify: roka verify",
            "YOLO locations: roka yolo-paths",
            "",
        ]),
        encoding="utf-8",
    )
    manifest = target / "HANDOFF_MANIFEST.sha256"
    lines = []
    for path in sorted(target.rglob("*")):
        if path.is_file() and not path.is_symlink() and path != manifest:
            lines.append(f"{digest(path)}  {path.relative_to(target)}")
    manifest.write_text("\n".join(lines) + "\n", encoding="utf-8")


def validate_tree(target):
    required = (
        "roka", "DEPENDENCIES_HUMBLE.lock", "HANDOFF_INFO.txt",
        "HANDOFF_MANIFEST.sha256", "tools/install_handoff.sh",
        "tools/install_system_dependencies_low_memory.sh",
        "docs/ROKA_통합_사용명령.txt",
        "docs/ROKA_YOLO_교체_가이드.md",
        "src/mission_vision/firmware/roka_iff_neopixel/roka_iff_neopixel.ino",
        "src/mission_vision/firmware/roka_iff_neopixel/README.md",
        "src/mission/roka_mission/roka_mission/traffic_signal_route_gate.py",
    )
    missing = [name for name in required if not (target / name).is_file()]
    if missing:
        raise RuntimeError(f"full handoff is missing required files: {missing}")
    for forbidden in GENERATED_EXCLUSIONS:
        if any(path.name == forbidden for path in target.rglob("*")):
            raise RuntimeError(f"generated cache leaked into handoff: {forbidden}")
    broken = [path for path in target.rglob("*") if path.is_symlink() and not path.exists()]
    if broken:
        raise RuntimeError(f"broken symlink in handoff: {broken[:3]}")
    validate_models(target)
    for line in (target / "HANDOFF_MANIFEST.sha256").read_text(
        encoding="utf-8"
    ).splitlines():
        checksum, relative = line.split("  ", 1)
        path = target / relative
        if not path.is_file() or digest(path) != checksum:
            raise RuntimeError(f"manifest mismatch: {relative}")


def archive_tree(target, output):
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    if temporary.exists():
        temporary.unlink()
    already_compressed = {".zstd", ".zip", ".pt", ".png", ".jpg", ".jpeg", ".mp4"}
    with zipfile.ZipFile(temporary, "w", allowZip64=True) as archive:
        for path in sorted(target.rglob("*")):
            relative = path.relative_to(target.parent)
            if path.is_symlink():
                info = zipfile.ZipInfo(str(relative))
                info.create_system = 3
                info.external_attr = (stat.S_IFLNK | 0o777) << 16
                archive.writestr(info, os.readlink(path))
            elif path.is_file():
                compression = (
                    zipfile.ZIP_STORED
                    if path.suffix.lower() in already_compressed
                    else zipfile.ZIP_DEFLATED
                )
                archive.write(path, relative, compress_type=compression, compresslevel=1)
    temporary.replace(output)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    with tempfile.TemporaryDirectory(prefix="roka_full_handoff_") as directory:
        target = copy_workspace(Path(directory))
        validate_models(target)
        write_metadata(target)
        validate_tree(target)
        archive_tree(target, output)
    print(
        f"[ROKA-FULL-HANDOFF][PASS] {output} "
        f"({output.stat().st_size / 1048576:.1f} MiB)"
    )


if __name__ == "__main__":
    main()
