#!/usr/bin/env python3
"""Create a clean real-robot handoff zip without local build or simulator data."""

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


WORKSPACE = Path(__file__).resolve().parents[1]
TOP_LEVEL_FILES = {".gitignore", "DEPENDENCIES_HUMBLE.lock", "roka"}
TOP_LEVEL_DIRECTORIES = {"src", "maps", "docs", "env", "third_party", "tools"}
USER_DOCUMENTS = {
    "ROKA_통합_사용명령.txt",
}
SKIP_PARTS = {
    "build", "install", "log", "reports", "datasets", "benchmarks",
    ".roka_python", "__pycache__", ".pytest_cache", ".staging", ".backup", ".git",
}


def excluded(relative):
    parts = relative.parts
    if any(part in SKIP_PARTS or part.endswith(".egg-info") for part in parts):
        return True
    if len(parts) >= 2 and parts[:2] == ("src", "simulation"):
        return True
    if len(parts) >= 3 and parts[:3] == ("src", "vendor", "lidar_slam_ros2"):
        return True
    if len(parts) >= 2 and parts[:2] == ("maps", "sim_pk3a_ground_truth"):
        return True
    if parts and parts[0] == "docs" and relative.name not in USER_DOCUMENTS:
        return True
    return relative.suffix in {".pyc", ".pyo"}


def copy_handoff(destination):
    target = destination / "roka_ws"
    target.mkdir(parents=True)
    for source in WORKSPACE.rglob("*"):
        relative = source.relative_to(WORKSPACE)
        if not relative.parts:
            continue
        if relative.parts[0] not in TOP_LEVEL_DIRECTORIES:
            continue
        if excluded(relative) or source.is_dir():
            continue
        output = target / relative
        output.parent.mkdir(parents=True, exist_ok=True)
        if source.is_symlink():
            # Preserve relative library links.  Broken documentation links from
            # extracted deb packages are harmless and remain broken instead of
            # aborting the whole handoff export.
            output.symlink_to(os.readlink(source))
        else:
            shutil.copy2(source, output)
    for name in TOP_LEVEL_FILES:
        source = WORKSPACE / name
        if source.is_file():
            shutil.copy2(source, target / name)

    write_handoff_metadata(target)
    return target


def digest(path):
    hasher = sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def package_names(target):
    result = []
    for manifest in sorted((target / "src").rglob("package.xml")):
        name = (ET.parse(manifest).getroot().findtext("name") or "").strip()
        if name:
            result.append(name)
    return sorted(result)


def write_handoff_metadata(target):
    packages = package_names(target)
    information = target / "HANDOFF_INFO.txt"
    information.write_text(
        "\n".join([
            "ROKA handoff format: 2",
            f"created_utc: {datetime.now(timezone.utc).isoformat(timespec='seconds')}",
            "target: Ubuntu 22.04 / ROS 2 Humble",
            "generated directories: build/install/log/.roka_python are intentionally absent",
            f"package_count: {len(packages)}",
            "packages: " + " ".join(packages),
            "install: ./tools/install_handoff.sh --command-name roka --domain-id 31",
            "verify: roka verify",
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


def validate_handoff(target):
    documents = {path.name for path in (target / "docs").glob("*") if path.is_file()}
    if documents != USER_DOCUMENTS:
        raise RuntimeError(f"handoff documents differ from contract: {documents}")
    forbidden = [
        path for path in target.rglob("*")
        if "mujoco" in path.name.lower() or (
            path.is_dir() and path.name == "simulation"
        )
    ]
    if forbidden:
        raise RuntimeError(f"simulator content leaked into handoff: {forbidden[:3]}")
    if any((target / name).exists() for name in ("build", "install", "log")):
        raise RuntimeError("generated build directories leaked into handoff")
    broken_links = [
        path for path in target.rglob("*")
        if path.is_symlink() and not path.exists()
    ]
    if broken_links:
        raise RuntimeError(f"broken symlinks leaked into handoff: {broken_links[:3]}")
    if not (target / "src" / "mission" / "roka_mission" / "package.xml").is_file():
        raise RuntimeError("mission package is missing from handoff")
    required = (
        "HANDOFF_INFO.txt",
        "HANDOFF_MANIFEST.sha256",
        "tools/install_handoff.sh",
        "tools/check_overlay.py",
        "src/manipulation/roka_arm/models/box_marker_best.pt",
        "src/mission_vision/package.xml",
        "src/mission_vision/models/IFF_0830.pt",
        "src/mission_vision/models/box_0830.pt",
        "src/mission_vision/models/vision_0830.pt",
    )
    missing = [name for name in required if not (target / name).is_file()]
    if missing:
        raise RuntimeError(f"required handoff files are missing: {missing}")
    expected_model = "059e5c485f5162659ae76b6492d7a8291bbcbce6ab75d1dbbf555f6e9f7acc95"
    model = target / "src/manipulation/roka_arm/models/box_marker_best.pt"
    if digest(model) != expected_model:
        raise RuntimeError("BOX model checksum differs from the deployment contract")
    mission_model_contract = {
        "IFF_0830.pt": "4a26f5212771f5d9b1587ecffbfc8dc1aca9584232b096c03b10626460626f57",
        "box_0830.pt": "cc4c52f9151bb1b765c38f7a7e225cb0f4267d67252a640ae72931a17bb8787b",
        "vision_0830.pt": "7bdfb6c208c4da0146f5089420c5dad4ddb40d14e205c05ff16806425bf30873",
    }
    for filename, expected in mission_model_contract.items():
        mission_model = target / "src" / "mission_vision" / "models" / filename
        if digest(mission_model) != expected:
            raise RuntimeError(f"mission model checksum differs: {filename}")

    listed = {}
    for line in (target / "HANDOFF_MANIFEST.sha256").read_text(encoding="utf-8").splitlines():
        checksum, relative = line.split("  ", 1)
        listed[relative] = checksum
    for relative, checksum in listed.items():
        path = target / relative
        if not path.is_file() or digest(path) != checksum:
            raise RuntimeError(f"handoff manifest mismatch: {relative}")


def archive_tree(target, output):
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    if temporary.exists():
        temporary.unlink()
    with zipfile.ZipFile(temporary, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(target.rglob("*")):
            relative = path.relative_to(target.parent)
            if path.is_symlink():
                info = zipfile.ZipInfo(str(relative))
                info.create_system = 3
                info.external_attr = (stat.S_IFLNK | 0o777) << 16
                archive.writestr(info, os.readlink(path))
            elif path.is_file():
                archive.write(path, path.relative_to(target.parent))
    temporary.replace(output)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    output = args.output.expanduser().resolve()
    with tempfile.TemporaryDirectory(prefix="roka_handoff_") as directory:
        target = copy_handoff(Path(directory))
        validate_handoff(target)
        archive_tree(target, output)
    print(f"[ROKA-HANDOFF][PASS] {output} ({output.stat().st_size / 1048576:.1f} MiB)")


if __name__ == "__main__":
    main()
