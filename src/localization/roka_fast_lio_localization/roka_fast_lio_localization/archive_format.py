"""Versioned SC-PGO archive checks shared by mapping and offline regression."""

from __future__ import annotations

from pathlib import Path
import struct


ARCHIVES = {
    "keyframe_poses.bin": {
        "current": b"ROKA_PGO1\0\0\0",
        "legacy": b"EXTREME_PGO1",
    },
    "scan_context.bin": {
        "current": b"ROKA_SC01\0\0\0",
        "legacy": b"EXTREME_SC01",
    },
}


def read_archive_header(path: Path) -> dict[str, int | bool | str]:
    """Read one fixed-width archive header and identify legacy ROKA maps."""
    path = Path(path)
    expected = ARCHIVES.get(path.name)
    if expected is None:
        raise ValueError(f"unsupported SC-PGO archive: {path.name}")
    with path.open("rb") as stream:
        payload = stream.read(24)
    if len(payload) != 24:
        raise ValueError(f"invalid SC-PGO archive header: {path.name}")
    magic = payload[:12]
    if magic == expected["current"]:
        legacy = False
    elif magic == expected["legacy"]:
        legacy = True
    else:
        raise ValueError(f"invalid SC-PGO archive magic: {path.name}")
    version, count = struct.unpack_from("<IQ", payload, 12)
    if version != 1 or count < 2:
        raise ValueError(
            f"invalid SC-PGO archive header: {path.name}, "
            f"version={version}, count={count}"
        )
    return {
        "bytes": path.stat().st_size,
        "count": int(count),
        "version": int(version),
        "legacy_magic": legacy,
    }


def validate_archive_pair(directory: Path) -> dict[str, dict[str, int | bool | str]]:
    """Validate both archives and require one keyframe descriptor per pose."""
    directory = Path(directory)
    result = {
        name: read_archive_header(directory / name)
        for name in ARCHIVES
    }
    if result["keyframe_poses.bin"]["count"] != result["scan_context.bin"]["count"]:
        raise ValueError("SC-PGO pose and scan-context archive counts differ")
    return result
