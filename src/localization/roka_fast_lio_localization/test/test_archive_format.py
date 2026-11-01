import struct

import pytest

from roka_fast_lio_localization.archive_format import (
    read_archive_header,
    validate_archive_pair,
)


def write_archive(path, magic, count=3):
    path.write_bytes(magic + struct.pack("<IQ", 1, count) + b"payload")


@pytest.mark.parametrize(
    ("name", "magic"),
    [
        ("keyframe_poses.bin", b"ROKA_PGO1\0\0\0"),
        ("scan_context.bin", b"ROKA_SC01\0\0\0"),
    ],
)
def test_current_archive_magic(name, magic, tmp_path):
    path = tmp_path / name
    write_archive(path, magic)
    assert read_archive_header(path)["legacy_magic"] is False


@pytest.mark.parametrize(
    ("name", "magic"),
    [
        ("keyframe_poses.bin", b"EXTREME_PGO1"),
        ("scan_context.bin", b"EXTREME_SC01"),
    ],
)
def test_legacy_archive_is_read_only_compatible(name, magic, tmp_path):
    path = tmp_path / name
    write_archive(path, magic)
    assert read_archive_header(path)["legacy_magic"] is True


def test_archive_pair_requires_matching_counts(tmp_path):
    write_archive(tmp_path / "keyframe_poses.bin", b"ROKA_PGO1\0\0\0", 3)
    write_archive(tmp_path / "scan_context.bin", b"ROKA_SC01\0\0\0", 2)
    with pytest.raises(ValueError, match="counts differ"):
        validate_archive_pair(tmp_path)
