"""Detect another real-hardware field launch before serial devices are opened."""

import os
from pathlib import Path


FIELD_LAUNCH_FILES = (
    "mapping_manual.launch.py",
    "pk3a_field.launch.py",
    "pk3a_autonomy.launch.py",
    "lidar_md_navigation.launch.py",
)


def ancestor_pids(proc_root: Path, pid: int) -> set[int]:
    """Return launch-wrapper ancestors so they are not mistaken for rivals."""
    result = set()
    current = int(pid)
    while current > 1:
        try:
            # After ``(comm)``: state is field 3 and parent PID is field 4.
            suffix = (Path(proc_root) / str(current) / "stat").read_text(
                encoding="utf-8"
            ).rsplit(") ", 1)[1]
            parent = int(suffix.split()[1])
        except (OSError, IndexError, ValueError):
            break
        if parent <= 1 or parent in result:
            break
        result.add(parent)
        current = parent
    return result


def other_field_launches(proc_root=Path("/proc"), current_pid=None):
    own_pid = os.getpid() if current_pid is None else int(current_pid)
    ignored = {own_pid, *ancestor_pids(Path(proc_root), own_pid)}
    matches = []
    for entry in Path(proc_root).glob("[0-9]*"):
        try:
            pid = int(entry.name)
        except ValueError:
            continue
        if pid in ignored:
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace"
            )
        except OSError:
            continue
        if "ros2 launch roka_nav2_bringup" not in command:
            continue
        if any(filename in command for filename in FIELD_LAUNCH_FILES):
            matches.append((pid, command.strip()))
    return sorted(matches)
