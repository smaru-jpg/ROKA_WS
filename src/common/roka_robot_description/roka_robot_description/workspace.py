"""Portable access to ROKA workspace-owned runtime data.

ROS package resources belong in the ament install prefix.  Maps, recorded
datasets and generated models are intentionally kept at the workspace root,
so callers need one reliable way to locate that root on every operator PC.
"""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix


def workspace_root() -> Path:
    """Return the ROKA workspace root without depending on a user name."""
    configured = os.environ.get("ROKA_WS_ROOT", "").strip()
    if configured:
        return Path(configured).expanduser().resolve()

    candidates: list[Path] = []
    try:
        prefix = Path(get_package_prefix("roka_robot_description")).resolve()
        candidates.extend((prefix, *prefix.parents))
    except LookupError:
        pass

    source = Path(__file__).resolve()
    candidates.extend(source.parents)
    for candidate in candidates:
        if (candidate / "src").is_dir() and (candidate / "maps").is_dir():
            return candidate

    raise RuntimeError(
        "ROKA workspace root was not found; run through roka or set "
        "ROKA_WS_ROOT to the extracted roka_ws directory"
    )


def workspace_path(*parts: str) -> Path:
    """Resolve a path stored below the ROKA workspace root."""
    return workspace_root().joinpath(*parts)
