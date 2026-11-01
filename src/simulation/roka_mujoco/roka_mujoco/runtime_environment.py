"""Locate the native MuJoCo ros2_control runtime used by capstone_ws."""

import os
from pathlib import Path
import sys

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
)


PACKAGE_NAME = "mujoco_ros2_control"
LOCAL_INSTALL = (
    Path.home() / ".local" / "ros-humble-mujoco" / "opt" / "ros" / "humble"
)


def _prepend_paths(variable: str, paths: list[Path]) -> None:
    current = [value for value in os.environ.get(variable, "").split(os.pathsep) if value]
    additions = [str(path) for path in paths if path.is_dir()]
    os.environ[variable] = os.pathsep.join(
        list(dict.fromkeys([*additions, *current]))
    )


def _append_paths(variable: str, paths: list[Path]) -> None:
    current = [value for value in os.environ.get(variable, "").split(os.pathsep) if value]
    additions = [str(path) for path in paths if path.is_dir()]
    os.environ[variable] = os.pathsep.join(
        list(dict.fromkeys([*current, *additions]))
    )


def configure_local_dependencies(prefix: Path = LOCAL_INSTALL) -> Path:
    """Expose local MuJoCo libraries without shadowing a workspace runtime."""
    resolved = Path(prefix).expanduser().resolve()
    _append_paths("AMENT_PREFIX_PATH", [resolved])
    _append_paths(
        "LD_LIBRARY_PATH",
        [resolved / "lib", resolved / "opt" / "mujoco_vendor" / "lib"],
    )
    return resolved


def configure_local_runtime(prefix: Path = LOCAL_INSTALL) -> Path:
    """Expose the user-local C++ runtime without copying or modifying it."""
    resolved = Path(prefix).expanduser().resolve()
    marker = (
        resolved
        / "share"
        / "ament_index"
        / "resource_index"
        / "packages"
        / PACKAGE_NAME
    )
    executable = resolved / "lib" / PACKAGE_NAME / "ros2_control_node"
    if not marker.is_file() or not executable.is_file():
        raise RuntimeError(
            f"{PACKAGE_NAME} is not installed in /opt/ros/humble or {resolved}."
        )

    python_site = (
        resolved
        / "local"
        / "lib"
        / f"python{sys.version_info.major}.{sys.version_info.minor}"
        / "dist-packages"
    )
    _prepend_paths("AMENT_PREFIX_PATH", [resolved])
    _prepend_paths(
        "LD_LIBRARY_PATH",
        [resolved / "lib", resolved / "opt" / "mujoco_vendor" / "lib"],
    )
    _prepend_paths("PATH", [resolved / "lib" / PACKAGE_NAME])
    _prepend_paths("PYTHONPATH", [python_site])
    return resolved


def ensure_mujoco_runtime() -> Path:
    """Return a usable native runtime prefix, with the capstone fallback."""
    try:
        package_prefix = Path(get_package_prefix(PACKAGE_NAME))
        if package_prefix.resolve() != LOCAL_INSTALL.resolve():
            configure_local_dependencies()
        return package_prefix
    except PackageNotFoundError:
        configured = configure_local_runtime()
    try:
        return Path(get_package_prefix(PACKAGE_NAME))
    except PackageNotFoundError as error:
        raise RuntimeError(
            f"Configured {configured}, but ROS cannot discover {PACKAGE_NAME}."
        ) from error
