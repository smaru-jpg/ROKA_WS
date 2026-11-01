"""Load and validate the single ROKA sensor/TF data contract."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


REQUIRED_TOPICS = {
    "lidar_packets",
    "imu",
    "wheel_odometry",
    "fused_odometry",
    "lidar_inertial_odometry",
    "dynamic_tf",
    "static_tf",
    "recorded_path",
    "motor_connected",
}

REQUIRED_ODOMETRY_AUTHORITIES = {
    "acquisition",
    "flat_processing",
    "track_processing",
    "flat_runtime",
    "track_runtime",
}


def default_contract_path() -> Path:
    """Return the installed contract, falling back to the source checkout."""
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory("roka_nav2_bringup"))
        candidate = installed / "config" / "sensor_data_contract.yaml"
        if candidate.is_file():
            return candidate
    except (ImportError, LookupError):
        pass
    return Path(__file__).resolve().parents[1] / "config" / "sensor_data_contract.yaml"


def load_data_contract(path: str | Path | None = None) -> dict[str, Any]:
    contract_path = Path(path).expanduser().resolve() if path else default_contract_path()
    document = yaml.safe_load(contract_path.read_text(encoding="utf-8")) or {}
    if int(document.get("schema_version", 0)) != 1:
        raise ValueError(f"unsupported sensor data contract: {contract_path}")
    topics = document.get("topics") or {}
    missing_topics = sorted(REQUIRED_TOPICS - set(topics))
    if missing_topics:
        raise ValueError("sensor contract is missing topics: " + ", ".join(missing_topics))
    for name, topic in topics.items():
        if not isinstance(topic, str) or not topic.startswith("/"):
            raise ValueError(f"invalid topic {name}: {topic!r}")
    authorities = document.get("odometry_authority") or {}
    missing_authorities = sorted(REQUIRED_ODOMETRY_AUTHORITIES - set(authorities))
    if missing_authorities:
        raise ValueError(
            "sensor contract is missing odometry authorities: "
            + ", ".join(missing_authorities)
        )
    for role, topic in authorities.items():
        if topic not in topics.values():
            raise ValueError(f"odometry authority {role} uses undeclared topic {topic}")
    return document


def acquisition_topics(contract: dict[str, Any]) -> list[str]:
    """Return compact, deterministic raw/context topics for one field run."""
    topics = contract["topics"]
    ordered = [
        topics["lidar_packets"],
        topics["imu"],
        topics["wheel_odometry"],
        topics["fused_odometry"],
        topics["lidar_inertial_odometry"],
        topics["dynamic_tf"],
        topics["static_tf"],
        topics.get("joy", "/joy"),
        topics.get("manual_command", "/cmd_vel_joy"),
        topics["recorded_path"],
        "/terrain/state",
        "/terrain/wheel_contact",
        topics["motor_connected"],
        "/md/left/raw_state",
        "/md/right/raw_state",
        "/diagnostics",
    ]
    return list(dict.fromkeys(ordered))

