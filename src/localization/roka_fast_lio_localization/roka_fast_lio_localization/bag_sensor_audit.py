#!/usr/bin/env python3
"""Audit immutable sensor timing and TF contracts in an ROKA rosbag."""

import argparse
from collections import defaultdict
import hashlib
import math
from pathlib import Path
import shutil
import statistics
import tempfile

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml

from roka_robot_description.workspace import workspace_root


REQUIRED_TOPICS = {
    "/velodyne_packets": "velodyne_msgs/msg/VelodyneScan",
    "/imu/data": "sensor_msgs/msg/Imu",
    "/wheel/odom": "nav_msgs/msg/Odometry",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
}

OPTIONAL_DIAGNOSTIC_TOPICS = {
    "/md/base_connected": "std_msgs/msg/Bool",
    "/md/left/raw_state": "std_msgs/msg/Float64MultiArray",
    "/md/right/raw_state": "std_msgs/msg/Float64MultiArray",
    "/diagnostics": "diagnostic_msgs/msg/DiagnosticArray",
}
CONTEXT_TOPICS = {"/track/recorded_path": "nav_msgs/msg/Path"}
AUDITED_TOPICS = {
    **REQUIRED_TOPICS,
    **OPTIONAL_DIAGNOSTIC_TOPICS,
    **CONTEXT_TOPICS,
}


def portable_source_path(source: Path) -> str:
    """Represent workspace data without embedding an operator home path."""
    try:
        return source.relative_to(workspace_root()).as_posix()
    except ValueError:
        return str(source)


def bag_fingerprint(source: Path):
    """Return a fast identity check for an immutable compressed bag bundle."""
    metadata = source / "metadata.yaml"
    digest = hashlib.sha256(metadata.read_bytes()).hexdigest()
    files = []
    for item in sorted(source.iterdir()):
        if item.is_file() and (
            item.name.endswith(".db3") or item.name.endswith(".db3.zstd")
        ):
            files.append({"name": item.name, "size_bytes": item.stat().st_size})
    return {"metadata_sha256": digest, "data_files": files}


def _stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def uint8_scalar(value, *, field_name="uint8") -> int:
    """Normalize ROS uint8 scalars from Python or Humble CDR bindings.

    Most generated ROS 2 messages expose a scalar ``uint8`` as ``int``.  Some
    Humble rosbag/CDR combinations expose the same field as one byte instead.
    Accept both representations, but reject an empty/multi-byte value so a
    malformed diagnostic message cannot silently pass the field audit.
    """
    if isinstance(value, (bytes, bytearray, memoryview)):
        raw = bytes(value)
        if len(raw) != 1:
            raise ValueError(
                f"{field_name} must contain exactly one byte, got {len(raw)}"
            )
        return raw[0]
    normalized = int(value)
    if not 0 <= normalized <= 255:
        raise ValueError(f"{field_name} is outside uint8 range: {normalized}")
    return normalized


def _percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round(fraction * (len(ordered) - 1))))
    return float(ordered[index])


def _reader_for_copy(source: Path, temporary_root: Path):
    bag_copy = temporary_root / "bag"
    bag_copy.mkdir()
    shutil.copy2(source / "metadata.yaml", bag_copy / "metadata.yaml")
    for item in source.iterdir():
        if item.is_file() and (item.name.endswith(".db3.zstd") or item.name.endswith(".db3")):
            # Prefer the compressed authoritative product when both exist.
            if item.name.endswith(".db3") and (source / f"{item.name}.zstd").exists():
                continue
            shutil.copy2(item, bag_copy / item.name)
    has_compressed = any(bag_copy.glob("*.db3.zstd"))
    reader = (
        rosbag2_py.SequentialCompressionReader()
        if has_compressed
        else rosbag2_py.SequentialReader()
    )
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_copy), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    return reader


def nearest_time_differences(reference, samples):
    if not reference or not samples:
        return []
    ordered = np.asarray(sorted(samples), dtype=np.float64)
    values = np.asarray(reference, dtype=np.float64)
    indices = np.searchsorted(ordered, values)
    indices = np.clip(indices, 0, len(ordered) - 1)
    previous = np.clip(indices - 1, 0, len(ordered) - 1)
    choose_previous = np.abs(ordered[previous] - values) < np.abs(ordered[indices] - values)
    selected = np.where(choose_previous, previous, indices)
    return [float(value) for value in (ordered[selected] - values)]


def audit_bag(source: Path, *, require_diagnostics: bool = False):
    source = source.expanduser().resolve()
    if not (source / "metadata.yaml").is_file():
        raise ValueError(f"rosbag metadata is missing: {source}")

    with tempfile.TemporaryDirectory(prefix="roka_bag_audit_") as temporary:
        reader = _reader_for_copy(source, Path(temporary))
        type_names = {item.name: item.type for item in reader.get_all_topics_and_types()}
        types = {name: get_message(type_name) for name, type_name in type_names.items()}
        counts = defaultdict(int)
        record_times = defaultdict(list)
        header_times = defaultdict(list)
        transport_offsets = defaultdict(list)
        invalid_values = defaultdict(int)
        tf_parents = defaultdict(set)
        lidar_packets = []
        imu_gyro = []
        imu_accel_norms = []
        imu_quaternion_norms = []
        imu_orientation_covariance_available = []
        imu_angular_covariance_available = []
        imu_accel_covariance_available = []
        wheel_twist = []
        wheel_pose_covariance_nonzero = []
        wheel_twist_covariance_nonzero = []
        md_raw = defaultdict(list)
        md_connected_values = []
        diagnostic_levels = []
        diagnostic_events = defaultdict(int)
        bag_start_record_time = None
        latest_path_points = 0
        terrain_diagnostic_extrema = {
            "maximum_risk": 0.0,
            "maximum_abs_relative_roll_deg": 0.0,
            "maximum_abs_relative_pitch_deg": 0.0,
            "maximum_abs_dynamic_roll_load_deg": 0.0,
            "maximum_abs_dynamic_pitch_load_deg": 0.0,
        }
        terrain_extrema_locations = {}

        while reader.has_next():
            topic, serialized, record_time_ns = reader.read_next()
            if topic not in AUDITED_TOPICS:
                continue
            message = deserialize_message(serialized, types[topic])
            record_seconds = float(record_time_ns) * 1.0e-9
            if bag_start_record_time is None:
                bag_start_record_time = record_seconds
            counts[topic] += 1
            record_times[topic].append(record_seconds)

            if hasattr(message, "header"):
                header_seconds = _stamp_seconds(message.header.stamp)
                header_times[topic].append(header_seconds)
                transport_offsets[topic].append(record_seconds - header_seconds)

            if topic == "/track/recorded_path":
                latest_path_points = len(message.poses)
            elif topic == "/velodyne_packets":
                lidar_packets.append(len(message.packets))
            elif topic == "/imu/data":
                values = (
                    message.angular_velocity.x,
                    message.angular_velocity.y,
                    message.angular_velocity.z,
                    message.linear_acceleration.x,
                    message.linear_acceleration.y,
                    message.linear_acceleration.z,
                )
                if not all(math.isfinite(value) for value in values):
                    invalid_values[topic] += 1
                else:
                    imu_gyro.append((header_times[topic][-1], *values[:3]))
                    imu_accel_norms.append(math.sqrt(sum(value * value for value in values[3:])))
                quaternion = message.orientation
                quaternion_values = (
                    quaternion.x, quaternion.y, quaternion.z, quaternion.w
                )
                if all(math.isfinite(value) for value in quaternion_values):
                    imu_quaternion_norms.append(
                        math.sqrt(sum(value * value for value in quaternion_values))
                    )
                imu_orientation_covariance_available.append(
                    message.orientation_covariance[0] >= 0.0
                    and any(abs(value) > 0.0 for value in message.orientation_covariance)
                )
                imu_angular_covariance_available.append(
                    any(abs(value) > 0.0 for value in message.angular_velocity_covariance)
                )
                imu_accel_covariance_available.append(
                    any(abs(value) > 0.0 for value in message.linear_acceleration_covariance)
                )
            elif topic == "/wheel/odom":
                values = (
                    message.pose.pose.position.x,
                    message.pose.pose.position.y,
                    message.twist.twist.linear.x,
                    message.twist.twist.angular.z,
                )
                if not all(math.isfinite(value) for value in values):
                    invalid_values[topic] += 1
                else:
                    wheel_twist.append((header_times[topic][-1], values[2], values[3]))
                wheel_pose_covariance_nonzero.append(
                    any(abs(value) > 0.0 for value in message.pose.covariance)
                )
                wheel_twist_covariance_nonzero.append(
                    any(abs(value) > 0.0 for value in message.twist.covariance)
                )
            elif topic in {"/tf", "/tf_static"}:
                for transform in message.transforms:
                    tf_parents[transform.child_frame_id].add(transform.header.frame_id)
            elif topic in {"/md/left/raw_state", "/md/right/raw_state"}:
                if len(message.data) >= 6 and all(
                    math.isfinite(value) for value in message.data[:6]
                ):
                    md_raw[topic].append(tuple(float(value) for value in message.data[:6]))
                else:
                    invalid_values[topic] += 1
            elif topic == "/md/base_connected":
                md_connected_values.append(bool(message.data))
            elif topic == "/diagnostics":
                for status in message.status:
                    level = uint8_scalar(
                        status.level,
                        field_name="diagnostic_msgs/DiagnosticStatus.level",
                    )
                    diagnostic_levels.append(level)
                    diagnostic_events[(
                        str(status.name), level, str(status.message)
                    )] += 1
                    if str(status.name) == "terrain_supervisor/safety":
                        values = {item.key: item.value for item in status.values}
                        extrema_fields = {
                            "risk": "maximum_risk",
                            "relative_roll_deg": "maximum_abs_relative_roll_deg",
                            "relative_pitch_deg": "maximum_abs_relative_pitch_deg",
                            "dynamic_roll_load_deg": (
                                "maximum_abs_dynamic_roll_load_deg"
                            ),
                            "dynamic_pitch_load_deg": (
                                "maximum_abs_dynamic_pitch_load_deg"
                            ),
                        }
                        for source_name, output_name in extrema_fields.items():
                            try:
                                value = abs(float(values[source_name]))
                            except (KeyError, TypeError, ValueError):
                                continue
                            if math.isfinite(value):
                                if value > terrain_diagnostic_extrema[output_name]:
                                    terrain_diagnostic_extrema[output_name] = value
                                    terrain_extrema_locations[output_name] = {
                                        "mapping_elapsed_sec": round(
                                            record_seconds - bag_start_record_time, 3
                                        ),
                                        "recorded_path_points": latest_path_points,
                                    }

    topics = {}
    failures = []
    warnings = []
    thresholds = {"/velodyne_packets": 9.0, "/imu/data": 90.0, "/wheel/odom": 10.0}
    for topic, expected_type in REQUIRED_TOPICS.items():
        stamps = header_times[topic]
        intervals = [later - earlier for earlier, later in zip(stamps, stamps[1:])]
        positive_intervals = [interval for interval in intervals if interval > 0.0]
        backwards = sum(1 for interval in intervals if interval < 0.0)
        duplicates = sum(1 for interval in intervals if interval == 0.0)
        duration = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
        rate = (len(stamps) - 1) / duration if duration > 0.0 else 0.0
        type_ok = type_names.get(topic) == expected_type
        entry = {
            "type": type_names.get(topic),
            "expected_type": expected_type,
            "count": counts[topic],
            "rate_hz": round(rate, 3),
            "header_backwards": backwards,
            "header_duplicates": duplicates,
            "period_p95_ms": (
                round(1000.0 * _percentile(positive_intervals, 0.95), 3)
                if positive_intervals
                else None
            ),
            "record_minus_header_median_ms": (
                round(1000.0 * statistics.median(transport_offsets[topic]), 3)
                if transport_offsets[topic]
                else None
            ),
            "record_minus_header_p95_ms": (
                round(1000.0 * _percentile(transport_offsets[topic], 0.95), 3)
                if transport_offsets[topic]
                else None
            ),
            "invalid_values": invalid_values[topic],
        }
        topics[topic] = entry
        if not type_ok or counts[topic] == 0:
            failures.append(f"missing or wrong type: {topic}")
        if backwards:
            failures.append(f"non-monotonic header stamps: {topic}={backwards}")
        if invalid_values[topic]:
            failures.append(f"non-finite values: {topic}={invalid_values[topic]}")
        if topic in thresholds and rate < thresholds[topic]:
            failures.append(f"rate below threshold: {topic}={rate:.2f}Hz")

    for topic, expected_type in OPTIONAL_DIAGNOSTIC_TOPICS.items():
        stamps = header_times[topic] or record_times[topic]
        duration = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
        rate = (len(stamps) - 1) / duration if duration > 0.0 else 0.0
        topics[topic] = {
            "type": type_names.get(topic),
            "expected_type": expected_type,
            "count": counts[topic],
            "rate_hz": round(rate, 3),
            "invalid_values": invalid_values[topic],
        }
        if counts[topic] == 0:
            message = f"diagnostic topic was not recorded: {topic}"
            if require_diagnostics:
                failures.append(message)
            else:
                warnings.append(message + "; required for new sessions")
        elif type_names.get(topic) != expected_type:
            failures.append(f"wrong diagnostic topic type: {topic}")
        elif invalid_values[topic]:
            failures.append(f"non-finite diagnostic values: {topic}")

    multiple_parents = {
        child: sorted(parents) for child, parents in tf_parents.items() if len(parents) > 1
    }
    if multiple_parents:
        failures.append(f"TF children with multiple parents: {multiple_parents}")

    imu_covariance = {
        "orientation_available_fraction": round(
            statistics.mean(imu_orientation_covariance_available), 4
        ) if imu_orientation_covariance_available else None,
        "angular_velocity_available_fraction": round(
            statistics.mean(imu_angular_covariance_available), 4
        ) if imu_angular_covariance_available else None,
        "linear_acceleration_available_fraction": round(
            statistics.mean(imu_accel_covariance_available), 4
        ) if imu_accel_covariance_available else None,
    }
    if imu_quaternion_norms and max(
        abs(value - 1.0) for value in imu_quaternion_norms
    ) > 0.05:
        failures.append("IMU quaternion norm differs from one by more than 0.05")
    if imu_covariance["angular_velocity_available_fraction"] != 1.0:
        failures.append("IMU angular-velocity covariance is unavailable")
    if imu_covariance["linear_acceleration_available_fraction"] != 1.0:
        failures.append("IMU acceleration covariance is unavailable")

    wheel_covariance = {
        "pose_nonzero_fraction": round(
            statistics.mean(wheel_pose_covariance_nonzero), 4
        ) if wheel_pose_covariance_nonzero else None,
        "twist_nonzero_fraction": round(
            statistics.mean(wheel_twist_covariance_nonzero), 4
        ) if wheel_twist_covariance_nonzero else None,
    }
    if wheel_covariance["pose_nonzero_fraction"] != 1.0:
        failures.append("wheel odometry pose covariance is zero or unavailable")
    if wheel_covariance["twist_nonzero_fraction"] != 1.0:
        failures.append("wheel odometry twist covariance is zero or unavailable")

    lidar_imu_differences = nearest_time_differences(
        header_times["/velodyne_packets"], header_times["/imu/data"]
    )
    absolute_lidar_imu = [abs(value) for value in lidar_imu_differences]
    timing = {
        "nearest_imu_minus_lidar_median_ms": (
            float(round(1000.0 * statistics.median(lidar_imu_differences), 3))
            if lidar_imu_differences else None
        ),
        "nearest_absolute_difference_p95_ms": (
            float(round(1000.0 * _percentile(absolute_lidar_imu, 0.95), 3))
            if absolute_lidar_imu else None
        ),
    }
    if absolute_lidar_imu and _percentile(absolute_lidar_imu, 0.95) > 0.020:
        failures.append("LiDAR/IMU nearest timestamp difference p95 exceeds 20 ms")

    excitation = {
        "imu_peak_abs_gyro_radps": [
            round(max((abs(row[axis]) for row in imu_gyro), default=0.0), 4)
            for axis in (1, 2, 3)
        ],
        "wheel_peak_abs_linear_mps": round(
            max((abs(row[1]) for row in wheel_twist), default=0.0), 4
        ),
        "wheel_peak_abs_yaw_radps": round(
            max((abs(row[2]) for row in wheel_twist), default=0.0), 4
        ),
    }
    excitation["sufficient_yaw_excitation"] = (
        excitation["imu_peak_abs_gyro_radps"][2] >= 0.15
        and excitation["wheel_peak_abs_yaw_radps"] >= 0.10
    )
    excitation["sufficient_roll_pitch_excitation"] = (
        max(excitation["imu_peak_abs_gyro_radps"][:2]) >= 0.10
    )

    md_summary = {}
    for topic in ("/md/left/raw_state", "/md/right/raw_state"):
        samples = md_raw[topic]
        md_summary[topic] = {
            "samples": len(samples),
            "peak_current_a": round(
                max((max(abs(row[2]), abs(row[3])) for row in samples), default=0.0),
                3,
            ),
            "nonzero_state_samples": sum(
                row[4] != 0.0 or row[5] != 0.0 for row in samples
            ),
        }

    imu_values = {
        "acceleration_norm_mps2": {
            "median": round(statistics.median(imu_accel_norms), 4)
            if imu_accel_norms else None,
            "p05": round(_percentile(imu_accel_norms, 0.05), 4)
            if imu_accel_norms else None,
            "p95": round(_percentile(imu_accel_norms, 0.95), 4)
            if imu_accel_norms else None,
        },
        "quaternion_norm": {
            "minimum": round(min(imu_quaternion_norms), 6)
            if imu_quaternion_norms else None,
            "maximum": round(max(imu_quaternion_norms), 6)
            if imu_quaternion_norms else None,
        },
        "covariance": imu_covariance,
    }

    diagnostic_issues = [
        {
            "name": name,
            "level": level,
            "severity": {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}.get(
                level, f"LEVEL_{level}"
            ),
            "message": message,
            "count": count,
        }
        for (name, level, message), count in sorted(
            diagnostic_events.items(),
            key=lambda item: (-item[0][1], -item[1], item[0][0], item[0][2]),
        )
        if level >= 1
    ]
    diagnostic_error_count = sum(level >= 2 for level in diagnostic_levels)
    if diagnostic_error_count:
        warnings.append(
            f"diagnostics recorded {diagnostic_error_count} ERROR/STALE statuses; "
            "see diagnostics.issues"
        )

    return {
        "schema_version": 3,
        "source": portable_source_path(source),
        "bag_fingerprint": bag_fingerprint(source),
        "result": "PASS" if not failures else "FAIL",
        "failures": failures,
        "warnings": warnings,
        "topics": topics,
        "velodyne_packets_per_scan": {
            "minimum": min(lidar_packets) if lidar_packets else None,
            "maximum": max(lidar_packets) if lidar_packets else None,
            "median": statistics.median(lidar_packets) if lidar_packets else None,
        },
        "lidar_imu_timing": timing,
        "calibration_excitation": excitation,
        "imu_values": imu_values,
        "wheel_odometry_covariance": wheel_covariance,
        "md_feedback": {
            "base_connected_false_samples": sum(
                not value for value in md_connected_values
            ),
            "controllers": md_summary,
        },
        "diagnostics": {
            "status_count": len(diagnostic_levels),
            "ok_status_count": sum(level == 0 for level in diagnostic_levels),
            "warning_status_count": sum(level == 1 for level in diagnostic_levels),
            "error_status_count": diagnostic_error_count,
            "issues": diagnostic_issues,
            "terrain_extrema": {
                name: round(value, 4)
                for name, value in terrain_diagnostic_extrema.items()
            },
            "terrain_extrema_locations": terrain_extrema_locations,
        },
        "tf_parents": {child: sorted(parents) for child, parents in sorted(tf_parents.items())},
        "runtime_checks_still_required": [
            "/velodyne_points fields x,y,z,intensity,ring,time",
            "LiDAR-to-IMU physical time offset under sufficient angular motion",
            "single live publisher ownership for dynamic TF",
        ],
    }


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "bag",
        help="rosbag2 directory, for example maps/flat/section_1/sensors",
    )
    parser.add_argument("--output", help="optional YAML report path")
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="print only rates and diagnostic issue counts; still write full --output",
    )
    parser.add_argument(
        "--require-diagnostics",
        action="store_true",
        help="fail if MD feedback/connection or /diagnostics was not recorded",
    )
    args = parser.parse_args(argv)
    report = audit_bag(
        Path(args.bag), require_diagnostics=args.require_diagnostics
    )
    rendered = yaml.safe_dump(report, sort_keys=False, allow_unicode=True)
    if args.summary_only:
        topics = report["topics"]
        diagnostics = report["diagnostics"]
        print(
            f"[SENSOR-AUDIT][RESULT] sensors={report['result']} | "
            f"VLP={topics['/velodyne_packets']['rate_hz']:.1f}Hz, "
            f"IMU={topics['/imu/data']['rate_hz']:.1f}Hz, "
            f"ODOM={topics['/wheel/odom']['rate_hz']:.1f}Hz, "
            f"diagnostic WARN/ERROR="
            f"{diagnostics['warning_status_count']}/"
            f"{diagnostics['error_status_count']}"
        )
        for issue in diagnostics["issues"][:5]:
            print(
                f"[SENSOR-AUDIT][RECORDED-{issue['severity']}] {issue['name']}: "
                f"{issue['message']} ({issue['count']}회)"
            )
        print(f"[SENSOR-AUDIT][REPORT] {args.output or 'not written'}")
    else:
        print(rendered, end="")
    if args.output:
        output = Path(args.output).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered, encoding="utf-8")
    raise SystemExit(0 if report["result"] == "PASS" else 1)


if __name__ == "__main__":
    main()
