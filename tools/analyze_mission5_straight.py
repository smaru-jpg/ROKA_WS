#!/usr/bin/env python3
"""Reconstruct a straight driving segment from an existing ROKA rosbag.

The script compares the previous wheel-differential heading integration with
the Section-5 wheel-distance + EBIMU-heading integration.  It does not publish
commands or access hardware.
"""

import argparse
import bisect
import json
import math
import shutil
import tempfile
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


WHEEL_RADIUS_M = 0.103
# Real four-wheel skid-steer calibration.  Physical wheel track remains
# 0.427149994 m and must not replace this odometry parameter.
EFFECTIVE_SEPARATION_M = 0.66


def yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def physical_rpm(data):
    # Both MD200T controllers use feedback signs MOT1=+1, MOT2=-1.
    return 0.5 * (float(data[0]) - float(data[1]))


def nearest_index(times, stamp):
    index = bisect.bisect_left(times, stamp)
    if index == 0:
        return 0
    if index == len(times):
        return len(times) - 1
    return index if times[index] - stamp < stamp - times[index - 1] else index - 1


def _read_bag(uri):
    reader = rosbag2_py.SequentialCompressionReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: get_message(item.type) for item in reader.get_all_topics_and_types()
    }
    wanted = {
        "/cmd_vel",
        "/imu/data",
        "/md/left/raw_state",
        "/md/right/raw_state",
    }
    samples = {name: [] for name in wanted}
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        if topic not in wanted:
            continue
        message = deserialize_message(raw, topic_types[topic])
        stamp = stamp_ns * 1.0e-9
        if topic == "/cmd_vel":
            value = (float(message.linear.x), float(message.angular.z))
        elif topic == "/imu/data":
            value = yaw_from_quaternion(message.orientation)
        else:
            if len(message.data) < 2:
                continue
            value = physical_rpm(message.data)
        samples[topic].append((stamp, value))
    return samples


def read_bag(uri):
    """Read compressed bags without leaving a decompressed DB beside the source."""
    uri = Path(uri)
    compressed_files = list(uri.glob("*.zstd"))
    if not compressed_files:
        return _read_bag(uri)
    with tempfile.TemporaryDirectory(prefix="roka_straight_bag_") as temp_dir:
        temporary_uri = Path(temp_dir) / "bag"
        temporary_uri.mkdir()
        shutil.copy2(uri / "metadata.yaml", temporary_uri / "metadata.yaml")
        for source in compressed_files:
            shutil.copy2(source, temporary_uri / source.name)
        return _read_bag(temporary_uri)


def synchronized_samples(samples):
    left = samples["/md/left/raw_state"]
    right = samples["/md/right/raw_state"]
    imu = samples["/imu/data"]
    cmd = samples["/cmd_vel"]
    if not left or not right or not imu or not cmd:
        raise RuntimeError("bag lacks cmd_vel, IMU, or left/right MD feedback")

    right_t = [item[0] for item in right]
    imu_t = [item[0] for item in imu]
    cmd_t = [item[0] for item in cmd]
    synced = []
    for stamp, left_rpm in left:
        ri = nearest_index(right_t, stamp)
        ii = nearest_index(imu_t, stamp)
        ci = nearest_index(cmd_t, stamp)
        if abs(right[ri][0] - stamp) > 0.15 or abs(imu[ii][0] - stamp) > 0.08:
            continue
        linear_cmd, angular_cmd = cmd[ci][1]
        synced.append(
            (stamp, left_rpm, right[ri][1], imu[ii][1], linear_cmd, angular_cmd)
        )
    return synced


def find_straight_segments(samples):
    groups = []
    current = []
    previous_stamp = None
    previous_sign = 0
    for item in samples:
        stamp, left_rpm, right_rpm, _yaw, linear_cmd, angular_cmd = item
        measured_speed = 0.5 * (left_rpm + right_rpm) * 2.0 * math.pi * WHEEL_RADIUS_M / 60.0
        sign = 1 if linear_cmd > 0.0 else -1
        eligible = (
            abs(linear_cmd) >= 0.08
            and abs(angular_cmd) <= 0.05
            and abs(measured_speed) >= 0.04
            and left_rpm * right_rpm > 0.0
        )
        continuous = previous_stamp is not None and stamp - previous_stamp <= 0.45
        if eligible and (not current or (continuous and sign == previous_sign)):
            current.append(item)
        else:
            if len(current) >= 8:
                groups.append(current)
            current = [item] if eligible else []
        previous_stamp = stamp
        previous_sign = sign
    if len(current) >= 8:
        groups.append(current)
    return groups


def reconstruct(segment):
    imu_x = [0.0]
    imu_y = [0.0]
    wheel_x = [0.0]
    wheel_y = [0.0]
    distance = [0.0]
    relative_yaw = 0.0
    wheel_yaw = 0.0
    previous_imu = segment[0][3]
    previous_stamp = segment[0][0]

    for stamp, left_rpm, right_rpm, imu_yaw, _linear_cmd, _angular_cmd in segment[1:]:
        dt = min(0.30, max(0.0, stamp - previous_stamp))
        delta_yaw = normalize_angle(imu_yaw - previous_imu)
        relative_yaw += delta_yaw
        previous_imu = imu_yaw

        left_mps = left_rpm * 2.0 * math.pi * WHEEL_RADIUS_M / 60.0
        right_mps = right_rpm * 2.0 * math.pi * WHEEL_RADIUS_M / 60.0
        linear = 0.5 * (left_mps + right_mps)
        wheel_yaw += (right_mps - left_mps) / EFFECTIVE_SEPARATION_M * dt
        ds = linear * dt

        imu_x.append(imu_x[-1] + ds * math.cos(relative_yaw))
        imu_y.append(imu_y[-1] + ds * math.sin(relative_yaw))
        wheel_x.append(wheel_x[-1] + ds * math.cos(wheel_yaw))
        wheel_y.append(wheel_y[-1] + ds * math.sin(wheel_yaw))
        distance.append(distance[-1] + abs(ds))
        previous_stamp = stamp

    return {
        "imu_x": imu_x,
        "imu_y": imu_y,
        "wheel_x": wheel_x,
        "wheel_y": wheel_y,
        "distance": distance,
        "imu_yaw_change_deg": math.degrees(relative_yaw),
        "wheel_yaw_change_deg": math.degrees(wheel_yaw),
    }


def lateral_errors(xs, ys):
    dx = xs[-1] - xs[0]
    dy = ys[-1] - ys[0]
    length = math.hypot(dx, dy)
    if length < 1.0e-9:
        return [0.0 for _ in xs]
    return [((x - xs[0]) * dy - (y - ys[0]) * dx) / length for x, y in zip(xs, ys)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary", type=Path)
    args = parser.parse_args()

    samples = synchronized_samples(read_bag(args.bag))
    segments = find_straight_segments(samples)
    if not segments:
        raise RuntimeError("no sustained straight cmd_vel segment found")
    candidates = [(reconstruct(segment), segment) for segment in segments]
    result, segment = max(candidates, key=lambda item: item[0]["distance"][-1])

    imu_errors = lateral_errors(result["imu_x"], result["imu_y"])
    wheel_errors = lateral_errors(result["wheel_x"], result["wheel_y"])
    duration = segment[-1][0] - segment[0][0]
    summary = {
        "bag": str(args.bag.resolve()),
        "segment_start_sec_from_bag": segment[0][0] - samples[0][0],
        "duration_sec": duration,
        "travel_distance_m": result["distance"][-1],
        "imu_heading_change_deg": result["imu_yaw_change_deg"],
        "wheel_heading_change_deg": result["wheel_yaw_change_deg"],
        "imu_max_lateral_error_m": max(abs(value) for value in imu_errors),
        "wheel_max_lateral_error_m": max(abs(value) for value in wheel_errors),
        "straight_segments_found": len(segments),
        "samples": len(segment),
    }

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
    axis = axes[0]
    axis.plot(result["imu_x"], result["imu_y"], color="#00a6d6", linewidth=2.4,
              label="Section 5: wheel distance + EBIMU yaw")
    axis.plot(result["wheel_x"], result["wheel_y"], color="#f28e2b", linewidth=1.5,
              alpha=0.85, label="Previous: wheel differential yaw")
    axis.plot(
        [result["imu_x"][0], result["imu_x"][-1]],
        [result["imu_y"][0], result["imu_y"][-1]],
        "k--", linewidth=1.2, label="Straight reference (start to end)",
    )
    axis.scatter([0.0], [0.0], color="green", s=55, label="start", zorder=5)
    axis.scatter([result["imu_x"][-1]], [result["imu_y"][-1]],
                 color="red", s=55, label="end", zorder=5)
    axis.set_title("Existing-data straight segment reconstruction")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.axis("equal")
    axis.grid(True, alpha=0.3)
    axis.legend(fontsize=8)

    axis = axes[1]
    axis.plot(result["distance"], imu_errors, color="#00a6d6", linewidth=2.2,
              label="Section 5 lateral deviation")
    axis.plot(result["distance"], wheel_errors, color="#f28e2b", linewidth=1.5,
              label="Wheel-only lateral deviation")
    axis.axhline(0.0, color="black", linestyle="--", linewidth=1.0)
    axis.set_title("Deviation from each start-to-end reference")
    axis.set_xlabel("travel distance [m]")
    axis.set_ylabel("lateral deviation [m]")
    axis.grid(True, alpha=0.3)
    axis.legend(fontsize=8)

    fig.suptitle(
        f"{duration:.1f} s / {summary['travel_distance_m']:.2f} m | "
        f"EBIMU max deviation {summary['imu_max_lateral_error_m']:.3f} m",
        fontsize=12,
    )
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=170)
    if args.summary:
        args.summary.parent.mkdir(parents=True, exist_ok=True)
        args.summary.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
