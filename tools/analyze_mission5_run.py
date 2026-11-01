#!/usr/bin/env python3
"""Summarize and plot a recorded Section-5 run without replaying commands."""

import argparse
import json
import math
from collections import Counter
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "/mission5/tracking_odom",
    "/odom",
    "/mission5/leader_trail",
    "/mission5/gap_m",
    "/mission5/status",
    "/mission5/control_mode",
    "/mission5/local_curvature",
    "/mission5/heading_error_rad",
    "/mission5/cross_track_error_m",
    "/cmd_vel_mission5_raw",
    "/cmd_vel_mission5_safe",
    "/cmd_vel_mission5_lifecycle_safe",
    "/cmd_vel_arm_locked",
    "/md/commanded_rpm",
    "/md/left/raw_state",
    "/md/right/raw_state",
}


def path_length(points):
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:])
    )


def yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def unwrap(values):
    if not values:
        return []
    result = [values[0]]
    for value in values[1:]:
        delta = math.atan2(math.sin(value - result[-1]), math.cos(value - result[-1]))
        result.append(result[-1] + delta)
    return result


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    index = min(len(ordered) - 1, max(0, round(fraction * (len(ordered) - 1))))
    return ordered[index]


def read(uri):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {
        item.name: get_message(item.type)
        for item in reader.get_all_topics_and_types()
        if item.name in TOPICS
    }
    records = {topic: [] for topic in TOPICS}
    first_stamp = None
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        if topic not in types:
            continue
        stamp = stamp_ns * 1.0e-9
        if first_stamp is None:
            first_stamp = stamp
        records[topic].append((stamp, deserialize_message(raw, types[topic])))
    return first_stamp, records


def odom_series(records):
    points = []
    headings = []
    times = []
    if not records:
        return times, points, headings
    x0 = records[0][1].pose.pose.position.x
    y0 = records[0][1].pose.pose.position.y
    yaw0 = yaw(records[0][1].pose.pose.orientation)
    cosine, sine = math.cos(yaw0), math.sin(yaw0)
    for stamp, message in records:
        dx = message.pose.pose.position.x - x0
        dy = message.pose.pose.position.y - y0
        # Express both odometry streams in their initial robot frame.
        points.append((cosine * dx + sine * dy, -sine * dx + cosine * dy))
        headings.append(yaw(message.pose.pose.orientation) - yaw0)
        times.append(stamp)
    return times, points, unwrap(headings)


def final_trail(records, tracking_start):
    if not records:
        return []
    message = max((message for _stamp, message in records), key=lambda item: len(item.poses))
    if not message.poses:
        return []
    x0, y0, yaw0 = tracking_start
    cosine, sine = math.cos(yaw0), math.sin(yaw0)
    result = []
    for pose in message.poses:
        dx = pose.pose.position.x - x0
        dy = pose.pose.position.y - y0
        result.append((cosine * dx + sine * dy, -sine * dx + cosine * dy))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    args = parser.parse_args()

    first_stamp, records = read(args.bag)
    tracking_records = records["/mission5/tracking_odom"]
    wheel_records = records["/odom"]
    if not tracking_records:
        raise RuntimeError("bag has no /mission5/tracking_odom")
    tracking_start_message = tracking_records[0][1].pose.pose
    tracking_start = (
        tracking_start_message.position.x,
        tracking_start_message.position.y,
        yaw(tracking_start_message.orientation),
    )
    tracking_t, tracking_xy, tracking_yaw = odom_series(tracking_records)
    wheel_t, wheel_xy, wheel_yaw = odom_series(wheel_records)
    trail_xy = final_trail(records["/mission5/leader_trail"], tracking_start)

    status = [message.data for _stamp, message in records["/mission5/status"]]
    control_modes = [
        message.data for _stamp, message in records["/mission5/control_mode"]
    ]
    local_curvature = [
        abs(float(message.data))
        for _stamp, message in records["/mission5/local_curvature"]
    ]
    heading_error = [
        abs(float(message.data))
        for _stamp, message in records["/mission5/heading_error_rad"]
    ]
    cross_track_error = [
        abs(float(message.data))
        for _stamp, message in records["/mission5/cross_track_error_m"]
    ]
    gaps = [float(message.data) for _stamp, message in records["/mission5/gap_m"]]
    raw_cmd = [
        (stamp, float(message.linear.x), float(message.angular.z))
        for stamp, message in records["/cmd_vel_mission5_raw"]
    ]
    safe_cmd = [
        (stamp, float(message.linear.x), float(message.angular.z))
        for stamp, message in records["/cmd_vel_mission5_safe"]
    ]
    lifecycle_cmd = [
        (stamp, float(message.linear.x), float(message.angular.z))
        for stamp, message in records["/cmd_vel_mission5_lifecycle_safe"]
    ]
    arm_cmd = [
        (stamp, float(message.linear.x), float(message.angular.z))
        for stamp, message in records["/cmd_vel_arm_locked"]
    ]
    commanded_rpm = [
        abs(float(value))
        for _stamp, message in records["/md/commanded_rpm"]
        for value in message.data[:2]
    ]
    rpm = []
    for topic in ("/md/left/raw_state", "/md/right/raw_state"):
        for _stamp, message in records[topic]:
            rpm.extend(abs(float(value)) for value in message.data[:2])

    valid_gaps = [value for value in gaps if math.isfinite(value) and value >= 0.0]
    summary = {
        "bag": str(args.bag.resolve()),
        "duration_sec": max(tracking_t) - min(tracking_t),
        "tracking_distance_m": path_length(tracking_xy),
        "tracking_displacement_m": math.hypot(tracking_xy[-1][0], tracking_xy[-1][1]),
        "tracking_yaw_change_deg": math.degrees(tracking_yaw[-1]),
        "wheel_distance_m": path_length(wheel_xy),
        "wheel_yaw_change_deg": math.degrees(wheel_yaw[-1]) if wheel_yaw else None,
        "final_tracking_vs_wheel_offset_m": (
            math.hypot(
                tracking_xy[-1][0] - wheel_xy[-1][0],
                tracking_xy[-1][1] - wheel_xy[-1][1],
            )
            if wheel_xy else None
        ),
        "leader_trail_points": len(trail_xy),
        "leader_trail_length_m": path_length(trail_xy),
        "gap_min_m": min(valid_gaps) if valid_gaps else None,
        "gap_median_m": percentile(valid_gaps, 0.5),
        "gap_p95_m": percentile(valid_gaps, 0.95),
        "gap_max_m": max(valid_gaps) if valid_gaps else None,
        "raw_max_linear_mps": max((abs(value[1]) for value in raw_cmd), default=0.0),
        "raw_max_angular_radps": max((abs(value[2]) for value in raw_cmd), default=0.0),
        "safe_max_linear_mps": max((abs(value[1]) for value in safe_cmd), default=0.0),
        "safe_max_angular_radps": max((abs(value[2]) for value in safe_cmd), default=0.0),
        "lifecycle_max_linear_mps": max(
            (abs(value[1]) for value in lifecycle_cmd), default=0.0
        ),
        "arm_gate_max_linear_mps": max(
            (abs(value[1]) for value in arm_cmd), default=0.0
        ),
        "maximum_commanded_motor_rpm": (
            max(commanded_rpm) if commanded_rpm else None
        ),
        "maximum_raw_motor_rpm": max(rpm) if rpm else None,
        "status_counts": dict(Counter(status)),
        "control_mode_counts": dict(Counter(control_modes)),
        "local_curvature_p95_inv_m": percentile(local_curvature, 0.95),
        "heading_error_p95_deg": (
            math.degrees(percentile(heading_error, 0.95)) if heading_error else None
        ),
        "cross_track_error_p95_m": percentile(cross_track_error, 0.95),
    }

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    axis = axes[0]
    if trail_xy:
        axis.plot([p[0] for p in trail_xy], [p[1] for p in trail_xy],
                  color="#1f77b4", linewidth=2.2, label="final leader trail")
    axis.plot([p[0] for p in tracking_xy], [p[1] for p in tracking_xy],
              color="#00a676", linewidth=1.8, label="robot: wheel distance + EBIMU yaw")
    if wheel_xy:
        axis.plot([p[0] for p in wheel_xy], [p[1] for p in wheel_xy],
                  color="#f28e2b", linewidth=1.2, alpha=0.8,
                  label="robot: wheel-only odometry")
    axis.scatter([0.0], [0.0], color="green", s=55, label="start", zorder=5)
    axis.scatter([tracking_xy[-1][0]], [tracking_xy[-1][1]],
                 color="red", s=55, label="end", zorder=5)
    axis.set_title("Section-5 recorded trajectories")
    axis.set_xlabel("forward x [m]")
    axis.set_ylabel("left y [m]")
    axis.axis("equal")
    axis.grid(True, alpha=0.3)
    axis.legend(fontsize=8)

    axis = axes[1]
    if valid_gaps:
        gap_t = [stamp - first_stamp for stamp, message in records["/mission5/gap_m"]
                 if math.isfinite(message.data) and message.data >= 0.0]
        axis.plot(gap_t, valid_gaps, color="#9467bd", linewidth=1.4,
                  label="target gap [m]")
        axis.axhline(2.0, color="black", linestyle="--", linewidth=1.0,
                     label="nominal follow distance")
    if raw_cmd:
        axis.plot([v[0] - first_stamp for v in raw_cmd], [v[1] for v in raw_cmd],
                  color="#2ca02c", linewidth=1.0, alpha=0.8,
                  label="raw linear cmd [m/s]")
        axis.plot([v[0] - first_stamp for v in raw_cmd], [v[2] for v in raw_cmd],
                  color="#d62728", linewidth=1.0, alpha=0.7,
                  label="raw angular cmd [rad/s]")
    axis.set_title("Tracking gap and commands")
    axis.set_xlabel("time from bag start [s]")
    axis.grid(True, alpha=0.3)
    axis.legend(fontsize=8)

    fig.suptitle(
        f"Section 5 | robot {summary['tracking_distance_m']:.1f} m | "
        f"leader trail {summary['leader_trail_length_m']:.1f} m | "
        f"max safe speed {summary['safe_max_linear_mps']:.2f} m/s"
    )
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=170)
    args.summary.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
                            encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
