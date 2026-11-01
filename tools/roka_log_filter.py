#!/usr/bin/env python3
"""Keep the operator terminal concise while preserving actionable failures."""

from __future__ import annotations

from collections import Counter
import re
import signal
import sys


NOISY_GROUPS = (
    ("tf_wait", ("passed to lookupTransform", "waiting for transform", "without TF"),
     "TF가 아직 준비되지 않음"),
    ("slam_pose", ("Failed to compute odom pose",), "SLAM odom pose 계산 대기"),
    ("qos", ("incompatible QoS",), "QoS 불일치"),
    ("collision_time", ("timestamps differ",), "충돌 포인트클라우드 시간 지연"),
    ("rviz_shader", ("active samplers with a different type", "GLSL link result"),
     "RViz 그래픽 드라이버 경고"),
)

SUPPRESSED_INFO = (
    "All log files can be found below",
    "Default logging verbosity",
    "process started with pid",
    "process has finished cleanly",
    "Stereo is NOT SUPPORTED",
    "OpenGl version",
    "number of map points",
)

IMPORTANT_TAGS = (
    "[ROKA]", "[FIELD-CHECK]", "[XBOX-", "[MAPPING-BAG]",
    "[MAP-CHECK]", "[CHECK]", "[AUTONOMY-PIPELINE]", "[LOCALIZATION]",
    "[TRACK-", "[MAP-BUNDLE]", "[SENSOR-AUDIT]",
    "[PROCESS]",
    "[ARM-CHECK]", "[ROKA-ARM]", "[ARM]",
    "[MISSION-1]", "[MISSION-2]", "[MD-HOLD]", "[ROKA-HANDOFF]",
    "[MISSION5-GATE]",
    "[Initialization]", "[Init Result]", "[Refinement]", "[Final Result]",
    "[CALIBRATION]",
    "[FAIL]", "[SAFETY STOP]", "[LIMIT]", "[TIMEOUT]",
)

PLAIN_FAILURES = (
    "No such file or directory",
    "could not open port",
    "Port Open failure",
    "SerialException",
    "Permission denied",
    "exited with code",
    "process exited",
    "Aborted",
    "terminate called",
    "CUDA error",
)


def normalized(line: str) -> str:
    value = re.sub(r"\[\d{9,}(?:\.\d+)?\]", "[time]", line)
    value = re.sub(r"pid \[?\d+\]?", "pid", value)
    return value.strip()


def main() -> None:
    # Ctrl+C belongs to the launched ROS stack.  Keep draining its final
    # shutdown messages instead of raising KeyboardInterrupt and closing the
    # pipe early, which used to make ros2 report the misleading exit code 120.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    repeats: Counter[str] = Counter()
    labels: dict[str, str] = {}
    seen: Counter[str] = Counter()
    traceback_lines = 0

    for raw in sys.stdin:
        line = raw.rstrip("\n")
        if not line.strip():
            continue
        # A completed offline processor classifies a failed LIO quality gate as
        # DEGRADED and preserves the map for diagnosis.  ros2 launch still logs
        # the monitor's intentional exit(1) as "process has died"; the explicit
        # [PROCESS][3D] line below is the actionable operator result.
        if "lio_regression_monitor" in line and "process has died" in line:
            continue
        if any(text in line for text in SUPPRESSED_INFO):
            continue

        group = None
        for key, patterns, label in NOISY_GROUPS:
            if any(pattern in line for pattern in patterns):
                group = key
                labels[key] = label
                break
        if group is not None:
            repeats[group] += 1
            if repeats[group] == 1:
                print(f"[LOG][WARN] {labels[group]} (반복은 숨김)", flush=True)
            continue

        key = normalized(line)
        important = any(tag in line for tag in IMPORTANT_TAGS)
        error = (
            "[ERROR]" in line or "Traceback (most recent call last)" in line
            or "process has died" in line or "Caught exception" in line
            or "RuntimeError:" in line or "ValueError:" in line
            or any(text in line for text in PLAIN_FAILURES)
        )
        warning = "[WARN]" in line or "[WARNING]" in line
        if "Traceback (most recent call last)" in line:
            traceback_lines = 12
        elif traceback_lines > 0:
            traceback_lines -= 1

        if important or error or warning or traceback_lines > 0:
            seen[key] += 1
            if seen[key] == 1:
                print(line, flush=True)

    for key, count in sorted(repeats.items()):
        if count > 1:
            print(f"[LOG][SUMMARY] {labels[key]}: {count}회", flush=True)
    duplicate_count = sum(count - 1 for count in seen.values() if count > 1)
    if duplicate_count:
        print(f"[LOG][SUMMARY] 동일 경고/오류 {duplicate_count}줄 추가 반복 생략", flush=True)


if __name__ == "__main__":
    main()
