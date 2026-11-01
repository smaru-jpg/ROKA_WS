"""Finalize a slam_toolbox map after an offline raw-bag replay."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import rclpy
from std_srvs.srv import Trigger

from .save_map_bundle import BundleSaver


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("output_directory")
    args = parser.parse_args(argv)
    output = Path(args.output_directory).expanduser().resolve()

    rclpy.init()
    saver = BundleSaver(
        output,
        environment="processed_2d",
        mapping_backend="slam_toolbox_2d",
        use_global_arguments=False,
    )
    try:
        path_result = saver.call(
            Trigger,
            "/track_path_recorder/stop_and_save",
            Trigger.Request(),
        )
        if not path_result.success:
            raise RuntimeError(path_result.message)
        saver.save()
        print(f"[PROCESS][2D] saved occupancy and aligned route: {output}")
        return 0
    except Exception as error:
        print(f"[PROCESS][2D][FAIL] {error}", file=sys.stderr)
        return 1
    finally:
        saver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
