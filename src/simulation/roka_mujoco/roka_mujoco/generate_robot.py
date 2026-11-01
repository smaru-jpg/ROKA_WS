"""Generate the delivered ROKA robot assets and PK-3A combined scene."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .generate_closed_loop import default_output_root
from .module_geometry import package_root
from .robot_model import generate_robot


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate ROKA robot assets from STEP and STL"
    )
    parser.add_argument(
        "--step",
        type=Path,
        default=Path.home() / "Downloads" / "ASSSEMBLEEE.STEP",
    )
    parser.add_argument(
        "--stl",
        type=Path,
        default=Path.home() / "Downloads" / "asssembleee.stl",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=package_root() / "config" / "roka_robot.json",
    )
    parser.add_argument("--output-root", type=Path, default=default_output_root())
    args = parser.parse_args()
    result = generate_robot(
        args.step.resolve(),
        args.stl.resolve(),
        args.config.resolve(),
        args.output_root.resolve(),
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
