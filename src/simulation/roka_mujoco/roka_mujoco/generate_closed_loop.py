"""Generate the metrically closed PK-3A loop candidate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .closed_loop_pipeline import generate
from .module_geometry import package_root


def default_config_path() -> Path:
    return package_root() / "config" / "pk3a_closed_loop.json"


def default_output_root() -> Path:
    return package_root() / "description" / "pk3a"


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate PK-3A closed loop")
    parser.add_argument("--config", type=Path, default=default_config_path())
    parser.add_argument("--output-root", type=Path, default=default_output_root())
    args = parser.parse_args()
    result = generate(args.config.resolve(), args.output_root.resolve())
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
