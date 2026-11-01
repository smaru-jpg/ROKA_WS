"""Validate the metrically closed PK-3A loop candidate."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .generate_closed_loop import default_config_path, default_output_root
from .closed_loop_validation import validate


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate PK-3A closed loop")
    parser.add_argument("--config", type=Path, default=default_config_path())
    parser.add_argument("--output-root", type=Path, default=default_output_root())
    args = parser.parse_args()
    result = validate(args.config.resolve(), args.output_root.resolve())
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if not result["passed"]:
        sys.exit(1)


if __name__ == "__main__":
    main()
