"""Select new and latest valid real-map bundles without overwriting runs."""

from pathlib import Path
import re

import yaml

from roka_nav2_bringup.map_bundle_quality import bundle_is_complete


TRACK_DIRECTORY_PATTERN = re.compile(r"track_v([0-9]+)$")
MAP_ID_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}")


def map_bundle_for_id(map_root, map_id):
    """Resolve a human-selected map name without permitting path traversal."""
    name = str(map_id).strip()
    if not MAP_ID_PATTERN.fullmatch(name):
        raise ValueError(
            "map_id must contain only letters, numbers, '_' or '-' "
            "and may not be a path"
        )
    return Path(map_root).expanduser().resolve() / name


def occupancy_map_is_valid(map_yaml):
    path = Path(map_yaml).expanduser().resolve()
    if not path.is_file():
        return False
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return False
    image = document.get("image")
    if not isinstance(image, str) or not image.strip():
        return False
    image_path = Path(image).expanduser()
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    return image_path.is_file()


def latest_valid_map_bundle(map_root):
    root = Path(map_root).expanduser().resolve()
    candidates = [
        path for path in root.rglob("occupancy.yaml")
        if ".staging" not in path.parts
        and ".backup" not in path.parts
        and occupancy_map_is_valid(path)
        and bundle_is_complete(path.parent)
    ] if root.is_dir() else []
    if not candidates:
        return None
    newest = max(candidates, key=lambda path: path.stat().st_mtime)
    return newest.parent


def next_mapping_bundle(map_root):
    root = Path(map_root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    numbered = []
    for candidate in root.iterdir():
        if not candidate.is_dir():
            continue
        match = TRACK_DIRECTORY_PATTERN.fullmatch(candidate.name)
        if match:
            numbered.append((int(match.group(1)), candidate))
    for _number, candidate in sorted(numbered):
        try:
            if not any(candidate.iterdir()):
                return candidate
        except OSError:
            continue
    next_number = max((number for number, _path in numbered), default=0) + 1
    return root / f"track_v{next_number}"
