"""Non-moving software and device identity check for the ROKA arm."""

import argparse
from hashlib import sha256
import importlib
from pathlib import Path
import shutil
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory

from . import arm_config as cfg


MD_SERIALS = ("A76BHBER", "A751CYHM")
MD_PATHS = tuple(
    Path("/dev/serial/by-id")
    / f"usb-FTDI_USB_to_RS-422_485_Adapter_{serial}-if00-port0"
    for serial in MD_SERIALS
)


def report(ok, label, detail):
    state = "PASS" if ok else "FAIL"
    print(f"[ARM-CHECK][{state}] {label}: {detail}")
    return ok


def import_check():
    ok = True
    for module in (
        "rclpy",
        "cv2",
        "numpy",
        "torch",
        "ultralytics",
        "cv_bridge",
        "serial",
        "dynamixel_sdk",
    ):
        try:
            imported = importlib.import_module(module)
            version = getattr(imported, "__version__", "available")
            ok = report(True, f"python {module}", str(version)) and ok
        except Exception as error:
            ok = report(False, f"python {module}", str(error)) and ok
    return ok


def model_check(model_path):
    if not model_path.is_file():
        return report(False, "YOLO model", f"missing {model_path}")
    digest = sha256(model_path.read_bytes()).hexdigest()
    if digest != cfg.YOLO_MODEL_SHA256:
        return report(False, "YOLO model", f"checksum mismatch {digest}")
    try:
        from ultralytics import YOLO

        model = YOLO(str(model_path))
        names = dict(model.names)
    except Exception as error:
        return report(False, "YOLO model", f"load failed: {error}")
    return report(True, "YOLO model", f"sha256={digest}; classes={names}")


def serial_candidates(assigned_paths=()):
    root = Path("/dev/serial/by-id")
    if not root.is_dir():
        return []
    assigned_targets = {
        target
        for target in (resolved_device(path) for path in assigned_paths)
        if target is not None
    }
    return sorted(
        path
        for path in root.iterdir()
        if "ftdi" in path.name.lower()
        and not any(serial in path.name for serial in MD_SERIALS)
        and resolved_device(path) not in assigned_targets
    )


def resolved_device(path):
    """Return a canonical existing device target, or None."""
    try:
        resolved = path.resolve(strict=True)
    except (FileNotFoundError, OSError, RuntimeError):
        return None
    return resolved


def assignment_check(arm_paths, reserved_paths=MD_PATHS):
    """Reject duplicate U2D2 assignments and reuse of a base MD adapter."""
    resolved_arm = [resolved_device(path) for path in arm_paths]
    if any(target is None for target in resolved_arm):
        return False, "one or more fixed arm device names are unresolved"
    if len(set(resolved_arm)) != len(resolved_arm):
        return False, "arm RS-485 and wrist TTL resolve to the same device"

    reserved = {
        target
        for target in (resolved_device(path) for path in reserved_paths)
        if target is not None
    }
    collisions = reserved.intersection(resolved_arm)
    if collisions:
        names = ", ".join(str(path) for path in sorted(collisions))
        return False, f"arm assignment reuses an MD200T adapter: {names}"
    return True, ", ".join(str(path) for path in resolved_arm)


def dynamixel_bus_check():
    """Read all actuators and confirm that a safe start-pose recovery is possible."""
    from .dynamixel_driver import DynamixelDriver

    driver = DynamixelDriver()
    try:
        ready = driver.initialize(
            enable_motion=False,
            allow_initial_recovery=True,
        )
    finally:
        driver.close_ports()
    return report(
        ready,
        "Dynamixel buses",
        "IDs 11/12/13 and 1/4/6 readable; initial section-pose recovery supported; torque/motion OFF"
        if ready
        else "one or more configured Dynamixel IDs are unreadable",
    )


def hardware_check():
    ok = True
    arm_paths = (
        Path(cfg.DEVICENAME_ARM),
        Path(cfg.DEVICENAME_WRIST),
    )
    for label, path in zip(("arm RS-485", "wrist TTL"), arm_paths):
        target = resolved_device(path)
        valid = target is not None
        detail = str(target) if valid else f"missing {path}"
        ok = report(valid, label, detail) and ok

    assignments_ok, assignment_detail = assignment_check(arm_paths)
    ok = report(
        assignments_ok,
        "serial isolation",
        assignment_detail,
    ) and ok

    if assignments_ok:
        ok = dynamixel_bus_check() and ok

    candidates = serial_candidates(arm_paths)
    report(
        True,
        "unassigned FTDI candidates (information only)",
        f"count={len(candidates)}; "
        + (", ".join(path.name for path in candidates) or "none"),
    )

    command = shutil.which("rs-enumerate-devices")
    if command is None:
        ok = report(False, "D435", "rs-enumerate-devices missing") and ok
    else:
        result = subprocess.run(
            [command, "-s"],
            text=True,
            capture_output=True,
            timeout=10,
            check=False,
        )
        found = "D435" in result.stdout
        detail = "detected" if found else "not connected"
        ok = report(found, "D435", detail) and ok
    return ok


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--require-hardware", action="store_true")
    parser.add_argument("--model", type=Path)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    package_share = Path(get_package_share_directory("roka_arm"))
    model = args.model or package_share / "models" / "box_marker_best.pt"

    imports_ok = import_check()
    model_ok = model_check(model)
    software_ok = imports_ok and model_ok
    hardware_ok = hardware_check() if args.require_hardware else True
    ready = software_ok and hardware_ok
    result = "READY" if ready else "NOT READY"
    suffix = "software only" if not args.require_hardware else "software+hardware"
    print(f"[ARM-CHECK][RESULT] {result} ({suffix}); no motor command was sent")
    return 0 if ready else 1


if __name__ == "__main__":
    sys.exit(main())
