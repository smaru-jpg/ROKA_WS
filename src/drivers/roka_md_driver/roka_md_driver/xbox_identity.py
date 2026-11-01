"""Resolve the ROKA Xbox without relying on js0 allocation order."""

from ctypes import CDLL, c_char_p, c_int, c_uint16, c_uint32, c_void_p
from ctypes.util import find_library
from dataclasses import dataclass
from pathlib import Path
import sys


ROKA_XBOX_MAC = "9c:aa:1b:72:51:9d"
ROKA_XBOX_USB_SERIAL = "30324E4530303834353334393035"
ROKA_XBOX_NAME = "Xbox Wireless Controller"
SDL_INIT_JOYSTICK = 0x00000200


@dataclass(frozen=True)
class JoystickIdentity:
    node: str
    name: str
    unique_id: str
    usb_serial: str


@dataclass(frozen=True)
class SdlJoystickIdentity:
    device_id: int
    name: str
    serial: str
    vendor: int = 0
    product: int = 0
    version: int = 0


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def _read_parent_attribute(device: Path, attribute: str) -> str:
    """Read a USB attribute from the input device or one of its parents."""
    current = device.resolve()
    for candidate in (current, *current.parents):
        value = _read_text(candidate / attribute)
        if value:
            return value
    return ""


def _identity_key(value: str) -> str:
    """Normalize MAC separators and serial-number letter case."""
    return "".join(
        character.lower() for character in value if character.isalnum()
    )


def _decode(value) -> str:
    if not value:
        return ""
    return value.decode("utf-8", errors="replace")


def enumerate_joysticks(sysfs_root: Path = Path("/sys/class/input")):
    """Return the kernel joystick devices currently available to ROS joy."""
    joysticks = []
    for entry in sorted(sysfs_root.glob("js*"), key=lambda item: item.name):
        device = entry / "device"
        joysticks.append(JoystickIdentity(
            node=f"/dev/input/{entry.name}",
            name=_read_text(device / "name"),
            unique_id=_read_text(device / "uniq").lower(),
            usb_serial=_read_parent_attribute(device, "serial").upper(),
        ))
    return joysticks


def enumerate_sdl_joysticks():
    """Return SDL indices together with the physical controller serial/MAC."""
    library_name = find_library("SDL2-2.0") or "libSDL2-2.0.so.0"
    sdl = CDLL(library_name)
    sdl.SDL_InitSubSystem.argtypes = [c_uint32]
    sdl.SDL_InitSubSystem.restype = c_int
    sdl.SDL_QuitSubSystem.argtypes = [c_uint32]
    sdl.SDL_NumJoysticks.restype = c_int
    sdl.SDL_JoystickNameForIndex.argtypes = [c_int]
    sdl.SDL_JoystickNameForIndex.restype = c_char_p
    sdl.SDL_JoystickOpen.argtypes = [c_int]
    sdl.SDL_JoystickOpen.restype = c_void_p
    sdl.SDL_JoystickClose.argtypes = [c_void_p]
    sdl.SDL_JoystickGetSerial.argtypes = [c_void_p]
    sdl.SDL_JoystickGetSerial.restype = c_char_p
    for function_name in (
        "SDL_JoystickGetVendor",
        "SDL_JoystickGetProduct",
        "SDL_JoystickGetProductVersion",
    ):
        function = getattr(sdl, function_name)
        function.argtypes = [c_void_p]
        function.restype = c_uint16
    sdl.SDL_GetError.restype = c_char_p

    if sdl.SDL_InitSubSystem(SDL_INIT_JOYSTICK) != 0:
        raise RuntimeError(
            "SDL joystick initialization failed: "
            f"{_decode(sdl.SDL_GetError())}"
        )
    devices = []
    try:
        count = sdl.SDL_NumJoysticks()
        if count < 0:
            raise RuntimeError(
                "SDL joystick enumeration failed: "
                f"{_decode(sdl.SDL_GetError())}"
            )
        for device_id in range(count):
            name = _decode(sdl.SDL_JoystickNameForIndex(device_id))
            joystick = sdl.SDL_JoystickOpen(device_id)
            if not joystick:
                devices.append(SdlJoystickIdentity(device_id, name, ""))
                continue
            try:
                devices.append(SdlJoystickIdentity(
                    device_id=device_id,
                    name=name,
                    serial=_decode(sdl.SDL_JoystickGetSerial(joystick)),
                    vendor=int(sdl.SDL_JoystickGetVendor(joystick)),
                    product=int(sdl.SDL_JoystickGetProduct(joystick)),
                    version=int(sdl.SDL_JoystickGetProductVersion(joystick)),
                ))
            finally:
                sdl.SDL_JoystickClose(joystick)
    finally:
        sdl.SDL_QuitSubSystem(SDL_INIT_JOYSTICK)
    return devices


def _kernel_inventory(joysticks) -> str:
    return ", ".join(
        f"{item.node} name={item.name!r} uniq={item.unique_id or '<none>'} "
        f"usb_serial={item.usb_serial or '<none>'}"
        for item in joysticks
    ) or "none"


def _sdl_inventory(devices) -> str:
    return ", ".join(
        f"id={item.device_id} name={item.name!r} "
        f"serial={item.serial or '<none>'} "
        f"vid:pid={item.vendor:04x}:{item.product:04x}"
        for item in devices
    ) or "none"


def resolve_assigned_sdl_device_id(
    requested_device_id,
    expected_mac=ROKA_XBOX_MAC,
    expected_usb_serial=ROKA_XBOX_USB_SERIAL,
    sysfs_root=Path("/sys/class/input"),
    sdl_devices=None,
):
    """Resolve the assigned controller's current SDL index by MAC/serial.

    ``js0`` and SDL index 0 are allocation-order values, not identities. The
    assigned controller must exist as a kernel joystick and match an opened
    SDL joystick by Bluetooth MAC or USB serial. A single-device fallback is
    allowed only when both layers expose exactly one joystick.
    """
    expected_keys = {
        key for key in (
            _identity_key(str(expected_mac)),
            _identity_key(str(expected_usb_serial)),
        ) if key
    }
    joysticks = enumerate_joysticks(Path(sysfs_root))
    kernel_matches = [
        item for item in joysticks
        if _identity_key(item.unique_id) in expected_keys
        or _identity_key(item.usb_serial) in expected_keys
    ]
    if len(kernel_matches) != 1:
        raise RuntimeError(
            f"ROKA Xbox (BT {expected_mac}, USB {expected_usb_serial}) is not "
            "exactly one input joystick; detected: "
            f"{_kernel_inventory(joysticks)}. Bluetooth Connected alone is "
            "insufficient; /dev/input/js* must exist."
        )

    devices = list(
        enumerate_sdl_joysticks() if sdl_devices is None else sdl_devices
    )
    sdl_matches = [
        item for item in devices if _identity_key(item.serial) in expected_keys
    ]
    if not sdl_matches and len(joysticks) == 1 and len(devices) == 1:
        sdl_matches = devices
    if len(sdl_matches) != 1:
        raise RuntimeError(
            "ROKA Xbox cannot be mapped to one SDL device by MAC/serial; "
            f"SDL detected: {_sdl_inventory(devices)}"
        )

    selected = sdl_matches[0]
    requested = str(requested_device_id).strip().lower()
    if requested != "auto":
        try:
            requested_index = int(requested)
        except ValueError as error:
            raise RuntimeError(
                f"joy_device_id={requested_device_id!r} must be auto or an "
                "integer"
            ) from error
        if requested_index != selected.device_id:
            raise RuntimeError(
                f"joy_device_id={requested_index} is not the assigned ROKA "
                "Xbox; "
                f"current assigned index is {selected.device_id}"
            )
    return selected.device_id


def resolve_exclusive_sdl_device_id(*args, **kwargs):
    """Backward-compatible name for older launch files and external tools."""
    return resolve_assigned_sdl_device_id(*args, **kwargs)


def main():
    try:
        device_id = resolve_assigned_sdl_device_id("auto")
    except RuntimeError as error:
        print(f"BLOCKED: {error}")
        return 1
    controller = enumerate_joysticks()[0]
    identity = controller.unique_id or controller.usb_serial
    print(
        f"OK: ROKA Xbox {identity} -> SDL device_id={device_id} "
        f"({controller.node}, {controller.name})"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
