from pathlib import Path

import pytest

from roka_md_driver.xbox_identity import (
    ROKA_XBOX_MAC,
    ROKA_XBOX_USB_SERIAL,
    SdlJoystickIdentity,
    enumerate_joysticks,
    resolve_exclusive_sdl_device_id,
)


def add_joystick(
    root: Path,
    node: str,
    name: str,
    unique_id: str = "",
    usb_serial: str = "",
):
    device = root / node / "device"
    device.mkdir(parents=True)
    (device / "name").write_text(name, encoding="utf-8")
    (device / "uniq").write_text(unique_id, encoding="utf-8")
    if usb_serial:
        (device / "serial").write_text(usb_serial, encoding="utf-8")


def sdl(device_id, serial):
    return SdlJoystickIdentity(
        device_id=device_id,
        name="Xbox Wireless Controller",
        serial=serial,
        vendor=0x045E,
        product=0x028E,
    )


def test_current_roka_controller_resolves_by_mac(tmp_path):
    add_joystick(
        tmp_path, "js3", "Xbox Wireless Controller", ROKA_XBOX_MAC.upper()
    )
    assert resolve_exclusive_sdl_device_id(
        "auto",
        sysfs_root=tmp_path,
        sdl_devices=[sdl(2, ROKA_XBOX_MAC.upper())],
    ) == 2
    assert enumerate_joysticks(tmp_path)[0].unique_id == ROKA_XBOX_MAC


def test_current_roka_usb_controller_resolves_to_only_sdl_index(tmp_path):
    add_joystick(
        tmp_path,
        "js0",
        "Microsoft X-Box One S pad",
        usb_serial=ROKA_XBOX_USB_SERIAL.lower(),
    )
    assert resolve_exclusive_sdl_device_id(
        "auto",
        sysfs_root=tmp_path,
        sdl_devices=[sdl(1, ROKA_XBOX_USB_SERIAL.lower())],
    ) == 1
    assert enumerate_joysticks(tmp_path)[0].usb_serial == ROKA_XBOX_USB_SERIAL


def test_missing_current_controller_is_fail_closed(tmp_path):
    with pytest.raises(
        RuntimeError, match="is not exactly one input joystick"
    ):
        resolve_exclusive_sdl_device_id("auto", sysfs_root=tmp_path)


def test_old_controller_cannot_take_device_zero(tmp_path):
    add_joystick(
        tmp_path, "js0", "Xbox Wireless Controller", "9c:aa:1b:6f:28:6b"
    )
    with pytest.raises(
        RuntimeError, match="is not exactly one input joystick"
    ):
        resolve_exclusive_sdl_device_id("auto", sysfs_root=tmp_path)


def test_other_usb_controller_cannot_take_device_zero(tmp_path):
    add_joystick(
        tmp_path,
        "js0",
        "Microsoft X-Box One S pad",
        usb_serial="DIFFERENT_CONTROLLER_SERIAL",
    )
    with pytest.raises(
        RuntimeError, match="is not exactly one input joystick"
    ):
        resolve_exclusive_sdl_device_id("auto", sysfs_root=tmp_path)


def test_two_controllers_select_roka_by_mac(tmp_path):
    add_joystick(tmp_path, "js0", "Xbox Wireless Controller", ROKA_XBOX_MAC)
    add_joystick(
        tmp_path, "js1", "Xbox Wireless Controller", "9c:aa:1b:6f:28:6b"
    )
    assert resolve_exclusive_sdl_device_id(
        "auto",
        sysfs_root=tmp_path,
        sdl_devices=[
            sdl(0, "9c:aa:1b:6f:28:6b"),
            sdl(1, ROKA_XBOX_MAC),
        ],
    ) == 1


def test_manual_nonzero_index_is_rejected(tmp_path):
    add_joystick(tmp_path, "js0", "Xbox Wireless Controller", ROKA_XBOX_MAC)
    with pytest.raises(RuntimeError, match="is not the assigned ROKA Xbox"):
        resolve_exclusive_sdl_device_id(
            "1", sysfs_root=tmp_path, sdl_devices=[sdl(0, ROKA_XBOX_MAC)]
        )


def test_two_controllers_without_sdl_serial_are_fail_closed(tmp_path):
    add_joystick(tmp_path, "js0", "Xbox Wireless Controller", ROKA_XBOX_MAC)
    add_joystick(
        tmp_path, "js1", "Xbox Wireless Controller", "9c:aa:1b:6f:28:6b"
    )
    with pytest.raises(RuntimeError, match="cannot be mapped"):
        resolve_exclusive_sdl_device_id(
            "auto",
            sysfs_root=tmp_path,
            sdl_devices=[sdl(0, ""), sdl(1, "")],
        )
