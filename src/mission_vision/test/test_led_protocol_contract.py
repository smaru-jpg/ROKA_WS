from pathlib import Path

import yaml

from mission_vision.neopixel_controller import COMMAND_BYTES, normalize_led_state


PACKAGE = Path(__file__).resolve().parents[1]
FIRMWARE = (
    PACKAGE / "firmware" / "roka_iff_neopixel" / "roka_iff_neopixel.ino"
)


def test_ros_and_nano_use_the_same_single_byte_protocol():
    assert COMMAND_BYTES == {
        "ENEMY": b"E",
        "FRIEND": b"F",
        "BOTH": b"B",
        "NONE": b"N",
    }
    source = FIRMWARE.read_text(encoding="utf-8")
    for command in ("E", "F", "B", "N"):
        assert f"state == '{command}'" in source


def test_friend_and_enemy_detection_reaches_the_nano_both_command():
    assert normalize_led_state("ENEMY,FRIEND") == "BOTH"
    assert normalize_led_state("friend,enemy") == "BOTH"
    assert normalize_led_state("ENEMY") == "ENEMY"
    assert normalize_led_state("FRIEND") == "FRIEND"
    assert normalize_led_state("NONE") == "NONE"
    assert normalize_led_state("unknown") is None

    spring = yaml.safe_load(
        (PACKAGE / "config" / "spring.yaml").read_text(encoding="utf-8")
    )
    assert spring["inference"]["detection_mode"] == "multiple"


def test_validated_nano_pin_pixel_count_and_baud_are_preserved():
    source = FIRMWARE.read_text(encoding="utf-8")
    assert "#define PIN 6" in source
    assert "#define NUMPIXELS 16" in source
    assert "Serial.begin(115200)" in source
    assert "NEO_GRB + NEO_KHZ800" in source

    spring = yaml.safe_load(
        (PACKAGE / "config" / "spring.yaml").read_text(encoding="utf-8")
    )
    assert spring["led"]["baud_rate"] == 115200
    assert spring["led"]["startup_delay_sec"] == 2.0
    # A portable handoff must not force a serial device belonging to another PC.
    assert spring["led"]["enabled"] is False
    assert spring["led"]["port"] == ""


def test_roka_exposes_a_drive_free_camera_and_led_test_command():
    launcher = (PACKAGE.parents[1] / "roka").read_text(encoding="utf-8")
    assert "roka vision spring-led" in launcher
    assert '"enable_led:=true"' in launcher
    assert '"led_serial_port:=${ROKA_LED_SERIAL}"' in launcher
