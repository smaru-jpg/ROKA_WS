import pytest

from roka_fast_lio_localization.bag_sensor_audit import uint8_scalar


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        (0, 0),
        (2, 2),
        (b"\x00", 0),
        (b"\x02", 2),
        (bytearray(b"\xff"), 255),
        (memoryview(b"\x01"), 1),
    ],
)
def test_uint8_scalar_accepts_ros_integer_and_humble_byte_forms(value, expected):
    assert uint8_scalar(value, field_name="DiagnosticStatus.level") == expected


@pytest.mark.parametrize("value", [b"", b"\x00\x01", -1, 256])
def test_uint8_scalar_rejects_malformed_values(value):
    with pytest.raises(ValueError, match="DiagnosticStatus.level"):
        uint8_scalar(value, field_name="DiagnosticStatus.level")
