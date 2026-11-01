import cv2
import numpy as np

from mission_vision.filters.hsv_ratio_filter import HSVRatioFilter
from mission_vision.vision_node import format_detection_label


def bgr_frame(hue, saturation=160, value=120):
    hsv = np.full((20, 20, 3), (hue, saturation, value), dtype=np.uint8)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def detection(class_name="enemy", bbox=(0, 0, 20, 20)):
    return {
        "class_name": class_name,
        "confidence": 0.91,
        "bbox": bbox,
    }


def test_target_detection_records_ratio_and_passes():
    target = detection()
    color_filter = HSVRatioFilter(
        target_classes=["enemy"],
        h_min=23,
        h_max=40,
        s_min=70,
        s_max=255,
        v_min=20,
        v_max=150,
        min_ratio=0.20,
        reject_below=True,
        result_name="enemy_green",
    )
    result = color_filter.process([target], {"frame": bgr_frame(30)})
    assert result == [target]
    assert target["hsv_results"]["enemy_green"] == 1.0
    assert format_detection_label(target, True).endswith("enemy_green:1.00 PASS")


def test_target_detection_below_ratio_is_rejected_but_keeps_debug_value():
    target = detection()
    color_filter = HSVRatioFilter(
        target_classes=["enemy"],
        h_min=23,
        h_max=40,
        min_ratio=0.20,
        reject_below=True,
        result_name="enemy_green",
    )
    assert color_filter.process([target], {"frame": bgr_frame(100)}) == []
    assert target["hsv_results"]["enemy_green"] == 0.0
    assert format_detection_label(target, False).endswith("enemy_green:0.00 REJECT")


def test_non_target_class_is_untouched_and_red_hue_wrap_is_supported():
    friend = detection("friend")
    enemy_filter = HSVRatioFilter(target_classes=["enemy"], reject_below=True)
    assert enemy_filter.process([friend], {"frame": bgr_frame(30)}) == [friend]
    assert "hsv_results" not in friend

    red = detection("red")
    red_filter = HSVRatioFilter(
        target_classes=["red"],
        h_min=170,
        h_max=10,
        s_min=50,
        v_min=50,
        result_name="red_hsv",
    )
    assert red_filter.process([red], {"frame": bgr_frame(179)}) == [red]
    assert red["hsv_results"]["red_hsv"] == 1.0
