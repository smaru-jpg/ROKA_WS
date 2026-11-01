from types import SimpleNamespace

import numpy as np
import pytest
from sensor_msgs.msg import Image
from types import SimpleNamespace

from mission_vision.cameras.ros_image_camera import RosImageCamera
from mission_vision.display import OpenCvDisplay
from mission_vision.vision_node import VisionNode


class Logger:
    def __init__(self):
        self.warnings = []
        self.errors = []

    def warning(self, message):
        self.warnings.append(message)

    def error(self, message):
        self.errors.append(message)


class Node:
    def __init__(self):
        self.logger = Logger()

    def get_logger(self):
        return self.logger

    def create_subscription(self, _type, _topic, callback, _qos):
        self.callback = callback
        return SimpleNamespace()


def test_headless_display_disables_itself_without_calling_highgui(monkeypatch):
    monkeypatch.delenv("DISPLAY", raising=False)
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    logger = Logger()
    display = OpenCvDisplay(True, "test", logger)
    assert not display.enabled
    assert display.show(np.zeros((8, 8, 3), dtype=np.uint8)) == -1
    assert len(logger.warnings) == 1


def test_camera_keeps_only_new_stamp_and_fails_soft_on_bad_conversion():
    node = Node()
    camera = RosImageCamera({"topic": "/test/image"}, node)
    message = Image()
    message.header.stamp.sec = 1
    message.header.stamp.nanosec = 2
    camera._receive(message)

    class BrokenBridge:
        def imgmsg_to_cv2(self, *_args, **_kwargs):
            raise RuntimeError("bad image")

    camera.bridge = BrokenBridge()
    assert camera.read() is None
    assert node.logger.errors == ["camera image conversion failed: bad image"]
    # The same malformed message is not converted and logged repeatedly.
    assert camera.read() is None
    assert len(node.logger.errors) == 1
    assert camera.age_seconds() is not None
    assert camera.age_seconds() < 1.0


def test_profile_inference_can_be_enabled_after_mission_handoff():
    node = object.__new__(VisionNode)
    node.mission_name = "traffic"
    node.enabled = False
    node.get_logger = lambda: SimpleNamespace(info=lambda _message: None)
    response = SimpleNamespace(success=False, message="")

    result = node._set_enabled(SimpleNamespace(data=True), response)

    assert node.enabled
    assert result.success
    assert result.message == "traffic inference enabled"


@pytest.mark.parametrize(
    ("profile", "expected"),
    [
        ("spring", {0: "enemy", 1: "friend"}),
        ("summer", {0: "box"}),
        (
            "autumn",
            {0: "A", 1: "E", 2: "Heart", 3: "K", 4: "M", 5: "O", 6: "R", 7: "Y"},
        ),
        ("traffic", {0: "blue", 1: "red"}),
    ],
)
def test_checkpoint_embedded_classes_match_selected_contract(profile, expected):
    yaml = pytest.importorskip("yaml")
    ultralytics = pytest.importorskip("ultralytics")
    from pathlib import Path

    package = Path(__file__).resolve().parents[1]
    metadata = yaml.safe_load(
        (package / "models" / "model_metadata.yaml").read_text(encoding="utf-8")
    )
    model = ultralytics.YOLO(str(package / "models" / metadata["models"][profile]["file"]))
    assert dict(model.names) == expected


def test_runtime_versions_match_the_recorded_model_environment():
    import cv2
    import torch
    import ultralytics
    import yaml
    from pathlib import Path

    package = Path(__file__).resolve().parents[1]
    metadata = yaml.safe_load(
        (package / "models" / "model_metadata.yaml").read_text(encoding="utf-8")
    )
    assert torch.__version__.split("+")[0] == metadata["runtime"]["torch"].split("+")[0]
    assert ultralytics.__version__ == str(metadata["runtime"]["ultralytics"])
    assert int(cv2.__version__.split(".")[0]) == 4
