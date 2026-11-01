from hashlib import sha256
from pathlib import Path

import yaml


def test_selected_model_files_match_declared_contracts():
    package = Path(__file__).resolve().parents[1]
    metadata = yaml.safe_load(
        (package / "models" / "model_metadata.yaml").read_text(encoding="utf-8")
    )
    expected_files = {
        "IFF_0830.pt",
        "box_0830.pt",
        "vision_0830.pt",  # one-version rollback until D435 field approval
        "vision_0831.pt",
        "traffic_0831.pt",
    }
    actual_files = {path.name for path in (package / "models").glob("*.pt")}
    assert actual_files == expected_files
    for contract in metadata["models"].values():
        model = package / "models" / contract["file"]
        assert sha256(model.read_bytes()).hexdigest() == contract["sha256"]
    for contract in metadata.get("rollback_models", {}).values():
        model = package / "models" / contract["file"]
        assert sha256(model.read_bytes()).hexdigest() == contract["sha256"]

    for profile, contract in metadata["models"].items():
        config = yaml.safe_load(
            (package / "config" / f"{profile}.yaml").read_text(encoding="utf-8")
        )
        assert config["model"]["path"] == f"models/{contract['file']}"
        assert config["model"]["sha256"] == contract["sha256"]
        assert config["model"]["expected_classes"] == contract["classes"]


def test_camera_profiles_share_one_ros_topic():
    package = Path(__file__).resolve().parents[1]
    topics = set()
    for name in ("spring", "summer", "autumn", "traffic"):
        config = yaml.safe_load(
            (package / "config" / f"{name}.yaml").read_text(encoding="utf-8")
        )
        assert config["camera"]["type"] == "ros"
        assert config["camera"]["fps"] == 30
        topics.add(config["camera"]["topic"])
    assert topics == {"/roka/arm_camera/color/image_raw"}
