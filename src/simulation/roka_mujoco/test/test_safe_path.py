from pathlib import Path

import yaml

from roka_mujoco.ground_truth_map import export_ground_truth
from roka_mujoco.safe_path import generate_safe_path


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "description" / "pk3a" / "models" / "pk3a_closed_loop.xml"
TRACK_METADATA = ROOT / "description" / "pk3a" / "assets" / "pk3a_closed_loop_metadata.json"


def test_safe_path_is_closed_noncentral_and_wheel_clear(tmp_path):
    map_result = export_ground_truth(SCENE, tmp_path / "map", 0.05, 0.10)
    output = tmp_path / "safe_path.yaml"
    result = generate_safe_path(
        Path(map_result["map_file"]),
        Path(map_result["metadata_file"]),
        TRACK_METADATA,
        output,
        sample_spacing_m=0.12,
        lateral_step_m=0.025,
    )
    document = yaml.safe_load(output.read_text())
    assert document["closed_loop"]
    assert result["point_count"] > 400
    assert result["noncentral_point_count"] > 0
    assert result["minimum_wheel_edge_m"] >= 0.084
    assert 0.05 <= result["speed_range_mps"][0] <= result["speed_range_mps"][1] <= 0.12
    assert all("cross_slope_deg" in point for point in document["points"])
