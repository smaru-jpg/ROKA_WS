from pathlib import Path

import numpy as np

from roka_mujoco.ground_truth_map import export_ground_truth, sample_ground_truth


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "description" / "pk3a" / "models" / "pk3a_closed_loop.xml"


def test_ground_truth_uses_only_track_collision_heightfields():
    layers, metadata = sample_ground_truth(SCENE, resolution_m=0.10, margin_m=0.10)

    assert metadata["track_geom_count"] == 64
    assert metadata["valid_cell_count"] > 6_000
    assert 0.0 < metadata["track_area_m2"] < 100.0
    assert metadata["minimum_elevation_m"] > -0.04
    assert metadata["maximum_elevation_m"] > 0.8
    assert metadata["maximum_slope_deg"] > 50.0

    valid = layers["rideable"]
    normal_norm = np.sqrt(
        layers["normal_x"][valid] ** 2
        + layers["normal_y"][valid] ** 2
        + layers["normal_z"][valid] ** 2
    )
    assert np.allclose(normal_norm, 1.0, atol=1e-4)
    assert np.all(layers["geom_id"][~layers["collision_valid"]] == -1)
    assert metadata["excluded_hfield_apron_cell_count"] > 0
    assert np.nanmax(layers["edge_distance_m"]) > 0.40


def test_ground_truth_artifact_round_trip(tmp_path):
    result = export_ground_truth(
        SCENE,
        tmp_path,
        resolution_m=0.10,
        margin_m=0.10,
    )
    archive = np.load(result["map_file"])
    assert set(archive.files) == {
        "elevation",
        "normal_x",
        "normal_y",
        "normal_z",
        "slope_deg",
        "step_height_m",
        "edge_distance_m",
        "collision_valid",
        "rideable",
        "valid",
        "geom_id",
        "module_index",
    }
    assert archive["elevation"].shape == tuple(
        reversed(result["size_cells"])
    )
