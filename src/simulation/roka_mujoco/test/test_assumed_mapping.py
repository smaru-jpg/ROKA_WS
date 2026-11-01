from pathlib import Path

import yaml

from roka_mujoco.assumed_mapping import build_assumed_mapping


ROOT = Path(__file__).resolve().parents[4]


def test_assumed_mapping_uses_successful_manual_lap_and_fails_closed(tmp_path):
    ground = ROOT / "maps" / "sim_pk3a_ground_truth"
    demos = ROOT / "datasets" / "xbox_demonstrations"
    result = build_assumed_mapping(
        ground / "pk3a_ground_truth.npz",
        ground / "pk3a_ground_truth.json",
        demos / "lap01_2.csv",
        [demos / "lap01.csv", demos / "fast_test01.csv"],
        ground / "pk3a_risk_aware_profile_v2.yaml",
        tmp_path,
        build_terrain=False,
    )
    assert result["status"] == "SIMULATED_MAPPING_READY_FOR_RVIZ"
    assert result["field_navigation_authorized"] is False
    assert result["route"]["points"] > 500
    assert result["route"]["length_m"] > 60.0
    assert result["route"]["closure_distance_m"] < 0.30
    assert result["virtual_lidar"]["coverage_fraction"] > 0.70
    for name in (
        "map.pcd", "map_dense.pcd", "surface.pcd", "occupancy.yaml",
        "track_profile.yaml", "pose_graph.g2o", "surface_extraction.yaml",
    ):
        assert (tmp_path / name).is_file()
    profile = yaml.safe_load((tmp_path / "track_profile.yaml").read_text())
    assert profile["path_source"] == "lap01_2.csv"
    assert profile["path_role"] == "manual_xbox_successful_lap"
    assert profile["field_navigation_authorized"] is False
