from pathlib import Path

import mujoco
import trimesh

from roka_mujoco.closed_loop_pipeline import generate


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "pk3a_closed_loop.json"


def test_closed_loop_uses_independent_watertight_module_solids(tmp_path):
    result = generate(CONFIG, tmp_path)
    assert result["geometry_pipeline"] == (
        "modular_watertight_visual_obj_with_hidden_coarse_hfield"
    )
    assert result["expanded_visual_face_count"] <= 500_000
    assert len(result["module_solids"]) == 12

    model = mujoco.MjModel.from_xml_path(result["model"])
    assert model.nbody == 65  # world plus 64 independently placed module bodies
    assert model.nmesh == 12
    assert model.nhfield == 12

    visual_ids = [
        model.geom(index).id
        for index in range(model.ngeom)
        if model.geom(index).name.startswith("visual_")
    ]
    collision_ids = [
        model.geom(index).id
        for index in range(model.ngeom)
        if model.geom(index).name.startswith("track_")
    ]
    assert len(visual_ids) == 64
    assert len(collision_ids) == 64
    assert all(model.geom_contype[index] == 0 for index in visual_ids)
    assert all(model.geom_conaffinity[index] == 0 for index in visual_ids)
    assert all(model.geom_group[index] == 4 for index in collision_ids)
    assert all(model.geom_rgba[index, 3] == 0.0 for index in collision_ids)

    for name, metrics in result["module_solids"].items():
        assert metrics["watertight"]
        assert metrics["winding_consistent"]
        assert metrics["volume_m3"] > 0.0
        mesh = trimesh.load_mesh(
            tmp_path / "assets" / "visual" / "modules" / f"{name}.obj",
            process=True,
        )
        assert mesh.is_watertight
        assert mesh.is_winding_consistent
        standalone = mujoco.MjModel.from_xml_path(
            str(tmp_path / "models" / "modules" / f"{name}.xml")
        )
        assert standalone.nmesh == 1
        assert standalone.nhfield == 1


def test_curved_collision_fields_lower_only_far_outside_samples(tmp_path):
    result = generate(CONFIG, tmp_path)
    curved = [
        value
        for name, value in result["asset_rasters"].items()
        if name.startswith("3CV1")
    ]
    assert curved
    for metrics in curved:
        assert metrics["outside_sample_count"] > 0
        assert 0 < metrics["collision_halo_sample_count"] < metrics[
            "outside_sample_count"
        ]
        assert metrics["minimum_height_m"] == -0.032
