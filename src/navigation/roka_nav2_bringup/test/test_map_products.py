from pathlib import Path
import shutil

import pytest
import yaml

from roka_nav2_bringup.map_bundle_quality import occupancy_metrics
from roka_nav2_bringup.map_review import (
    active_reviewed_map,
    active_reviewed_occupancy,
    approve_map,
    approve_occupancy,
)
from roka_nav2_bringup.optimized_map_products import (
    derive_occupancy_from_optimized_map,
    derive_optimized_profile,
)
from roka_nav2_bringup.drivable_surface import extract_drivable_surface
from roka_nav2_bringup.pcd_terrain_profile import load_ascii_xyz


PACKAGE_ROOT = Path(__file__).parents[1]


def write_track_pcd(path: Path):
    points = []
    for index in range(400):
        x = 0.01 * index
        for y in (-0.60, -0.55, 0.55, 0.60):
            points.append((x, y, 0.0, 1.0))
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n"
        "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n"
    )
    body = "".join(f"{x} {y} {z} {intensity}\n" for x, y, z, intensity in points)
    path.write_text(header + body, encoding="ascii")


def write_optimized_csv(path: Path):
    rows = ["stamp,x,y,z,roll,pitch,yaw\n"]
    rows.extend(f"{index},{0.4 * index},0,0,0,0,0\n" for index in range(10))
    path.write_text("".join(rows), encoding="utf-8")


def test_drivable_surface_separates_floor_from_vertical_scene(tmp_path):
    profile = {
        "frame_id": "map",
        "track_width_m": 1.0,
        "points": [
            {"x": 0.2 * index, "y": 0.0, "z": 0.103, "yaw": 0.0}
            for index in range(21)
        ],
    }
    (tmp_path / "track_profile.yaml").write_text(
        yaml.safe_dump(profile), encoding="utf-8"
    )
    write_optimized_csv(tmp_path / "optimized_path.csv")
    # Rewrite the short helper route to cover the complete four metres.
    rows = ["stamp,x,y,z,roll,pitch,yaw\n"]
    rows.extend(
        f"{index},{0.4 * index},0,0.103,0,0,0\n" for index in range(11)
    )
    (tmp_path / "optimized_path.csv").write_text("".join(rows), encoding="utf-8")
    points = []
    for x_index in range(41):
        x = 0.1 * x_index
        for y_index in range(-5, 6):
            y = 0.1 * y_index
            points.append((x, y, 0.0))
        # A wall in the same XY corridor must not become a terrain ramp.
        for z_index in range(1, 16):
            points.append((x, 0.45, 0.1 * z_index))
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\n"
        "SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n"
    )
    (tmp_path / "map_dense.pcd").write_text(
        header + "".join(f"{x} {y} {z}\n" for x, y, z in points),
        encoding="ascii",
    )

    report = extract_drivable_surface(tmp_path)
    surface = load_ascii_xyz(tmp_path / "surface.pcd")

    assert report["result"] == "PASS"
    assert report["surface_cells"] >= 200
    assert float(surface[:, 2].max()) < 0.05


def test_sc_pgo_products_share_one_map_frame(tmp_path):
    raw = {
        "frame_id": "odom",
        "environment": "inline",
        "closed_loop": False,
        "track_width_m": 0.9144,
        "default_speed_mps": 0.20,
        "points": [{"x": 0.4 * index, "y": 0.0, "yaw": 0.0} for index in range(10)],
    }
    raw_path = tmp_path / "track_profile.yaml"
    raw_path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    csv_path = tmp_path / "optimized_path.csv"
    write_optimized_csv(csv_path)
    profile = derive_optimized_profile(raw_path, csv_path, raw_path)
    assert profile["frame_id"] == "map"
    assert profile["profile_role"] == "optimized_mapping_evidence"
    assert len(profile["points"]) >= 30

    pcd = tmp_path / "map.pcd"
    write_track_pcd(pcd)
    derive_occupancy_from_optimized_map(pcd, raw_path, tmp_path / "occupancy.yaml")
    metrics = occupancy_metrics(tmp_path / "occupancy.yaml")
    assert metrics["known_count"] > 500
    assert metrics["occupied_count"] > 30


def test_fragmentation_quality_ignores_only_components_up_to_15_cm(tmp_path):
    width = 10
    height = 8
    pixels = [254] * (width * height)
    # Three 5 cm cells span exactly 15 cm and are ignored only by the map
    # fragmentation quality count.  They remain occupied map cells.
    for column in range(1, 4):
        pixels[1 * width + column] = 0
    # Four cells span 20 cm and must remain a significant map component.
    for column in range(1, 5):
        pixels[5 * width + column] = 0
    (tmp_path / "occupancy.pgm").write_text(
        "P2\n"
        f"{width} {height}\n"
        "255\n"
        + " ".join(str(value) for value in pixels)
        + "\n",
        encoding="ascii",
    )
    (tmp_path / "occupancy.yaml").write_text(
        "image: occupancy.pgm\n"
        "resolution: 0.05\n"
        "origin: [0.0, 0.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n",
        encoding="utf-8",
    )

    metrics = occupancy_metrics(tmp_path / "occupancy.yaml")

    assert metrics["occupied_count"] == 7
    assert metrics["occupied_component_count"] == 2
    assert metrics["occupied_ignored_small_component_count"] == 1
    assert metrics["occupied_fragment_component_count"] == 1
    assert metrics["fragmentation_ignore_maximum_extent_m"] == pytest.approx(
        0.15
    )


def test_review_approval_selects_cleaned_map_and_detects_mutation(tmp_path):
    source = tmp_path / "map.pcd"
    write_track_pcd(source)
    cleaned = tmp_path / "map_cleaned.pcd"
    shutil.copyfile(source, cleaned)
    (tmp_path / "map_cleanup.yaml").write_text(
        yaml.safe_dump({
            "schema_version": 1,
            "source_map": str(source),
            "cleaned_map": str(cleaned),
            "source_preserved": True,
        }),
        encoding="utf-8",
    )
    assert approve_map(tmp_path, use_cleaned=True) == cleaned
    assert active_reviewed_map(tmp_path) == cleaned
    cleaned.write_bytes(cleaned.read_bytes() + b"\n")
    with pytest.raises(ValueError, match="changed after approval"):
        active_reviewed_map(tmp_path)


def test_occupancy_approval_selects_cleaned_map_and_detects_mutation(tmp_path):
    raw = {
        "frame_id": "map",
        "environment": "flat",
        "closed_loop": False,
        "track_width_m": 0.9144,
        "default_speed_mps": 0.20,
        "points": [{"x": 0.4 * index, "y": 0.0, "yaw": 0.0} for index in range(10)],
    }
    profile = tmp_path / "track_profile.yaml"
    profile.write_text(yaml.safe_dump(raw), encoding="utf-8")
    pcd = tmp_path / "map.pcd"
    write_track_pcd(pcd)
    source_yaml = tmp_path / "occupancy.yaml"
    derive_occupancy_from_optimized_map(pcd, profile, source_yaml)
    cleaned_yaml = tmp_path / "occupancy_cleaned.yaml"
    cleaned_pgm = tmp_path / "occupancy_cleaned.pgm"
    cleaned_document = yaml.safe_load(source_yaml.read_text(encoding="utf-8"))
    shutil.copyfile(tmp_path / cleaned_document["image"], cleaned_pgm)
    cleaned_document["image"] = cleaned_pgm.name
    cleaned_yaml.write_text(yaml.safe_dump(cleaned_document), encoding="utf-8")
    (tmp_path / "occupancy_cleanup.yaml").write_text(
        yaml.safe_dump({
            "schema_version": 1,
            "source_yaml": str(source_yaml),
            "cleaned_yaml": str(cleaned_yaml),
            "source_preserved": True,
            "changed_cells": 0,
        }),
        encoding="utf-8",
    )
    assert approve_occupancy(tmp_path, use_cleaned=True) == cleaned_yaml
    assert active_reviewed_occupancy(tmp_path) == cleaned_yaml
    cleaned_pgm.write_bytes(cleaned_pgm.read_bytes() + b"\n")
    with pytest.raises(ValueError, match="changed after approval"):
        active_reviewed_occupancy(tmp_path)


def test_occupancy_cleanup_rviz_opens_in_map_top_down_view():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "rviz" / "occupancy_cleanup.rviz").read_text(encoding="utf-8")
    )
    manager = config["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] == "map"
    assert manager["Views"]["Current"]["Class"] == "rviz_default_plugins/TopDownOrtho"
    assert manager["Views"]["Current"]["Target Frame"] == "map"

    displays = {display["Name"]: display for display in manager["Displays"]}
    original = displays["Original 2D Map (reference)"]["Topic"]
    working = displays["Working 2D Map (editable)"]["Topic"]
    assert original["Value"] == "/occupancy_cleanup/original"
    assert working["Value"] == "/occupancy_cleanup/working"
    assert original["Durability Policy"] == "Transient Local"
    assert working["Durability Policy"] == "Transient Local"
    assert displays["Original 2D Map (reference)"]["Enabled"] is False
    assert displays["Working 2D Map (editable)"]["Enabled"] is True
    overlays = {
        "Recorded Mapping Path": "/occupancy_cleanup/recorded_path",
        "Vehicle Left Edge": "/occupancy_cleanup/vehicle_left",
        "Vehicle Right Edge": "/occupancy_cleanup/vehicle_right",
    }
    for name, topic in overlays.items():
        assert displays[name]["Enabled"] is True
        assert displays[name]["Topic"]["Value"] == topic
        assert displays[name]["Topic"]["Durability Policy"] == "Transient Local"


@pytest.mark.parametrize(
    "filename,target_frame",
    [
        ("mapping_growth_2d.rviz", "base_footprint"),
        ("mapping_fastlio.rviz", "base_link"),
    ],
)
def test_mapping_rviz_uses_dense_indoor_lidar_view(filename, target_frame):
    config = yaml.safe_load(
        (PACKAGE_ROOT / "rviz" / filename).read_text(encoding="utf-8")
    )
    manager = config["Visualization Manager"]
    view = config["Views"]["Current"]
    assert view["Class"] == "rviz_default_plugins/Orbit"
    assert view["Target Frame"] == target_frame
    assert float(view["Distance"]) <= 10.0
    displays = {display["Name"]: display for display in manager["Displays"]}
    cloud = next(
        display
        for name, display in displays.items()
        if name.startswith("Dense Forward LiDAR") or name.startswith("Forward 120deg")
    )
    assert cloud["Enabled"] is True
    assert cloud["Topic"]["Value"] == "/mapping/front_120_points"
    assert cloud["Topic"]["Reliability Policy"] == "Best Effort"
    assert float(cloud["Decay Time"]) >= 2.0
    assert int(cloud["Size (Pixels)"]) >= 4
    assert cloud["Color Transformer"] == "AxisColor"
    assert cloud["Axis"] == "Z"


def test_autonomy_rviz_shows_the_actual_follow_path_and_progress():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "rviz" / "lidar_md_navigation.rviz").read_text(
            encoding="utf-8"
        )
    )
    manager = config["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] == "map"
    assert manager["Views"]["Current"]["Class"] == "rviz_default_plugins/TopDownOrtho"
    displays = {display["Name"]: display for display in manager["Displays"]}
    expected = {
        "Approved Mapped Path": "/track/centerline",
        "Traversed Approved Path": "/track/traversed_path",
        "Remaining Approved Path": "/track/remaining_path",
        "AMCL Robot Pose": "/track/localized_pose",
        "Reference Lookahead Target": "/track/current_target",
        "Track Progress": "/track/progress_marker",
    }
    for name, topic in expected.items():
        assert displays[name]["Enabled"] is True
        assert displays[name]["Topic"]["Value"] == topic
    assert displays["AMCL Robot Pose"]["Class"] == "rviz_default_plugins/Pose"


def test_recorded_review_separates_glim_style_evidence_and_terrain_layers():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "rviz" / "recorded_mapping_replay.rviz").read_text(
            encoding="utf-8"
        )
    )
    groups = {
        display["Name"]: display
        for display in config["Visualization Manager"]["Displays"]
        if display.get("Class") == "rviz_common/Group"
    }
    assert set(groups) >= {
        "01 SCANS - 현재 센서",
        "02 MAPS - 누적/최적화 지도",
        "03 TRAJECTORIES - 경로/그래프",
        "04 TERRAIN - 자동 판정",
        "05 REVIEW - 수동 다각형",
    }
    nested = {
        item["Name"]: item
        for group in groups.values()
        for item in group["Displays"]
    }
    assert nested["원시 rosbag 누적"]["Topic"]["Value"] == "/replay/raw_accumulated_cloud"
    assert nested["조밀 SC-PGO 최적화 지도"]["Topic"]["Value"] == "/track_visual/map_dense"
    assert nested["SC-PGO 키프레임/팩터"]["Topic"]["Value"] == "/track_visual/pose_graph"
    assert nested["경사 slope_deg"]["Color Layer"] == "slope_deg"
    assert nested["단차 step_height"]["Color Layer"] == "step_height"
    assert nested["수동 blocked keepout 마스크"]["Topic"]["Value"] == "/terrain/manual_keepout_mask"
