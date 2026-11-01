"""Build an efficient MuJoCo robot model from the delivered STEP/STL pair."""

from __future__ import annotations

import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

import numpy as np
import trimesh


WHEEL_NAMES = (
    "front_left",
    "front_right",
    "rear_left",
    "rear_right",
)

TRACK_CAMERA_NAMES = ("overview", "top", "entry", "start")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _step_inventory(path: Path) -> Dict[str, Any]:
    """Extract a reproducible, geometry-free inventory from an AP214 STEP file."""
    text = path.read_text(encoding="latin-1")
    products = [
        match.group(1).replace("''", "'")
        for match in re.finditer(
            r"PRODUCT\s*\(\s*'((?:[^']|'')*)'\s*,", text
        )
    ]
    return {
        "application_protocol": (
            "AP214" if "AUTOMOTIVE_DESIGN" in text else "unknown"
        ),
        "millimetre_units_declared": bool(re.search(
            r"SI_UNIT\s*\(\s*\.MILLI\.\s*,\s*\.METRE\.\s*\)", text
        )),
        "product_count": len(products),
        "products": sorted(set(products), key=str.casefold),
        "assembly_occurrence_count": text.count(
            "NEXT_ASSEMBLY_USAGE_OCCURRENCE"
        ),
        "manifold_solid_brep_count": text.count("MANIFOLD_SOLID_BREP"),
        "closed_shell_count": text.count("CLOSED_SHELL"),
        "product_definition_entity_count": text.count("PRODUCT_DEFINITION (")
        + text.count("PRODUCT_DEFINITION("),
        "product_definition_reference_count": text.count("PRODUCT_DEFINITION"),
    }


def _component_records(mesh: trimesh.Trimesh) -> List[Dict[str, Any]]:
    components = trimesh.graph.connected_components(
        mesh.face_adjacency,
        nodes=np.arange(len(mesh.faces), dtype=np.int64),
        min_len=1,
    )
    records: List[Dict[str, Any]] = []
    for index, faces in enumerate(components):
        faces = np.asarray(faces, dtype=np.int64)
        vertex_ids = np.unique(mesh.faces[faces].reshape(-1))
        points = mesh.vertices[vertex_ids]
        bounds = np.stack((points.min(axis=0), points.max(axis=0)))
        records.append({
            "index": index,
            "faces": faces,
            "face_count": int(len(faces)),
            "bounds": bounds,
            "extents": bounds[1] - bounds[0],
            "centre": bounds.mean(axis=0),
        })
    return records


def _identify_wheels(
    records: List[Dict[str, Any]], raw: Dict[str, Any]
) -> Tuple[Dict[str, List[Dict[str, Any]]], np.ndarray]:
    diameter = float(raw["diameter_mm"])
    width = float(raw["axial_width_mm"])
    tolerance = float(raw["dimension_tolerance_mm"])
    minimum_faces = int(raw["minimum_main_component_faces"])
    candidates = [
        record for record in records
        if record["face_count"] >= minimum_faces
        and abs(record["extents"][0] - diameter) <= tolerance
        and abs(record["extents"][1] - diameter) <= tolerance
        and abs(record["extents"][2] - width) <= tolerance
    ]
    if len(candidates) != 4:
        detail = [
            {
                "faces": item["face_count"],
                "extents_mm": np.round(item["extents"], 3).tolist(),
            }
            for item in candidates
        ]
        raise ValueError(f"Expected four MDH250 wheel components, found {detail}")

    reference = np.mean([item["centre"] for item in candidates], axis=0)
    main_by_name: Dict[str, Dict[str, Any]] = {}
    for item in candidates:
        front_rear = "front" if item["centre"][0] > reference[0] else "rear"
        left_right = "left" if item["centre"][2] < reference[2] else "right"
        name = f"{front_rear}_{left_right}"
        if name in main_by_name:
            raise ValueError(f"Duplicate wheel classification {name}")
        main_by_name[name] = item
    if set(main_by_name) != set(WHEEL_NAMES):
        raise ValueError(f"Incomplete wheel classification: {sorted(main_by_name)}")

    margin = float(raw["component_margin_mm"])
    groups: Dict[str, List[Dict[str, Any]]] = {
        name: [] for name in WHEEL_NAMES
    }
    for record in records:
        matches = []
        for name, main in main_by_name.items():
            lower = main["bounds"][0] - margin
            upper = main["bounds"][1] + margin
            if np.all(record["bounds"][0] >= lower) and np.all(
                record["bounds"][1] <= upper
            ):
                normalized = np.linalg.norm(
                    (record["centre"] - main["centre"])
                    / np.maximum(main["extents"], 1.0)
                )
                matches.append((float(normalized), name))
        if matches:
            groups[min(matches)[1]].append(record)

    assigned = {
        record["index"]
        for group in groups.values()
        for record in group
    }
    groups["body"] = [
        record for record in records if record["index"] not in assigned
    ]
    return groups, reference


def _cad_to_mujoco(points_mm: np.ndarray, reference_mm: np.ndarray) -> np.ndarray:
    relative = (np.asarray(points_mm, dtype=float) - reference_mm) * 0.001
    return np.column_stack((relative[:, 0], -relative[:, 2], relative[:, 1]))


def _group_mesh(
    source: trimesh.Trimesh,
    records: Iterable[Dict[str, Any]],
    reference_mm: np.ndarray,
    local_origin_m: np.ndarray | None = None,
) -> trimesh.Trimesh:
    face_ids = np.concatenate([
        record["faces"] for record in records
    ]).astype(np.int64)
    source_faces = source.faces[face_ids]
    vertex_ids, inverse = np.unique(source_faces.reshape(-1), return_inverse=True)
    vertices = _cad_to_mujoco(source.vertices[vertex_ids], reference_mm)
    if local_origin_m is not None:
        vertices -= np.asarray(local_origin_m, dtype=float)
    faces = inverse.reshape((-1, 3))
    result = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    result.remove_unreferenced_vertices()
    return result


def _export_obj(mesh: trimesh.Trimesh, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        trimesh.exchange.obj.export_obj(
            mesh,
            include_normals=False,
            include_color=False,
            include_texture=False,
            header=None,
        ),
        encoding="utf-8",
    )


def generate_robot_assets(
    step_path: Path,
    stl_path: Path,
    output_root: Path,
    config: Dict[str, Any],
) -> Dict[str, Any]:
    step_path = Path(step_path).resolve()
    stl_path = Path(stl_path).resolve()
    if not step_path.is_file():
        raise FileNotFoundError(step_path)
    if not stl_path.is_file():
        raise FileNotFoundError(stl_path)

    mesh = trimesh.load_mesh(stl_path, process=True)
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(f"Expected one STL mesh, got {type(mesh).__name__}")
    if len(mesh.faces) < 1000 or not np.isfinite(mesh.vertices).all():
        raise ValueError("Delivered STL is empty or contains non-finite vertices")

    records = _component_records(mesh)
    groups, reference_mm = _identify_wheels(
        records, config["wheel_identification"]
    )
    visual_root = output_root / "assets" / "robot" / "visual"
    wheel_records: Dict[str, Dict[str, Any]] = {}
    exported_face_count = 0

    body_mesh = _group_mesh(mesh, groups["body"], reference_mm)
    _export_obj(body_mesh, visual_root / "base_and_locked_arm.obj")
    exported_face_count += len(body_mesh.faces)

    for name in WHEEL_NAMES:
        main = max(groups[name], key=lambda item: item["face_count"])
        wheel_centre_m = _cad_to_mujoco(
            main["centre"].reshape((1, 3)), reference_mm
        )[0]
        wheel_mesh = _group_mesh(
            mesh, groups[name], reference_mm, local_origin_m=wheel_centre_m
        )
        _export_obj(wheel_mesh, visual_root / f"wheel_{name}.obj")
        exported_face_count += len(wheel_mesh.faces)
        wheel_records[name] = {
            "position_m": np.round(wheel_centre_m, 9).tolist(),
            "cad_centre_mm": np.round(main["centre"], 6).tolist(),
            "main_component_extents_mm": np.round(
                main["extents"], 6
            ).tolist(),
            "component_count": len(groups[name]),
            "visual_vertex_count": int(len(wheel_mesh.vertices)),
            "visual_face_count": int(len(wheel_mesh.faces)),
        }

    if exported_face_count != len(mesh.faces):
        raise ValueError(
            f"Face partition lost geometry: {exported_face_count} != {len(mesh.faces)}"
        )

    metadata = {
        "schema_version": 1,
        "name": config["name"],
        "source": {
            "step_filename": step_path.name,
            "step_size_bytes": step_path.stat().st_size,
            "step_sha256": _sha256(step_path),
            "stl_filename": stl_path.name,
            "stl_size_bytes": stl_path.stat().st_size,
            "stl_sha256": _sha256(stl_path),
        },
        "step_inventory": _step_inventory(step_path),
        "axis_transform": {
            "description": "MuJoCo X=CAD X, Y=-CAD Z, Z=CAD Y",
            "scale": 0.001,
            "reference_cad_mm": np.round(reference_mm, 9).tolist(),
            "matrix_cad_to_mujoco": [
                [0.001, 0.0, 0.0],
                [0.0, 0.0, -0.001],
                [0.0, 0.001, 0.0],
            ],
        },
        "source_mesh": {
            "vertex_count_after_merge": int(len(mesh.vertices)),
            "face_count": int(len(mesh.faces)),
            "connected_component_count": len(records),
            "bounds_cad_mm": np.round(mesh.bounds, 6).tolist(),
            "extents_cad_mm": np.round(mesh.extents, 6).tolist(),
            "watertight": bool(mesh.is_watertight),
        },
        "body": {
            "visual_vertex_count": int(len(body_mesh.vertices)),
            "visual_face_count": int(len(body_mesh.faces)),
            "component_count": len(groups["body"]),
            "visual_bounds_m": np.round(body_mesh.bounds, 9).tolist(),
        },
        "wheels": wheel_records,
        "exported_visual_face_count": int(exported_face_count),
        "collision_policy": "primitive boxes and cylinders only",
        "sensors": config["sensors"],
        "simulation_stage": config["simulation_stage"],
    }
    metadata_path = output_root / "assets" / "robot" / "robot_metadata.json"
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return metadata


def _numbers(values: Iterable[float]) -> str:
    return " ".join(f"{float(value):.9g}" for value in values)


def _robot_xml_fragments(
    metadata: Dict[str, Any],
    config: Dict[str, Any],
    *,
    driver_camera: bool = False,
) -> Tuple[str, str, str, str]:
    assets = [
        '    <material name="robot_frame_mat" rgba="0.16 0.22 0.30 1" '
        'specular="0.24" shininess="0.32" roughness="0.58"/>',
        '    <material name="robot_wheel_mat" rgba="0.035 0.040 0.045 1" '
        'specular="0.06" shininess="0.08" roughness="0.94"/>',
        '    <mesh name="robot_base_visual" '
        'file="../assets/robot/visual/base_and_locked_arm.obj" '
        'smoothnormal="true"/>',
    ]
    for name in WHEEL_NAMES:
        assets.append(
            f'    <mesh name="robot_wheel_{name}_visual" '
            f'file="../assets/robot/visual/wheel_{name}.obj" '
            'smoothnormal="true"/>'
        )

    initial = config["initial_pose"]
    roll, pitch, yaw = np.deg2rad([
        initial["roll_deg"], initial["pitch_deg"], initial["yaw_deg"]
    ])
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    quat = [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]
    base = config["base_physics"]
    wheel = config["wheel_physics"]
    contact = wheel["contact"]
    body_lines = [
        f'    <body name="base_link" pos="{_numbers(initial["position_m"])}" '
        f'quat="{_numbers(quat)}">',
        '      <freejoint name="floating_base_joint"/>',
        f'      <inertial pos="{_numbers(base["centre_of_mass_m"])}" '
        f'mass="{float(base["mass_kg_excluding_wheels"]):.9g}" '
        f'diaginertia="{_numbers(base["diagonal_inertia_kg_m2"])}"/>',
        '      <geom name="robot_base_visual_geom" type="mesh" '
        'mesh="robot_base_visual" material="robot_frame_mat" '
        'mass="0" contype="0" conaffinity="0" group="0"/>',
    ]
    for box in base["collision_boxes"]:
        body_lines.append(
            f'      <geom name="robot_{box["name"]}_collision" type="box" '
            f'pos="{_numbers(box["position_m"])}" '
            f'size="{_numbers(box["half_size_m"])}" mass="0" '
            'rgba="0 0 0 0" group="3" contype="1" conaffinity="1"/>'
        )

    friction = _numbers(contact["friction"])
    solref = _numbers(contact["solref"])
    solimp = _numbers(contact["solimp"])
    for name in WHEEL_NAMES:
        position = list(metadata["wheels"][name]["position_m"])
        position[2] = 0.0
        body_lines.extend([
            f'      <body name="{name}_wheel_link" pos="{_numbers(position)}">',
            f'        <joint name="{name}_wheel_joint" type="hinge" '
            'axis="0 1 0" limited="false" '
            f'damping="{float(wheel["joint_damping_nms_per_rad"]):.9g}" '
            f'frictionloss="{float(wheel["joint_frictionloss_nm"]):.9g}" '
            f'armature="{float(wheel["joint_armature_kg_m2"]):.9g}"/>',
            f'        <geom name="{name}_wheel_visual_geom" type="mesh" '
            f'mesh="robot_wheel_{name}_visual" material="robot_wheel_mat" '
            'mass="0" contype="0" conaffinity="0" group="0"/>',
            f'        <geom name="{name}_wheel_collision" type="ellipsoid" '
            f'size="{float(wheel["radius_m"]):.9g} '
            f'{float(wheel["half_width_m"]):.9g} '
            f'{float(wheel["radius_m"]):.9g}" '
            f'mass="{float(wheel["mass_kg_each"]):.9g}" '
            f'friction="{friction}" condim="{int(contact["condim"])}" '
            f'solref="{solref}" solimp="{solimp}" priority="2" '
            'rgba="0 0 0 0" group="3" contype="1" conaffinity="1"/>',
            '      </body>',
        ])
    imu = config["sensors"]["imu"]
    body_lines.append(
        f'      <site name="base_imu_site" pos="{_numbers(imu["position_m"])}" '
        'size="0.008" rgba="0.2 0.8 1 0"/>'
    )
    if driver_camera:
        camera = config["driver_camera"]
        body_lines.append(
            f'      <camera name="{camera["name"]}" '
            f'pos="{_numbers(camera["position_m"])}" '
            f'xyaxes="{_numbers(camera["xyaxes"])}" '
            f'resolution="{int(camera["offscreen_resolution"][0])} '
            f'{int(camera["offscreen_resolution"][1])}" '
            f'fovy="{float(camera["vertical_fov_deg"]):.9g}"/>'
        )
    else:
        body_lines.append(
            '      <camera name="robot_follow" mode="trackcom" '
            'pos="-1.4 -1.2 0.9" '
            'xyaxes="0.707107 -0.707107 0 0.408248 0.408248 0.816497"/>'
        )
    body_lines.extend([
        '    </body>',
    ])

    maximum_speed = float(wheel["maximum_speed_rpm"]) * 2.0 * math.pi / 60.0
    maximum_torque = float(wheel["maximum_torque_nm"])
    actuators = ["  <actuator>"]
    for name in WHEEL_NAMES:
        actuators.append(
            f'    <velocity name="{name}_wheel_actuator" '
            f'joint="{name}_wheel_joint" '
            f'kv="{float(wheel["velocity_gain"]):.9g}" '
            f'ctrllimited="true" ctrlrange="{-maximum_speed:.9g} {maximum_speed:.9g}" '
            f'forcelimited="true" forcerange="{-maximum_torque:.9g} {maximum_torque:.9g}"/>'
        )
    actuators.append("  </actuator>")
    sensors = """  <sensor>
    <framepos name="base_position" objtype="body" objname="base_link"/>
    <framequat name="base_imu_quat" objtype="site" objname="base_imu_site"/>
    <gyro name="base_imu_gyro" site="base_imu_site"/>
    <accelerometer name="base_imu_accel" site="base_imu_site"/>
  </sensor>"""
    return "\n".join(assets), "\n".join(body_lines), "\n".join(actuators), sensors


def compose_robot_scene(
    track_scene: Path,
    output_scene: Path,
    metadata: Dict[str, Any],
    config: Dict[str, Any],
    *,
    driver_camera: bool = False,
) -> None:
    source = Path(track_scene).read_text(encoding="utf-8")
    asset_xml, body_xml, actuator_xml, sensor_xml = _robot_xml_fragments(
        metadata, config, driver_camera=driver_camera
    )
    if driver_camera:
        camera_pattern = re.compile(
            r'\n    <camera name="(?:'
            + "|".join(TRACK_CAMERA_NAMES)
            + r')".*?/>',
            re.DOTALL,
        )
        source, removed_camera_count = camera_pattern.subn("", source)
        if removed_camera_count != len(TRACK_CAMERA_NAMES):
            raise ValueError(
                "Expected to remove four fixed track cameras, removed "
                f"{removed_camera_count}"
            )
        width, height = config["driver_camera"]["offscreen_resolution"]
        source, width_count = re.subn(
            r'(\boffwidth=")[^"]+("\s+offheight=")',
            rf'\g<1>{int(width)}\g<2>',
            source,
            count=1,
        )
        source, height_count = re.subn(
            r'(\boffheight=")[^"]+("/>)',
            rf'\g<1>{int(height)}\g<2>',
            source,
            count=1,
        )
        if width_count != 1 or height_count != 1:
            raise ValueError("Track scene offscreen resolution was not found")
    if source.count("  </asset>") != 1 or source.count("  </worldbody>") != 1:
        raise ValueError("Track scene does not have one asset/worldbody section")
    source = source.replace("  </asset>", f"{asset_xml}\n  </asset>")
    source = source.replace("  </worldbody>", f"{body_xml}\n  </worldbody>")
    source = source.replace(
        "</mujoco>", f"{actuator_xml}\n{sensor_xml}\n</mujoco>"
    )
    output_scene.parent.mkdir(parents=True, exist_ok=True)
    output_scene.write_text(source, encoding="utf-8")


def generate_robot(
    step_path: Path,
    stl_path: Path,
    config_path: Path,
    output_root: Path,
) -> Dict[str, Any]:
    config = json.loads(Path(config_path).read_text(encoding="utf-8"))
    metadata = generate_robot_assets(
        step_path, stl_path, output_root, config
    )
    track_scene = output_root / "models" / "pk3a_closed_loop.xml"
    output_scene = output_root / "models" / "pk3a_robot_scene.xml"
    if not track_scene.is_file():
        raise FileNotFoundError(
            f"Generate the track before composing the robot: {track_scene}"
        )
    compose_robot_scene(track_scene, output_scene, metadata, config)
    driver_scene = output_root / "models" / "pk3a_robot_drive_scene.xml"
    compose_robot_scene(
        track_scene,
        driver_scene,
        metadata,
        config,
        driver_camera=True,
    )
    return {
        "scene": str(output_scene),
        "driver_scene": str(driver_scene),
        "metadata": str(output_root / "assets" / "robot" / "robot_metadata.json"),
        "visual_face_count": metadata["exported_visual_face_count"],
        "wheel_positions_m": {
            name: metadata["wheels"][name]["position_m"]
            for name in WHEEL_NAMES
        },
    }
