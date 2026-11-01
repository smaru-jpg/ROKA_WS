"""Generate the first independently reviewable PK-3A closed-loop section."""

from __future__ import annotations

import json
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np

from .footprint_overlap import find_nonadjacent_overlaps
from .module_geometry import (
    ModuleSpec,
    connector_geometry,
    load_json,
    mirror_module_y,
    module_bounds,
    orient_course_across_width,
    package_root,
    rasterize_module,
    surface_height,
    surface_point,
)
from .solid_geometry import (
    DEFAULT_VISUAL_SPACING_M,
    GROUND_Z,
    write_module_assets,
)
from .port_assembly import (
    PlanPose,
    assemble_route,
    place_at_port,
    transform_direction,
    transform_xy,
    transform_xyz,
)
from .port_geometry import load_port_sources, sample_port
from .slope_analysis import (
    annotate_seam_tangent_metrics,
    surface_slope_metrics,
)


def _half_straight_spec(raw: Dict[str, Any]) -> ModuleSpec:
    properties = dict(raw)
    properties.update({"page": 3, "confidence": "user_defined_closed_loop_filler"})
    return ModuleSpec(
        name=str(raw["name"]),
        page=3,
        shape=str(raw["shape"]),
        length=float(raw["length_m"]),
        width=float(raw["width_m"]),
        inlet=np.asarray(raw["inlet_right_left_m"], dtype=float),
        outlet=np.asarray(raw["outlet_right_left_m"], dtype=float),
        properties=properties,
    )


def _source_module_name(spec: ModuleSpec) -> str:
    source = spec.properties.get("_mirror_source_spec")
    if source is None:
        source = spec.properties.get("_course_axis_source_spec")
    if source is None:
        source = spec.properties.get("_turn_variant_source_spec")
    if source is not None:
        return _source_module_name(source)
    return spec.name


def _turn_variant_spec(
    source: ModuleSpec,
    name: str,
    turn_deg: float,
) -> ModuleSpec:
    """Make a plan-curvature variant without changing the source surface.

    Negative turns are produced by mirroring a positive-angle source variant.
    This keeps the bank handedness and the curved footprint driven by the same
    signed angle instead of only changing a metadata field on an existing
    mirrored module.
    """
    if turn_deg == 0.0:
        raise ValueError(f"Turn variant {name} must have a non-zero turn")
    if "turn_deg" not in source.properties:
        raise ValueError(f"Turn variant source {source.name} is not curved")
    properties = dict(source.properties)
    properties["turn_deg"] = abs(float(turn_deg))
    properties["_turn_variant_source_spec"] = source
    properties["turn_variant_source_name"] = source.name
    positive = ModuleSpec(
        name=f"{name}_POSITIVE_SOURCE",
        page=source.page,
        shape=source.shape,
        length=source.length,
        width=source.width,
        inlet=source.inlet.copy(),
        outlet=source.outlet.copy(),
        properties=properties,
    )
    if turn_deg < 0.0:
        return mirror_module_y(positive, name)
    return ModuleSpec(
        name=name,
        page=positive.page,
        shape=positive.shape,
        length=positive.length,
        width=positive.width,
        inlet=positive.inlet,
        outlet=positive.outlet,
        properties=positive.properties,
    )


def _module_world_bounds(spec: ModuleSpec, pose: PlanPose) -> np.ndarray:
    bounds = module_bounds(spec)
    corners = np.array([
        [bounds[0], bounds[2]],
        [bounds[0], bounds[3]],
        [bounds[1], bounds[2]],
        [bounds[1], bounds[3]],
    ])
    world = transform_xy(corners, pose)
    return np.array([
        world[:, 0].min(),
        world[:, 0].max(),
        world[:, 1].min(),
        world[:, 1].max(),
    ])


def _world_bounds(instances: List[Dict[str, Any]], modules: Dict[str, ModuleSpec]) -> np.ndarray:
    bounds = np.asarray([
        _module_world_bounds(modules[item["module"]], item["pose"])
        for item in instances
    ])
    return np.array([
        bounds[:, 0].min(),
        bounds[:, 1].max(),
        bounds[:, 2].min(),
        bounds[:, 3].max(),
    ])


def _asset_data(
    modules: Dict[str, ModuleSpec],
    visual_spacing: float,
    collision_spacing: float,
    visual_root: Path,
    collision_root: Path,
) -> Tuple[Dict[str, Dict[str, Any]], List[str]]:
    colours = {
        "3DB1": "0.64 0.46 0.30 1",
        "3BA1": "0.76 0.57 0.36 1",
        "3TA1L": "0.76 0.64 0.43 1",
        "F_HALF": "0.42 0.64 0.72 1",
        "F_CLOSE": "0.35 0.62 0.72 1",
        "3TB1L": "0.69 0.49 0.32 1",
        "3CV1": "0.73 0.56 0.36 1",
    }
    records: Dict[str, Dict[str, Any]] = {}
    xml = []
    for name, spec in modules.items():
        source_name = _source_module_name(spec)
        records[name] = write_module_assets(
            name,
            spec,
            visual_root,
            collision_root,
            visual_spacing=visual_spacing,
            collision_spacing=collision_spacing,
            ground_z=GROUND_Z,
        )
        records[name]["material_rgba"] = colours[source_name]
        record = records[name]
        bounds = record["bounds"]
        x_half = 0.5 * (bounds[1] - bounds[0])
        y_half = 0.5 * (bounds[3] - bounds[2])
        xml.extend([
            f'    <material name="mat_{name}" rgba="{colours[source_name]}" '
            'specular="0.08" shininess="0.12" roughness="0.90"/>',
            f'    <mesh name="visual_{name}" file="../assets/visual/modules/{name}.obj" '
            'smoothnormal="false"/>',
            f'    <hfield name="collision_{name}" '
            f'file="../assets/collision/modules/{name}.png" '
            f'size="{x_half:.9f} {y_half:.9f} {record["height_range"]:.9f} '
            f'{record["base_depth"]:.9f}"/>',
        ])
    return records, xml


def _standalone_module_xml(
    name: str,
    spec: ModuleSpec,
    asset: Dict[str, Any],
    surface_contact: Dict[str, Any],
) -> str:
    """Return an independently loadable solid-module MuJoCo model."""
    friction = " ".join(str(value) for value in surface_contact["friction"])
    solref = " ".join(str(value) for value in surface_contact["solref"])
    solimp = " ".join(str(value) for value in surface_contact["solimp"])
    bounds = asset["bounds"]
    x_half = 0.5 * (bounds[1] - bounds[0])
    y_half = 0.5 * (bounds[3] - bounds[2])
    centre = asset["bounds_centre"]
    connector = connector_geometry(spec)
    inlet = np.asarray(connector["inlet_point"], dtype=float)
    outlet = np.asarray(connector["outlet_point"], dtype=float)
    inlet_left = inlet + 0.5 * spec.width * np.array([
        -np.sin(connector["inlet_heading"]),
        np.cos(connector["inlet_heading"]),
    ])
    inlet_right = 2.0 * inlet - inlet_left
    outlet_left = outlet + 0.5 * spec.width * np.array([
        -np.sin(connector["outlet_heading"]),
        np.cos(connector["outlet_heading"]),
    ])
    outlet_right = 2.0 * outlet - outlet_left
    extent = max(1.5, spec.length + spec.width)
    return f'''<?xml version="1.0" encoding="utf-8"?>
<!-- Independently reviewable watertight PK-3A solid module. -->
<mujoco model="PK-3A solid module {name}">
  <compiler angle="radian"/>
  <option timestep="0.002" integrator="implicitfast" gravity="0 0 -9.81"/>
  <visual>
    <headlight ambient="0.34 0.34 0.34" diffuse="0.76 0.76 0.76"/>
    <quality shadowsize="1024" offsamples="1"/>
    <global azimuth="135" elevation="-28" offwidth="1100" offheight="720"/>
  </visual>
  <statistic center="0 0 0.20" extent="{extent:.6f}"/>
  <asset>
    <texture name="ground_tex" type="2d" builtin="checker" width="128" height="128"
             rgb1="0.40 0.41 0.42" rgb2="0.31 0.32 0.33"/>
    <material name="ground_mat" texture="ground_tex" texrepeat="8 8"
              texuniform="true" roughness="0.96"/>
    <material name="module_mat" rgba="{asset["material_rgba"]}"
              specular="0.08" shininess="0.12" roughness="0.90"/>
    <mesh name="module_visual" file="../../assets/visual/modules/{name}.obj" smoothnormal="false"/>
    <hfield name="module_collision" file="../../assets/collision/modules/{name}.png"
            size="{x_half:.9f} {y_half:.9f} {asset["height_range"]:.9f} {asset["base_depth"]:.9f}"/>
  </asset>
  <worldbody>
    <light pos="-2 -3 5" dir="0.2 0.3 -1" directional="true" castshadow="false"/>
    <geom name="ground" type="plane" pos="0 0 {GROUND_Z:.6f}" size="3 3 0.1"
          rgba="0 0 0 0" friction="0.9 0.01 0.001"/>
    <geom name="ground_visual" type="box" pos="0 0 {GROUND_Z - 0.025:.6f}"
          size="3 3 0.025" material="ground_mat" contype="0" conaffinity="0"/>
    <body name="module">
      <geom name="visual_{name}" type="mesh" mesh="module_visual" material="module_mat"
            contype="0" conaffinity="0"/>
      <geom name="track_{name}" type="hfield" hfield="module_collision"
            pos="{centre[0]:.9f} {centre[1]:.9f} {asset["minimum"]:.9f}"
            rgba="0 0 0 0" group="4" friction="{friction}"
            condim="{int(surface_contact["condim"])}"
            priority="{int(surface_contact["priority"])}"
            solmix="{float(surface_contact["solmix"]):.9g}"
            solref="{solref}" solimp="{solimp}"/>
      <site name="inlet_right" pos="{inlet_right[0]:.9f} {inlet_right[1]:.9f} 0" size="0.01" rgba="0 0 1 0"/>
      <site name="inlet_left" pos="{inlet_left[0]:.9f} {inlet_left[1]:.9f} 0" size="0.01" rgba="0 0 1 0"/>
      <site name="outlet_right" pos="{outlet_right[0]:.9f} {outlet_right[1]:.9f} 0" size="0.01" rgba="1 0 0 0"/>
      <site name="outlet_left" pos="{outlet_left[0]:.9f} {outlet_left[1]:.9f} 0" size="0.01" rgba="1 0 0 0"/>
    </body>
    <camera name="module_view" pos="{-extent:.6f} {-extent:.6f} {0.75 * extent:.6f}"
            xyaxes="0.707107 -0.707107 0 0.408248 0.408248 0.816497"/>
    <camera name="module_top" pos="0 0 {1.4 * extent:.6f}" xyaxes="1 0 0 0 1 0"/>
  </worldbody>
</mujoco>
'''


def _write_standalone_module_models(
    model_root: Path,
    modules: Dict[str, ModuleSpec],
    assets: Dict[str, Dict[str, Any]],
    surface_contact: Dict[str, Any],
) -> None:
    module_root = model_root / "modules"
    module_root.mkdir(parents=True, exist_ok=True)
    for name, spec in modules.items():
        (module_root / f"{name}.xml").write_text(
            _standalone_module_xml(name, spec, assets[name], surface_contact),
            encoding="utf-8",
        )


def _seam_visuals(seams: List[Dict[str, Any]]) -> str:
    geoms = []
    for seam in seams:
        points = np.asarray(seam["points_xyz_m"], dtype=float)[::10]
        for segment in range(len(points) - 1):
            start = points[segment].copy()
            end = points[segment + 1].copy()
            start[2] += 0.012
            end[2] += 0.012
            geoms.append(
                f'    <geom name="seam_{seam["index"]:02d}_{segment:02d}" '
                'type="capsule" '
                f'fromto="{start[0]:.6f} {start[1]:.6f} {start[2]:.6f} '
                f'{end[0]:.6f} {end[1]:.6f} {end[2]:.6f}" '
                'size="0.004" rgba="0.10 0.95 0.40 1" '
                'contype="0" conaffinity="0"/>'
            )
    return "\n".join(geoms)


def _start_zone_geometry(
    raw: Dict[str, Any] | None,
    instances: List[Dict[str, Any]],
    modules: Dict[str, ModuleSpec],
) -> Dict[str, Any] | None:
    """Resolve a visual-only start section against an assembled instance."""
    if not raw:
        return None
    instance_ids = [str(value) for value in raw.get("instance_ids", [])]
    if not instance_ids:
        raise ValueError("start_zone.instance_ids must not be empty")
    line_instance_id = str(raw.get("line_instance_id", instance_ids[0]))
    by_id = {item["id"]: item for item in instances}
    missing = [value for value in instance_ids if value not in by_id]
    if line_instance_id not in by_id:
        missing.append(line_instance_id)
    if missing:
        raise ValueError(f"Unknown start-zone instance(s): {sorted(set(missing))}")

    item = by_id[line_instance_id]
    spec = modules[item["module"]]
    progress = float(raw.get("line_progress_m", 0.08))
    progress = float(np.clip(progress, 0.0, spec.length))
    lateral_inset = float(raw.get("lateral_inset_m", 0.025))
    lateral_limit = max(0.0, 0.5 * spec.width - lateral_inset)
    lateral = np.linspace(-lateral_limit, lateral_limit, 17)
    local_xy = np.asarray([
        surface_point(spec, progress, float(value))
        for value in lateral
    ])
    local_height = surface_height(
        spec,
        np.full_like(lateral, progress),
        lateral,
    )
    local_xyz = np.column_stack((local_xy, local_height))
    world_xyz = transform_xyz(local_xyz, item["pose"])
    return {
        "label": str(raw.get("label", "START")),
        "instance_ids": instance_ids,
        "line_instance_id": line_instance_id,
        "line_progress_m": progress,
        "line_width_m": float(raw.get("line_width_m", 0.05)),
        "zone_rgba": [float(value) for value in raw.get(
            "zone_rgba", [0.08, 0.38, 0.92, 1.0]
        )],
        "line_rgba": [float(value) for value in raw.get(
            "line_rgba", [1.0, 1.0, 1.0, 1.0]
        )],
        "line_points_xyz_m": world_xyz,
        "line_centre_xyz_m": world_xyz.mean(axis=0),
    }


def _start_zone_visuals(start_zone: Dict[str, Any] | None) -> str:
    if start_zone is None:
        return ""
    points = np.asarray(start_zone["line_points_xyz_m"], dtype=float).copy()
    points[:, 2] += 0.014
    radius = 0.5 * float(start_zone["line_width_m"])
    rgba = " ".join(f"{value:.3f}" for value in start_zone["line_rgba"])
    geoms = []
    for index, (start, end) in enumerate(zip(points[:-1], points[1:])):
        geoms.append(
            f'    <geom name="start_line_{index:02d}" type="capsule" '
            f'fromto="{start[0]:.6f} {start[1]:.6f} {start[2]:.6f} '
            f'{end[0]:.6f} {end[1]:.6f} {end[2]:.6f}" '
            f'size="{radius:.6f}" rgba="{rgba}" '
            'contype="0" conaffinity="0"/>'
        )
    return "\n".join(geoms)


def _model_xml(
    instances: List[Dict[str, Any]],
    seams: List[Dict[str, Any]],
    modules: Dict[str, ModuleSpec],
    assets: Dict[str, Dict[str, Any]],
    asset_xml: List[str],
    surface_contact: Dict[str, Any],
    contact_solver: Dict[str, Any],
    bounds: np.ndarray,
    model_name: str,
    start_zone: Dict[str, Any] | None = None,
) -> str:
    friction = " ".join(str(value) for value in surface_contact["friction"])
    solref = " ".join(str(value) for value in surface_contact["solref"])
    solimp = " ".join(str(value) for value in surface_contact["solimp"])
    condim = int(surface_contact["condim"])
    priority = int(surface_contact["priority"])
    solmix = float(surface_contact["solmix"])
    option_attributes = (
        f'cone="{contact_solver["cone"]}" '
        f'solver="{contact_solver["solver"]}" '
        f'iterations="{int(contact_solver["iterations"])}" '
        f'tolerance="{float(contact_solver["tolerance"]):.12g}" '
        f'impratio="{float(contact_solver["impratio"]):.9g}"'
    )
    multiccd_flag = (
        '    <flag multiccd="enable"/>\n'
        if bool(contact_solver.get("multiccd", False))
        else ""
    )
    module_bodies = []
    for item in instances:
        pose = item["pose"]
        asset = assets[item["module"]]
        half_yaw = 0.5 * pose.yaw
        material = (
            "mat_start_zone"
            if start_zone is not None and item["id"] in start_zone["instance_ids"]
            else f'mat_{item["module"]}'
        )
        local_centre = asset["bounds_centre"]
        module_bodies.append(
            f'    <body name="module_{item["id"]}" '
            f'pos="{pose.xy[0]:.9f} {pose.xy[1]:.9f} 0" '
            f'quat="{np.cos(half_yaw):.9f} 0 0 {np.sin(half_yaw):.9f}">\n'
            f'      <geom name="visual_{item["id"]}" type="mesh" '
            f'mesh="visual_{item["module"]}" material="{material}" '
            'contype="0" conaffinity="0" group="0"/>\n'
            f'      <geom name="track_{item["id"]}" type="hfield" '
            f'hfield="collision_{item["module"]}" '
            f'pos="{local_centre[0]:.9f} {local_centre[1]:.9f} '
            f'{asset["minimum"]:.9f}" rgba="0 0 0 0" group="4" '
            f'friction="{friction}" condim="{condim}" priority="{priority}" '
            f'solmix="{solmix:.9g}" solref="{solref}" solimp="{solimp}"/>\n'
            '    </body>'
        )
    centre = np.array([
        0.5 * (bounds[0] + bounds[1]),
        0.5 * (bounds[2] + bounds[3]),
    ])
    span = max(bounds[1] - bounds[0], bounds[3] - bounds[2])
    extent = max(5.0, 0.75 * span)
    ground_half = 0.75 * span + 2.0
    overview_pos = np.array([centre[0] - extent, centre[1] - extent, extent])
    if start_zone is None:
        start_centre = np.array([bounds[0], bounds[2], 0.4])
    else:
        start_centre = np.asarray(
            start_zone["line_centre_xyz_m"],
            dtype=float,
        )
    start_camera_pos = start_centre + np.array([-2.4, -2.4, 2.0])
    start_material = ""
    if start_zone is not None:
        rgba = " ".join(f"{value:.3f}" for value in start_zone["zone_rgba"])
        start_material = (
            f'    <material name="mat_start_zone" rgba="{rgba}" '
            'specular="0.12" shininess="0.18" roughness="0.75"/>\n'
        )
    return f'''<?xml version="1.0" encoding="utf-8"?>
<!-- {model_name} generated from approved module profiles. -->
<mujoco model="{model_name}">
  <compiler angle="radian"/>
  <option timestep="0.002" integrator="implicitfast" gravity="0 0 -9.81" {option_attributes}>
{multiccd_flag}  </option>
  <visual>
    <headlight diffuse="0.72 0.72 0.72"
               ambient="0.32 0.32 0.32"
               specular="0.12 0.12 0.12"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <quality shadowsize="1024" offsamples="1"/>
    <map znear="0.01" zfar="100"/>
    <global azimuth="135" elevation="-35" offwidth="1100" offheight="620"/>
  </visual>
  <statistic center="{centre[0]:.6f} {centre[1]:.6f} 0.20" extent="{extent:.6f}"/>
  <asset>
    <texture name="ground_tex" type="2d" builtin="checker" width="256" height="256"
             rgb1="0.40 0.41 0.42" rgb2="0.31 0.32 0.33"/>
    <material name="ground_mat" texture="ground_tex" texrepeat="18 18"
              texuniform="true" reflectance="0.04" roughness="0.96"/>
{start_material}{chr(10).join(asset_xml)}
  </asset>
  <worldbody>
    <light pos="{centre[0] - 3:.3f} {centre[1] - 4:.3f} 10" dir="0.2 0.3 -1"
           directional="true" castshadow="false"/>
    <geom name="ground" type="plane" pos="{centre[0]:.6f} {centre[1]:.6f} {GROUND_Z:.6f}"
          size="{ground_half:.3f} {ground_half:.3f} 0.1"
          rgba="0 0 0 0" friction="0.9 0.01 0.001"/>
    <geom name="ground_visual" type="box"
          pos="{centre[0]:.6f} {centre[1]:.6f} {GROUND_Z - 0.025:.6f}"
          size="{ground_half:.3f} {ground_half:.3f} 0.025"
          material="ground_mat" contype="0" conaffinity="0"/>
{chr(10).join(module_bodies)}
{_start_zone_visuals(start_zone)}
    <camera name="overview" pos="{overview_pos[0]:.6f} {overview_pos[1]:.6f} {overview_pos[2]:.6f}"
            xyaxes="0.707107 -0.707107 0 0.408248 0.408248 0.816497"/>
    <camera name="top" pos="{centre[0]:.6f} {centre[1]:.6f} {1.8 * extent:.6f}"
            xyaxes="1 0 0 0 1 0"/>
    <camera name="entry" pos="{start_camera_pos[0]:.6f} {start_camera_pos[1]:.6f} {start_camera_pos[2]:.6f}"
            xyaxes="0.707107 -0.707107 0 0.408248 0.408248 0.816497"/>
    <camera name="start" pos="{start_camera_pos[0]:.6f} {start_camera_pos[1]:.6f} {start_camera_pos[2]:.6f}"
            xyaxes="0.707107 -0.707107 0 0.408248 0.408248 0.816497"/>
  </worldbody>
</mujoco>
'''


def _serializable_instance(item: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "index": item["index"],
        "id": item["id"],
        "module": item["module"],
        "input_port": item["input_port"],
        "output_port": item["output_port"],
        "pose_xy_m": np.round(item["pose"].xy, 9).tolist(),
        "yaw_deg": round(float(np.rad2deg(item["pose"].yaw)), 9),
    }


def _serializable_start_zone(
    start_zone: Dict[str, Any] | None,
) -> Dict[str, Any] | None:
    if start_zone is None:
        return None
    return {
        "label": start_zone["label"],
        "instance_ids": list(start_zone["instance_ids"]),
        "line_instance_id": start_zone["line_instance_id"],
        "line_progress_m": start_zone["line_progress_m"],
        "line_width_m": start_zone["line_width_m"],
        "line_centre_xyz_m": np.round(
            start_zone["line_centre_xyz_m"], 9
        ).tolist(),
        "line_points_xyz_m": np.round(
            start_zone["line_points_xyz_m"], 9
        ).tolist(),
    }


def _surface_contact_settings(generation: Dict[str, Any]) -> Dict[str, Any]:
    """Return a validated MuJoCo contact profile with legacy fallback."""
    raw = generation.get("surface_contact")
    if raw is None:
        raw = {
            "friction": generation["friction"],
            "condim": 4,
            "solref": [0.006, 1.0],
            "solimp": [0.95, 0.99, 0.001],
            "priority": 0,
            "solmix": 1.0,
            "interpretation": "legacy contact profile",
        }
    result = {
        "friction": [float(value) for value in raw["friction"]],
        "condim": int(raw.get("condim", 4)),
        "solref": [float(value) for value in raw.get("solref", [0.006, 1.0])],
        "solimp": [float(value) for value in raw.get(
            "solimp", [0.95, 0.99, 0.001]
        )],
        "priority": int(raw.get("priority", 0)),
        "solmix": float(raw.get("solmix", 1.0)),
        "interpretation": str(raw.get("interpretation", "")),
    }
    if len(result["friction"]) != 3 or any(
        value < 0.0 for value in result["friction"]
    ):
        raise ValueError("surface_contact.friction must contain 3 non-negative values")
    if result["condim"] not in (1, 3, 4, 6):
        raise ValueError("surface_contact.condim must be one of 1, 3, 4 or 6")
    if len(result["solref"]) != 2:
        raise ValueError("surface_contact.solref must contain 2 values")
    if len(result["solimp"]) not in (3, 5):
        raise ValueError("surface_contact.solimp must contain 3 or 5 values")
    return result


def _contact_solver_settings(generation: Dict[str, Any]) -> Dict[str, Any]:
    raw = generation.get("contact_solver", {})
    return {
        "cone": str(raw.get("cone", "pyramidal")),
        "solver": str(raw.get("solver", "Newton")),
        "iterations": int(raw.get("iterations", 50)),
        "tolerance": float(raw.get("tolerance", 1.0e-8)),
        "impratio": float(raw.get("impratio", 1.0)),
        "multiccd": bool(raw.get("multiccd", False)),
        "interpretation": str(raw.get("interpretation", "")),
    }


def _append_closure_seam(
    closure: Dict[str, Any],
    instances: List[Dict[str, Any]],
    seams: List[Dict[str, Any]],
    modules: Dict[str, ModuleSpec],
    aliases: Dict[str, Dict[str, str]],
    sample_count: int,
    maximum_height_error: float,
) -> Dict[str, float]:
    first = instances[0]
    last = instances[-1]
    first_port_name = str(closure.get("first_input_port", "inlet"))
    target_port = sample_port(
        modules[last["module"]],
        last["output_port"],
        sample_count,
        aliases.get(last["module"], {}).get(last["output_port"]),
    )
    source_port = sample_port(
        modules[first["module"]],
        first_port_name,
        sample_count,
        aliases.get(first["module"], {}).get(first_port_name),
    )
    expected_pose, fitted = place_at_port(
        target_port,
        last["pose"],
        source_port,
        maximum_height_error,
    )
    position_error = float(np.linalg.norm(expected_pose.xy - first["pose"].xy))
    yaw_delta = float(expected_pose.yaw - first["pose"].yaw)
    yaw_error = abs(float(np.rad2deg(np.arctan2(np.sin(yaw_delta), np.cos(yaw_delta)))))

    target_local = np.asarray(target_port["points_xyz_m"], dtype=float)
    source_local = np.asarray(source_port["points_xyz_m"], dtype=float)
    if fitted["source_order"] == "reversed":
        source_local = source_local[::-1]
    target_world = transform_xyz(target_local, last["pose"])
    source_world = transform_xyz(source_local, first["pose"])
    plan_error = np.linalg.norm(target_world[:, :2] - source_world[:, :2], axis=1)
    height_error = np.abs(target_world[:, 2] - source_world[:, 2])
    target_inward = transform_direction(
        target_port["inward_direction_xy"], last["pose"]
    )
    source_inward = transform_direction(
        source_port["inward_direction_xy"], first["pose"]
    )
    closure_seam = {
        "index": len(seams),
        "from_instance": last["id"],
        "to_instance": first["id"],
        "target_port": target_port["id"],
        "source_port": source_port["id"],
        "source_order": fitted["source_order"],
        "maximum_height_error_m": float(height_error.max()),
        "rms_height_error_m": float(np.sqrt(np.mean(height_error**2))),
        "maximum_plan_error_m": float(plan_error.max()),
        "rms_plan_error_m": float(np.sqrt(np.mean(plan_error**2))),
        "inward_direction_dot": float(np.dot(target_inward, source_inward)),
        "target_inward_direction_xy": target_inward.tolist(),
        "source_inward_direction_xy": source_inward.tolist(),
        "points_xyz_m": target_world.tolist(),
    }
    maximum_position_error = float(closure.get("maximum_position_error_m", 0.005))
    maximum_yaw_error = float(closure.get("maximum_yaw_error_deg", 0.1))
    maximum_plan_error = float(closure.get("maximum_plan_error_m", 0.005))
    if (
        position_error > maximum_position_error
        or yaw_error > maximum_yaw_error
        or closure_seam["maximum_plan_error_m"] > maximum_plan_error
    ):
        raise ValueError(
            "Closed-loop mismatch: "
            f"position={position_error:.9f} m, yaw={yaw_error:.9f} deg, "
            f'plan={closure_seam["maximum_plan_error_m"]:.9f} m'
        )
    seams.append(closure_seam)
    return {
        "position_error_m": position_error,
        "yaw_error_deg": yaw_error,
        "maximum_plan_error_m": closure_seam["maximum_plan_error_m"],
        "maximum_height_error_m": closure_seam["maximum_height_error_m"],
    }


def generate(config_path: Path, output_root: Path) -> Dict[str, Any]:
    config = load_json(config_path)
    artifact_stem = str(config.get("artifact_stem", "pk3a_closed_loop"))
    registry_path = Path(config["port_registry"])
    if not registry_path.is_absolute():
        registry_path = package_root() / registry_path
    registry, modules = load_port_sources(registry_path)
    modules = dict(modules)
    course_lengths = config.get("course_axis_lengths_m", {})
    course_blends = config.get("course_axis_blend_modes", {})
    for source_name in config.get("course_axis_modules", []):
        oriented_name = f"{source_name}_C"
        modules[oriented_name] = orient_course_across_width(
            modules[source_name],
            oriented_name,
            course_lengths.get(source_name),
            course_blends.get(source_name, "linear"),
        )
    for source_name in config.get("mirror_y_modules", []):
        mirror_name = f"{source_name}_M"
        modules[mirror_name] = mirror_module_y(
            modules[source_name], mirror_name
        )
    for raw in config.get("turn_variants", []):
        source_name = str(raw["source"])
        variant_name = str(raw["name"])
        modules[variant_name] = _turn_variant_spec(
            modules[source_name],
            variant_name,
            float(raw["turn_deg"]),
        )
    if "half_straight" in config:
        modules["F_HALF"] = _half_straight_spec(config["half_straight"])
    for raw in config.get("custom_modules", []):
        custom = _half_straight_spec(raw)
        modules[custom.name] = custom
    aliases = {
        name: dict(raw.get("aliases", {}))
        for name, raw in registry["modules"].items()
    }
    if "half_straight" in config:
        aliases["F_HALF"] = {
            "inlet": "low_profile_in",
            "outlet": "low_profile_out",
            "right_edge": "right_constant_edge",
            "left_edge": "left_constant_edge",
        }
    for raw in config.get("custom_modules", []):
        name = str(raw["name"])
        aliases[name] = {
            "inlet": "constant_profile_in",
            "outlet": "constant_profile_out",
            "right_edge": "right_constant_edge",
            "left_edge": "left_constant_edge",
        }
    for source_name in config.get("course_axis_modules", []):
        aliases[f"{source_name}_C"] = {
            "inlet": "course_entry_profile",
            "outlet": "course_exit_profile",
            "right_edge": "drawing_start_cross_section",
            "left_edge": "drawing_end_cross_section",
        }
    for source_name in config.get("mirror_y_modules", []):
        aliases[f"{source_name}_M"] = dict(aliases[source_name])
    for raw in config.get("turn_variants", []):
        aliases[str(raw["name"])] = dict(aliases[str(raw["source"])])
    start = config["start_pose"]
    instances, seams = assemble_route(
        config["route"],
        modules,
        aliases,
        int(registry["sample_count"]),
        float(registry["tolerances"]["maximum_height_error_m"]),
        PlanPose(
            xy=np.asarray(start["xy_m"], dtype=float),
            yaw=np.deg2rad(float(start["yaw_deg"])),
        ),
    )
    closure_metrics = None
    if "closure" in config:
        closure_metrics = _append_closure_seam(
            config["closure"],
            instances,
            seams,
            modules,
            aliases,
            int(registry["sample_count"]),
            float(registry["tolerances"]["maximum_height_error_m"]),
        )
    start_zone = _start_zone_geometry(
        config.get("start_zone"),
        instances,
        modules,
    )
    maximum_course_tangent_error = annotate_seam_tangent_metrics(
        instances,
        seams,
        modules,
    )
    overlap_records = find_nonadjacent_overlaps(instances, modules)
    maximum_overlap = max(
        (item["overlap_area_m2"] for item in overlap_records),
        default=0.0,
    )
    maximum_allowed_overlap = float(
        config["generation"].get("maximum_nonadjacent_overlap_m2", 1.0e-6)
    )
    if maximum_overlap > maximum_allowed_overlap:
        worst = overlap_records[0]
        raise ValueError(
            "Non-adjacent footprint overlap "
            f'{maximum_overlap:.9f} m^2 between '
            f'{worst["first_instance"]} and {worst["second_instance"]}'
        )
    generation = config["generation"]
    visual_root = output_root / "assets" / "visual" / "modules"
    collision_root = output_root / "assets" / "collision" / "modules"
    model_root = output_root / "models"
    model_root.mkdir(parents=True, exist_ok=True)
    used_names = {item["module"] for item in instances}
    used_modules = {
        name: spec for name, spec in modules.items() if name in used_names
    }
    slope_metrics = {
        name: surface_slope_metrics(spec)
        for name, spec in used_modules.items()
    }
    assets, asset_xml = _asset_data(
        used_modules,
        float(generation.get("visual_spacing_m", DEFAULT_VISUAL_SPACING_M)),
        float(generation.get("collision_spacing_m", generation["cell_size_m"])),
        visual_root,
        collision_root,
    )
    bounds = _world_bounds(instances, modules)
    review_asset_root = output_root / "assets"
    review_asset_root.mkdir(parents=True, exist_ok=True)
    model_path = model_root / f"{artifact_stem}.xml"
    surface_contact = _surface_contact_settings(generation)
    contact_solver = _contact_solver_settings(generation)
    _write_standalone_module_models(
        model_root,
        used_modules,
        assets,
        surface_contact,
    )
    model_path.write_text(
        _model_xml(
            instances,
            seams,
            modules,
            assets,
            asset_xml,
            surface_contact,
            contact_solver,
            bounds,
            config["name"],
            start_zone,
        ),
        encoding="utf-8",
    )
    variant_counts = Counter(item["module"] for item in instances)
    counts = Counter(
        _source_module_name(modules[item["module"]]) for item in instances
    )
    cv_yaw_values = [
        item["pose"].yaw
        for item in instances
        if _source_module_name(modules[item["module"]]) == "3CV1"
    ]
    cv_yaw = np.unwrap(cv_yaw_values) if cv_yaw_values else np.asarray([])
    cv_group_turns = []
    active_group = []
    for item in instances:
        is_cv = _source_module_name(modules[item["module"]]) == "3CV1"
        if not is_cv:
            if active_group:
                cv_group_turns.append(float(sum(active_group)))
                active_group = []
            continue
        spec = modules[item["module"]]
        direction = 1.0
        if item["input_port"] == "outlet" and item["output_port"] == "inlet":
            direction = -1.0
        active_group.append(direction * float(spec.properties["turn_deg"]))
    if active_group:
        cv_group_turns.append(float(sum(active_group)))
    cv_turn_deg = float(sum(cv_group_turns))
    adjacent_axis_angles = [
        float(np.rad2deg(np.arccos(np.clip(abs(np.cos(
            instances[index]["pose"].yaw
            - instances[index - 1]["pose"].yaw
        )), 0.0, 1.0))))
        for index in range(1, len(instances))
    ]
    result = {
        "schema_version": 4,
        "artifact_stem": artifact_stem,
        "name": config["name"],
        "track_role": str(config.get("track_role", "prototype")),
        "model": str(model_path),
        "instance_count": len(instances),
        "seam_count": len(seams),
        "closed_loop": closure_metrics is not None,
        "closure": closure_metrics,
        "module_counts": dict(counts),
        "module_variant_counts": dict(variant_counts),
        "nonadjacent_overlap_count": len(overlap_records),
        "maximum_nonadjacent_overlap_area_m2": maximum_overlap,
        "nonadjacent_overlaps": overlap_records,
        "maximum_analytic_seam_height_error_m": max(
            seam["maximum_height_error_m"] for seam in seams
        ),
        "maximum_analytic_seam_plan_error_m": max(
            seam["maximum_plan_error_m"] for seam in seams
        ),
        "maximum_course_tangent_error_deg": maximum_course_tangent_error,
        "module_slope_metrics": slope_metrics,
        "cv_pose_yaw_span_deg": (
            float(np.rad2deg(cv_yaw[-1] - cv_yaw[0]))
            if len(cv_yaw) >= 2
            else 0.0
        ),
        "cv_boundary_turn_deg": cv_turn_deg,
        "cv_group_turns_deg": cv_group_turns,
        "maximum_adjacent_axis_angle_deg": max(adjacent_axis_angles),
        "adjacent_axis_angles_deg": adjacent_axis_angles,
        "world_bounds_xy_m": bounds.tolist(),
        "start_zone": _serializable_start_zone(start_zone),
        "surface_material": dict(config.get("surface_material", {})),
        "surface_contact": surface_contact,
        "contact_solver": contact_solver,
        "geometry_pipeline": "modular_watertight_visual_obj_with_hidden_coarse_hfield",
        "expanded_visual_face_count": int(sum(
            assets[item["module"]]["visual_face_count"]
            for item in instances
        )),
        "instances": [_serializable_instance(item) for item in instances],
        "seams": seams,
        "asset_rasters": {
            name: {
                "shape": value["raster_shape"],
                "actual_cell_size_m": value["actual_cell_size_m"],
                "minimum_height_m": value["minimum"],
                "maximum_height_m": value["maximum"],
                "base_depth_m": value["base_depth"],
                "outside_sample_count": value["outside_sample_count"],
                "collision_halo_sample_count": value[
                    "collision_halo_sample_count"
                ],
            }
            for name, value in assets.items()
        },
        "module_solids": {
            name: {
                "visual_vertex_count": value["visual_vertex_count"],
                "visual_face_count": value["visual_face_count"],
                "watertight": value["visual_watertight"],
                "winding_consistent": value["visual_winding_consistent"],
                "volume_m3": value["visual_volume_m3"],
                "bounds_xyz_m": value["visual_bounds_xyz_m"],
            }
            for name, value in assets.items()
        },
    }
    metadata_path = review_asset_root / f"{artifact_stem}_metadata.json"
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return result
